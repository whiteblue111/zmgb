#include "circle.hpp"
#include "zf_common_headfile.hpp" 
#include <opencv2/opencv.hpp>
using namespace cv;

int circle_enconder;
enum circle_type_e circle_type = CIRCLE_NONE;

float angle_begin;
float angle_current;
int   g_left_begin_turn_id = -1;  
static float CIRCLE_IN_Y_TH = 70.0f;   // 你可调：60~90  


//方便串口收发
const char *circle_type_name[CIRCLE_NUM] = {
        "CIRCLE_NONE",
        "CIRCLE_LEFT_BEGIN", "CIRCLE_RIGHT_BEGIN",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

// 编码器，用于防止一些重复触发等。


int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;
void make_left_border_fallback(float pts[][2], int* num, int max_len)  
{  
    if (!pts || !num) return;  
    if (*num > 0) return;  // 有左线就不兜底  
  
    int n = (max_len < POINTS_MAX_LEN) ? max_len : POINTS_MAX_LEN;  
    for (int i = 0; i < n; i++) {  
        pts[i][0] = 0.0f;          // 最左边界  
        pts[i][1] = (float)i;      // 按点序给个递增y（与你重采样序一致即可）  
    }  
    *num = n;  
}  


// void check_circle() 
// {
//     // 非圆环模式下，单边L角点, 单边长直道
//     if (circle_type == CIRCLE_NONE && Lpt_l_found && !Lpt_r_found && rpts_r_resample_num > 195 ) {
//         circle_type = CIRCLE_LEFT_BEGIN;
//     }
//     if (circle_type == CIRCLE_NONE && Lpt_r_found && !Lpt_l_found && rpts_l_resample_num > 195 ) {
//         circle_type = CIRCLE_RIGHT_BEGIN;
//     }
// }


// 找单调性转折点：  
// mode = -1 找“先减后增”(谷点), mode = +1 找“先增后减”(峰点)  
// 返回是否找到，out_id为转折点索引  
static bool find_monotonic_turn_x(  
    float pts[][2], int num,  
    int start_id, int end_id,  
    int mode,            // -1 valley, +1 peak  
    float dx_eps,        // 抑制抖动阈值，如 0.5~1.0  
    int min_run,         // 前后至少连续点数，如 3  
    int *out_id)  
{  
    if (!pts || !out_id) return false;  
    if (num < 8) return false;  
  
    if (start_id < 1) start_id = 1;  
    if (end_id > num - 2) end_id = num - 2;  
    if (start_id >= end_id) return false;  
  
    // 先算符号序列  
    // sign: -1 / 0 / +1  
    static int sgn[POINTS_MAX_LEN];  
    for (int i = start_id; i <= end_id; i++) {  
        float dx = pts[i+1][0] - pts[i][0];  
        if (dx > dx_eps) sgn[i] = +1;  
        else if (dx < -dx_eps) sgn[i] = -1;  
        else sgn[i] = 0;  
    }  
  
    // 扫描翻转点  
    for (int i = start_id + min_run; i <= end_id - min_run; i++) {  
        // 跳过0平台  
        int l = i - 1, r = i;  
        while (l >= start_id && sgn[l] == 0) l--;  
        while (r <= end_id   && sgn[r] == 0) r++;  
        if (l < start_id || r > end_id) continue;  
  
        bool is_turn = false;  
        if (mode == -1) is_turn = (sgn[l] < 0 && sgn[r] > 0); // 谷  
        else            is_turn = (sgn[l] > 0 && sgn[r] < 0); // 峰  
        if (!is_turn) continue;  
  
        // 连续性校验：左侧 min_run 个趋势一致，右侧 min_run 个趋势一致  
        int okL = 0, okR = 0;  
        for (int k = i - min_run; k < i; k++) {  
            if (mode == -1) { if (sgn[k] <= 0) okL++; } // 谷点左侧应偏负  
            else            { if (sgn[k] >= 0) okL++; } // 峰点左侧应偏正  
        }  
        for (int k = i; k < i + min_run; k++) {  
            if (mode == -1) { if (sgn[k] >= 0) okR++; } // 谷点右侧应偏正  
            else            { if (sgn[k] <= 0) okR++; } // 峰点右侧应偏负  
        }  
  
        if (okL >= min_run - 1 && okR >= min_run - 1) {  
            *out_id = i;  
            return true;  
        }  
    }  
  
    return false;  
}  

//没加判定另一边是直道
void check_circle()
{
    if(circle_type == CIRCLE_NONE && angle_l_max > 75. / 180. *PI && angle_l_max_id >= 0 && angle_l_max_id < 45 && rpts_r_resample_num >195 ){
        circle_type = CIRCLE_LEFT_BEGIN; 
        angle_begin = g_angle_yaw;
    }
    if(circle_type == CIRCLE_NONE && angle_r_max > 75. / 180. *PI && angle_r_max_id < 45 && rpts_l_resample_num >195){
        circle_type = CIRCLE_RIGHT_BEGIN;
        angle_begin = g_angle_yaw;
    }
}

// void run_circle() {


//     // 左环开始，寻外直道右线
//     if (circle_type == CIRCLE_LEFT_BEGIN) {
//         track_type = TRACK_RIGHT;

//         //先丢左线后有线
//         if (ipts_l[0][1] < 80) { none_left_line++; }
//         if (ipts_l[0][1] > 93 && none_left_line > 2) {
//             have_left_line++;
//             if (have_left_line > 1) {
//                 circle_type = CIRCLE_LEFT_IN;
//                 none_left_line = 0;
//                 have_left_line = 0;

//             }
//         }
//     }
//     //入环，寻内圆左线
//     else if (circle_type == CIRCLE_LEFT_IN) {
//         track_type = TRACK_LEFT;

//         //编码器打表过1/4圆   应修正为右线为转弯无拐点
//         if (rpts_l_resample_num >50 ) 
//         { 
//             circle_type = CIRCLE_LEFT_RUNNING; 
//         }
//     }
//     //正常巡线，寻外圆右线
//     else if (circle_type == CIRCLE_LEFT_RUNNING) {
//         track_type = TRACK_RIGHT;

//         if (Lpt_r_found) rpts_r_resample_num = rpts_rc_num = Lpt_r_id;
//         //外环拐点(右L点)
//         if (Lpt_r_found && Lpt_r_id < 30) {
//             circle_type = CIRCLE_LEFT_OUT;
//         }
//     }
//     //出环，寻内圆
// //     else if (circle_type == CIRCLE_LEFT_OUT) {
// //         track_type = TRACK_LEFT;

// //         //右线为长直道
// //         if (is_straight_r) {
// //             circle_type = CIRCLE_LEFT_END;
// //         }
// //     }
// //     //走过圆环，寻右线
// //     else if (circle_type == CIRCLE_LEFT_END) {
// //         track_type = TRACK_RIGHT;

// //         //左线先丢后有
// //         if (rpts_l_resample_num < 0.2 / resample_dist) { none_left_line++; }
// //         if (rpts_l_resample_num > 1.0 / resample_dist && none_left_line > 3) {
// //             circle_type = CIRCLE_NONE;
// //             none_left_line = 0;
// //         }
// //     }
// //     //右环控制，前期寻左直道  
// // else if (circle_type == CIRCLE_RIGHT_BEGIN) {  
// //     track_type = TRACK_LEFT;  
  
// //     //先丢右线后有线  
// //     if (rpts_r_resample_num < 0.2 / resample_dist) { none_right_line++; }  
// //     if (rpts_r_resample_num > 1.0 / resample_dist && none_right_line > 2) {  
// //         have_right_line++;  
// //         if (have_right_line > 1) {  
// //             circle_type = CIRCLE_RIGHT_IN;  
// //             none_right_line = 0;  
// //             have_right_line = 0;    
// //         }  
// //     }  
// //     }  
// //     //入右环，寻右内圆环  
// //     else if (circle_type == CIRCLE_RIGHT_IN) {  
// //         track_type = TRACK_RIGHT;  
    
// //         //编码器打表过1/4圆，应修正为左线为转弯无拐点  
// //         if (rpts_r_resample_num < 0.1 / resample_dist ) {  
// //             circle_type = CIRCLE_RIGHT_RUNNING;  
// //         }  
// //     }  
// //     //正常巡线，寻外圆左线  
// //     else if (circle_type == CIRCLE_RIGHT_RUNNING) {  
// //         track_type = TRACK_LEFT;  
    
// //         //外环存在拐点（左L点）  
// //         if (Lpt_l_found) rpts_l_resample_num = rpts_lc_num = Lpt_l_id;  
// //         if (Lpt_l_found && Lpt_l_id < 0.4 / resample_dist) {  
// //             circle_type = CIRCLE_RIGHT_OUT;  
// //         }  
// //     }  
// //     //出环，寻内圆  
// //     else if (circle_type == CIRCLE_RIGHT_OUT) {  
// //         track_type = TRACK_RIGHT;  
    
// //         //建议后续改成“左右线都直”的联合判据  
// //         if (is_straight_l) {  
// //             circle_type = CIRCLE_RIGHT_END;  
// //         }  
// //     }  
// //     //走过圆环，寻左线  
// //     else if (circle_type == CIRCLE_RIGHT_END) {  
// //         track_type = TRACK_LEFT;  
    
// //         //右线先丢后有（与左环END对称）  
// //         if (rpts_r_resample_num < 0.2 / resample_dist) { none_right_line++; }  
// //         if (rpts_r_resample_num > 1.0 / resample_dist && none_right_line > 2) {  
// //             circle_type = CIRCLE_NONE;  
// //             none_right_line = 0;  
// //         }  
// //     }  

// }

void run_circle()  
{  
    // 防抖/防连跳计数器  
    static int begin_confirm_cnt = 0;   // BEGIN->IN 连续命中计数  
    static int in_hold_frames    = 0;   // IN 最小驻留帧数  
  
    // 左环开始：找左线单调转折点（先增后减，mode=+1）  
    if (circle_type == CIRCLE_LEFT_BEGIN)  
    {  
        track_type = TRACK_RIGHT;  
  
        int turn_id = -1;  
        bool found_turn = find_monotonic_turn_x(  
            ipts_l, ipts_l_num,  
            8, 80,          // 搜索区间  
            +1,             // 先增后减（峰点）  
            0.8f,           // dx_eps  
            3,              // min_run  
            &turn_id);  
  
        if (found_turn && turn_id >= 0 && turn_id < ipts_l_num)  
        {  
            g_left_begin_turn_id = turn_id;  
            float turn_y = ipts_l[turn_id][1];  
  
            // 这里按你当前定义：turn_y 大于阈值才进 IN  
            bool y_ok = (turn_y > CIRCLE_IN_Y_TH);  
  
            if (y_ok && angle_l_max < 75. / 180. *PI) begin_confirm_cnt++;  
            else      begin_confirm_cnt = 0;  
  
            // 连续2帧满足才切换，防止抖动  
            if (begin_confirm_cnt >= 2)  
            {  
                circle_type = CIRCLE_LEFT_IN;  
                begin_confirm_cnt = 0;  
                in_hold_frames = 0;   // 进入 IN 重置驻留计数  
            }  
        }  
        else  
        {  
            g_left_begin_turn_id = -1;  
            begin_confirm_cnt = 0;  
        }  
    }  
  
    // 入环：寻内圆左线  
    else if (circle_type == CIRCLE_LEFT_IN)  
    {  
        track_type = TRACK_LEFT;  
        in_hold_frames++;  
  
        // 防止刚进IN就立刻跳RUNNING：先驻留几帧  
        const int IN_MIN_HOLD_FRAMES = 8;  
        if (in_hold_frames >= IN_MIN_HOLD_FRAMES &&  (g_angle_yaw - angle_begin < -45.0f))  
        {  
            circle_type = CIRCLE_LEFT_RUNNING;  
            in_hold_frames = 0;  
        }  
    }  
  
    // 环内运行：寻外圆右线  
    else if (circle_type == CIRCLE_LEFT_RUNNING)  
    {  
        track_type = TRACK_RIGHT;  
  
        // if (angle_r_max > 75.0f / 180.0f * PI &&  
        //     angle_r_max_id >= 0 &&  
        //     angle_r_max_id < rpts_r_resample_num &&  
        //     angle_r_max_id < 30)  
        // {  
        //     circle_type = CIRCLE_LEFT_OUT;  
        // }  
        if(g_angle_yaw - angle_begin < -220){
            circle_type = CIRCLE_LEFT_OUT;
        }

    }  
  
    // 出环  
    else if (circle_type == CIRCLE_LEFT_OUT)  
    {  
        track_type = TRACK_LEFT;  
  
        if (g_angle_yaw - angle_begin < -355.0f)  
        {  
            circle_type = CIRCLE_LEFT_END;  
        }  
    }  
  
    // 结束过渡  
    else if (circle_type == CIRCLE_LEFT_END)  
    {  
        track_type = TRACK_RIGHT;  
        // 你后续可在这里补“回到 NONE”的条件  
    }  
}  

//     //右环控制，前期寻左直道  
// else if (circle_type == CIRCLE_RIGHT_BEGIN) {  
//     track_type = TRACK_LEFT;  
  
//     //先丢右线后有线  
//     if (rpts_r_resample_num < 0.2 / resample_dist) { none_right_line++; }  
//     if (rpts_r_resample_num > 1.0 / resample_dist && none_right_line > 2) {  
//         have_right_line++;  
//         if (have_right_line > 1) {  
//             circle_type = CIRCLE_RIGHT_IN;  
//             none_right_line = 0;  
//             have_right_line = 0;    
//         }  
//     }  
//     }  
//     //入右环，寻右内圆环  
//     else if (circle_type == CIRCLE_RIGHT_IN) {  
//         track_type = TRACK_RIGHT;  
    
//         //编码器打表过1/4圆，应修正为左线为转弯无拐点  
//         if (rpts_r_resample_num < 0.1 / resample_dist ) {  
//             circle_type = CIRCLE_RIGHT_RUNNING;  
//         }  
//     }  
//     //正常巡线，寻外圆左线  
//     else if (circle_type == CIRCLE_RIGHT_RUNNING) {  
//         track_type = TRACK_LEFT;  
    
//         //外环存在拐点（左L点）  
//         if (Lpt_l_found) rpts_l_resample_num = rpts_lc_num = Lpt_l_id;  
//         if (Lpt_l_found && Lpt_l_id < 0.4 / resample_dist) {  
//             circle_type = CIRCLE_RIGHT_OUT;  
//         }  
//     }  
//     //出环，寻内圆  
//     else if (circle_type == CIRCLE_RIGHT_OUT) {  
//         track_type = TRACK_RIGHT;  
    
//         //建议后续改成“左右线都直”的联合判据  
//         if (is_straight_l) {  
//             circle_type = CIRCLE_RIGHT_END;  
//         }  
//     }  
//     //走过圆环，寻左线  
//     else if (circle_type == CIRCLE_RIGHT_END) {  
//         track_type = TRACK_LEFT;  
    
//         //右线先丢后有（与左环END对称）  
//         if (rpts_r_resample_num < 0.2 / resample_dist) { none_right_line++; }  
//         if (rpts_r_resample_num > 1.0 / resample_dist && none_right_line > 2) {  
//             circle_type = CIRCLE_NONE;  
//             none_right_line = 0;  
//         }  
//     }  


#include "cross.hpp"  
#include "zf_common_headfile.hpp"  
#include <cmath>  
#include <cstdint>
#include <csignal>
#include <opencv2/opencv.hpp>
using namespace cv;
enum cross_type_e cross_type = CROSS_NONE;

const char *cross_type_name[CROSS_NUM] = {
        "CROSS_NONE",
        "CROSS_BEGIN", "CROSS_IN",
};

// 编码器值，用于防止一些重复触发等。
int64_t cross_encoder;

bool far_Lpt_l_found, far_Lpt_r_found;
int far_Lpt_l_id, far_Lpt_r_id;
extern volatile sig_atomic_t g_dbg_stage_id;
extern volatile sig_atomic_t g_dbg_frame_id;

extern float mapx[120][160];
extern float mapy[120][160];


// 以下定义为十字寻远线设定
//原图
float far_ipts_l[FAR_POINTS_MAX_LEN][2];
float far_ipts_r[FAR_POINTS_MAX_LEN][2];
int far_ipts_l_num, far_ipts_r_num;
//逆透视处理后
float far_rpts_l[FAR_POINTS_MAX_LEN][2];
float far_rpts_r[FAR_POINTS_MAX_LEN][2];
int far_rpts_l_num, far_rpts_r_num;
//滤波后
float far_rpts_l_blur[FAR_POINTS_MAX_LEN][2];
float far_rpts_r_blur[FAR_POINTS_MAX_LEN][2];
int far_rpts_l_blur_num,far_rpts_r_blur_num;
//等距采样后
float far_rpts_l_resample[FAR_POINTS_MAX_LEN][2];
float far_rpts_r_resample[FAR_POINTS_MAX_LEN][2];
int far_rpts_l_resample_num, far_rpts_r_resample_num;
//局部角度变化率
float far_angles_l[FAR_POINTS_MAX_LEN];
float far_angles_r[FAR_POINTS_MAX_LEN];
int far_angles_l_num, far_angles_r_num;
//非极大抑制后角度
float far_angles_nms_l[FAR_POINTS_MAX_LEN];
float far_angles_nms_r[FAR_POINTS_MAX_LEN];
int far_angles_nms_l_num, far_angles_nms_r_num;
//边线最大角度
float far_angle_l_max, far_angle_r_max;
int far_angle_l_max_id, far_angle_r_max_id;
//中线
float far_rpts_c[FAR_POINTS_MAX_LEN][2];
int far_rpts_c_num = 0;
//中线归一化
float far_rpts_c_same[FAR_POINTS_MAX_LEN][2];
int far_rpts_c_same_num = 0;
//中线等距采样
float far_rpts_c_resample[FAR_POINTS_MAX_LEN][2];
int far_rpts_c_resample_num = 0;
//单边巡线状态
track_type_e far_track_type = TRACK_LEFT; // 当前跟踪边线，初始默认跟踪左边

int not_have_line = 0;

//找远线起始点
int far_y1,far_y2 ; 
int far_x1 =UVC_WIDTH / 6,far_x2 =UVC_WIDTH - far_x1;
/**
 * @brief 十字补线函数（固定数组长度版）
 * @param pts_in 边线点数组
 * @param num 指向数组当前有效点数的指针（作为上限参考，不增加其值）
 * @param corner_index 角点索引（补线的起点）
 * @param dist 补线步长
 */
void supplement_line(float pts_in[][2], int* num, int corner_index, float dist) {
    // 1. 安全检查
    if (corner_index <= 1 || corner_index >= *num) return;

    // 2. 统计斜率（平均角度）
    float avg_angle = 0;
    for (int i = 0; i < corner_index - 1; i++) {
        float dx = pts_in[i + 1][0] - pts_in[i][0];
        float dy = pts_in[i + 1][1] - pts_in[i][1];
        avg_angle += -atan2f(dy, dx);
    }
    avg_angle /= (corner_index - 1);

    float start_x = pts_in[corner_index][0];
    float start_y = pts_in[corner_index][1];
    float abs_angle = fabs(avg_angle);

    // 3. 补线逻辑
    // 垂直趋势判定：45° ~ 135° (PI/4 ~ 3PI/4)
    if (abs_angle > PI / 4 && abs_angle < 3 * PI / 4) {
        int current_idx = corner_index;

        // --- 修改部分开始 ---
        // 取消了 start_y >= 0 的限制，使其能够补出图像之外的“虚拟点”
        // 循环直到 current_idx 达到数组的最大索引 (POINTS_MAX_LEN - 1)
        while (current_idx < (POINTS_MAX_LEN - 1)) {
            start_x += dist * (float)cos(avg_angle);
            start_y -= dist * (float)sin(avg_angle);

            current_idx++; // 移动到下一个位置
            pts_in[current_idx][0] = start_x;
            pts_in[current_idx][1] = start_y;
        }
        // 更新有效点数为填满后的总长度
        *num = POINTS_MAX_LEN;
        // --- 修改部分结束 ---
    } 
    else {
        // 水平趋势：从拐点坐标向下拉线，覆盖水平边线信息
        for (int i = 0; i < corner_index; i++)
        {
            pts_in[i][0] = pts_in[corner_index][0];
            pts_in[i][1] = pts_in[corner_index][1] + dist * (corner_index - i);
        }
    }
}
// 从角点向下补线（图像坐标系：y向下增大）  
// void supplement_line_down(float pts_in[][2], int* num, int corner_index, float dist) {  
//     // 1. 安全检查  
//     if (!pts_in || !num) return;  
//     if (corner_index <= 1 || corner_index >= *num) return;  
  
//     // 2. 统计斜率（平均角度）  
//     float avg_angle = 0.0f;  
//     for (int i = 0; i < corner_index - 1; i++) {  
//         float dx = pts_in[i + 1][0] - pts_in[i][0];  
//         float dy = pts_in[i + 1][1] - pts_in[i][1];  
//         avg_angle += -atan2f(dy, dx);  
//     }  
//     avg_angle /= (corner_index - 1);  
  
//     float start_x = pts_in[corner_index][0];  
//     float start_y = pts_in[corner_index][1];  
//     float abs_angle = fabsf(avg_angle);  
  
//     // 3. 补线逻辑  
//     // 垂直趋势判定：45° ~ 135°  
//     if (abs_angle > PI / 4 && abs_angle < 3 * PI / 4) {  
//         int current_idx = corner_index;  
  
//         // 从角点向下补（与原函数 y -= ... 相反）  
//         while (current_idx < (POINTS_MAX_LEN - 1)) {  
//             start_x += dist * cosf(avg_angle);  
//             start_y += dist * sinf(avg_angle);   // 向下关键：+  
  
//             current_idx++;  
//             pts_in[current_idx][0] = start_x;  
//             pts_in[current_idx][1] = start_y;  
//         }  
//         *num = POINTS_MAX_LEN;  
//     }   
//     else {  
//         // 水平趋势：从角点坐标向上/下拉一条，按你需求这里也改成向下  
//         for (int i = corner_index + 1; i < POINTS_MAX_LEN; i++) {  
//             pts_in[i][0] = pts_in[corner_index][0];  
//             pts_in[i][1] = pts_in[corner_index][1] + dist * (i - corner_index);  
//         }  
//         *num = POINTS_MAX_LEN;  
//     }  
// }  

//双L角点,切十字模式
void check_cross() {  
    bool l_ok = (angle_l_max_id >= 0 && angle_l_max_id < rpts_l_resample_num);  
    bool r_ok = (angle_r_max_id >= 0 && angle_r_max_id < rpts_r_resample_num);  
    if (!l_ok || !r_ok) return;  
    //找到两角点且合理
    bool Xfound = (angle_l_max > 45.0f/180.0f*PI) &&  
                  (angle_r_max > 45.0f/180.0f*PI) &&  
                  (rpts_l_resample[angle_l_max_id][1] > 60.0f) &&  
                  (rpts_r_resample[angle_r_max_id][1] > 60.0f);  
    //距离判据
    float dx = rpts_l_resample[angle_l_max_id][0] - rpts_r_resample[angle_r_max_id][0];  
    float dy = rpts_l_resample[angle_l_max_id][1] - rpts_r_resample[angle_r_max_id][1];  
    float corner_dist = sqrtf(dx * dx + dy * dy);  
    bool dist_right = (corner_dist > ROAD_WIDTH - 10.0f) && (corner_dist < ROAD_WIDTH + 10.0f);  
  
    if (cross_type == CROSS_NONE && Xfound && dist_right) {  
        cross_type = CROSS_BEGIN;  
    }  
}  
  
// 确保这些外部变量在你的项目中已经定义
// extern float aim_dist; 
// extern int track_type; 
// #define TRACK_LEFT 1
// #define TRACK_RIGHT 2

/**
 * @brief 十字状态机执行函数（含入十字截断与十字内远线决策）
 * @param img 传入原始图像用于寻远线
 * @return 无返回值
 * @note 在 CROSS_IN 状态下，track_type 与 far_track_type 保持一致，确保控制链使用远线决策
 */
void run_cross(cv::Mat img) {  
    // 1. 判断当前帧中是否找到了符合阈值的左右近端L角点
    bool l_ok = (angle_l_max_id >= 0 && angle_l_max_id < rpts_l_resample_num) && 
                (angle_l_max > 45.0f / 180.0f * PI);
    bool r_ok = (angle_r_max_id >= 0 && angle_r_max_id < rpts_r_resample_num) && 
                (angle_r_max > 45.0f / 180.0f * PI);
    
    bool Xfound = l_ok && r_ok;
    
    // 获取当前编码器值 (请根据你的龙芯/逐飞库替换具体的获取函数，例如 encoder_get(xxx))
    // int64_t current_encoder = get_total_encoder(); 

    // ==========================================
    // CROSS_BEGIN：检测到十字，但还没进入中心
    // ==========================================
    if (cross_type == CROSS_BEGIN) {  
        //先判断角点在截断，然后根据是否截断来巡线或许会好
        rpts_l_resample_num = angle_l_max_id;
        rpts_r_resample_num = angle_r_max_id;
        
        // 判断角点是否逼近车头，决定是否进入 CROSS_IN
        int close_threshold = 10;
        // 当角点距离车头很近时，切换状态
        if (Xfound && (angle_l_max_id < close_threshold || angle_r_max_id < close_threshold)) {
            cross_type = CROSS_IN;
            
            // 记录进入十字时的编码器值，可以用于后续超时保护
            // cross_encoder = current_encoder; 
            
            not_have_line = 0; // 重置丢线计数器，为致盲期做准备
        }
    }  

    // ==========================================
    // CROSS_IN：处于十字中心，近线致盲，依靠远线引导
    // ==========================================
    else if (cross_type == CROSS_IN) {  
        
        // 调用你已经写好的寻远线函数
        cross_farline(img); 
        // 状态机核心：CROSS_IN 仅使用远线选边结果
        track_type = far_track_type;
        // // 兜底防丢线逻辑：如果远角点没搜到，但是右边近线丢了，强制贴右
        // else if (not_have_line > 0 && rpts_r_resample_num < 5) { 
        //     track_type = TRACK_RIGHT; 
        // }  
        // else if (not_have_line > 0 && rpts_l_resample_num < 5) { 
        //     track_type = TRACK_LEFT; 
        // }  

        // 统计近线点数。如果两边点数都极少，说明此时车在十字正中间的“空白致盲区”
        if (rpts_l_resample_num < 5 && rpts_r_resample_num < 5) { 
            not_have_line++; 
        }
        
        // 退出十字条件：致盲期已过（连续多帧没线），且现在两边重新出现了长近线
        if (not_have_line > 3 && rpts_l_resample_num > 20 && rpts_r_resample_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }
    }  
}

/**
 * @brief 在远线 NMS 角度序列中筛选满足阈值的局部峰值角点
 * @param angles_raw     原始角度序列（弧度）
 * @param angles_nms     NMS 后角度序列（弧度）
 * @param num            角度序列有效长度（点）
 * @param max_scan       最大扫描点数上限（点）
 * @param idx_limit      角点索引上限（点）
 * @param conf_min       conf 最小阈值（弧度）
 * @param conf_max       conf 最大阈值（弧度）
 * @param corner_id_out  输出角点索引（点）
 * @return 是否找到有效角点（true=找到，false=未找到）
 * @note conf 使用原始角度邻域差：|a[i]| - (|a[i-1]|+|a[i+1]|)/2，仅在 NMS 非零候选中比较并取最大 conf
 */
static bool select_far_corner_from_nms(const float angles_raw[],
                                       const float angles_nms[],
                                       int num,
                                       int max_scan,
                                       int idx_limit,
                                       float conf_min,
                                       float conf_max,
                                       int *corner_id_out) {
    if (angles_raw == nullptr || angles_nms == nullptr || corner_id_out == nullptr || num <= 0) {
        return false;
    }

    const int scan_num = (num < max_scan) ? num : max_scan;
    int best_id = -1;
    float best_conf = -1.0f;

    for (int i = 0; i < scan_num; ++i) {
        if (i >= idx_limit) break;
        if (fabsf(angles_nms[i]) < 1e-4f) continue;

        const int im1 = (i > 0) ? (i - 1) : i;
        const int ip1 = (i + 1 < num) ? (i + 1) : i;
        const float conf = fabsf(angles_raw[i]) - (fabsf(angles_raw[im1]) + fabsf(angles_raw[ip1])) / 2.0f;

        if (conf >= conf_min && conf <= conf_max && conf > best_conf) {
            best_conf = conf;
            best_id = i;
        }
    }

    if (best_id >= 0) {
        *corner_id_out = best_id;
        return true;
    }
    return false;
}

  


// void run_cross(Mat img,float pts_l[][2],int num_l,float pts_r[][2],int num_r) 
// {
//     //检测到十字，先按照近线走
//     if (cross_type == CROSS_BEGIN) 
//     {

//         aim_dist = 0.4;
//         //近角点过少，进入远线控制
//         if (((angle_l_max_id < 30  || angle_r_max_id < 30))/* || (rpts1_num <30 && rpts0_num<30)*/) {
//             cross_type = CROSS_IN;
//         }
//     }
//         //远线控制进十字,begin_y渐变靠近防丢线
//     else if (cross_type == CROSS_IN) {
//         //寻远线,算法与近线相同
//         cross_farline(img);

//         if (num_l < 5 && num_r < 5) { not_have_line++; }
//         if (not_have_line > 2 && num_l > 20 && num_r > 20) {
//             cross_type = CROSS_NONE;
//             not_have_line = 0;
//         }
//         if (far_Lpt1_found) { track_type = TRACK_RIGHT; }
//         else if (far_Lpt0_found) { track_type = TRACK_LEFT; }
//         else if (not_have_line > 0 && num_r < 5) { track_type = TRACK_RIGHT; }
//         else if (not_have_line > 0 && num_l < 5) { track_type = TRACK_LEFT; }

//     }
// }



/**
 * @brief 十字远线提取与远线中线构建
 * @param img 传入二值图像（320x240 坐标系下裁剪后处理）
 * @return 无返回值
 * @note 函数内部会更新 far_track_type，并输出 far_rpts_c_resample 供 CROSS_IN 阶段控制使用
 */
void cross_farline(Mat img) 
{
    g_dbg_stage_id = 102;
    int begin_y = 115;//距离图像底部开始找远线起始点
    uint8_t* ptr = nullptr;
    bool white_found = false;
    far_ipts_l_num = sizeof(far_ipts_l) / sizeof(far_ipts_l[0]);
    //在begin_y向两边找黑线

    //全白  far_x1 = 0,从边界找
    for (int y =begin_y; y > 0; y--) {
        //先白后黑，先找white
        ptr = (uint8_t*)img.ptr(y);
        if (ptr [far_x1]>= 125) { white_found = true; }
        if (ptr [far_x1] < 125 && (white_found )) {
            far_y1 = y;
            break;
        }
    }
    ptr = (uint8_t*)img.ptr(far_y1+1);
    //从找到角点位置开始寻找
    if (ptr [far_x1] >= 125)
        findline_lefthand_adaptive(img, far_x1, far_y1 +1 ,  far_ipts_l, &far_ipts_l_num);
    else far_ipts_l_num = 0;

    int x2 = UVC_WIDTH / 2 + begin_x, y2 = begin_y;
    white_found = false;
    far_ipts_r_num = sizeof(far_ipts_r) / sizeof(far_ipts_r[0]);


    for (int y = begin_y; y > 0; y--) {
        uint8_t* ptr=img.ptr(y);
        //先白后黑，先找white
        if (ptr[far_x2] >= 125) { white_found = true; }
        if (ptr[far_x2] < 125 && (white_found)) {
            far_y2 = y;
            break;
        }
    }

    //从找到角点位置开始寻找
     ptr=img.ptr(far_y2+1);
    if (ptr[far_x2] >= 125)
        findline_righthand_adaptive(img,  far_x2, far_y2 + 1, far_ipts_r, &far_ipts_r_num);
    else far_ipts_r_num = 0;


    far_rpts_l_num = 0;
    far_rpts_r_num = 0;

// ====================================================================
    // ✨ 远线逆透视（查表极速版）与近线流程对齐
    // ====================================================================
    far_rpts_l_num = 0;
    far_rpts_r_num = 0;

    // -------- 1. 左边线查表映射 --------
    for (int i = 0; i < far_ipts_l_num; i++) 
    { 
        // 1. 获取原始图像坐标并取整（确保坐标在 160x120 范围内）
        int px = (int)(far_ipts_l[i][0] + 0.5f); 
        int py = (int)(far_ipts_l[i][1] + 0.5f);

        // 2. 边界检查，防止索引越界导致程序崩溃
        if (px >= 0 && px < 160 && py >= 0 && py < 120) 
        {
            // 3. 核心步骤：使用 g_ipm_valid 过滤掉无效点
            if (g_ipm_valid[py][px] && far_rpts_l_num < FAR_POINTS_MAX_LEN) 
            {   
                // 4. 直接映射到物理坐标
                far_rpts_l[far_rpts_l_num][0] = g_ipm_lut_u[py][px];  
                far_rpts_l[far_rpts_l_num][1] = g_ipm_lut_v[py][px];  
                far_rpts_l_num++;  
            } else if (g_ipm_valid[py][px] && far_rpts_l_num >= FAR_POINTS_MAX_LEN) {
                // #region agent log
                static int s_dbg_far_left_overflow_cnt = 0;
                if (s_dbg_far_left_overflow_cnt < 12) {
                    fprintf(stderr, "[termdbg] cross_farline left truncated frame=%d py=%d px=%d far_rpts_l_num=%d cap=%d\n",
                            (int)g_dbg_frame_id, py, px, far_rpts_l_num, FAR_POINTS_MAX_LEN);
                    s_dbg_far_left_overflow_cnt++;
                }
                // #endregion
            } 
        }
    }

    // -------- 2. 右边线查表映射 --------
    for (int i = 0; i < far_ipts_r_num; i++) 
    { 
        int px = (int)(far_ipts_r[i][0] + 0.5f);
        int py = (int)(far_ipts_r[i][1] + 0.5f);

        if (px >= 0 && px < 160 && py >= 0 && py < 120) 
        {
            if (g_ipm_valid[py][px] && far_rpts_r_num < FAR_POINTS_MAX_LEN) 
            {  
                far_rpts_r[far_rpts_r_num][0] = g_ipm_lut_u[py][px];  
                far_rpts_r[far_rpts_r_num][1] = g_ipm_lut_v[py][px];  
                far_rpts_r_num++;  
            } else if (g_ipm_valid[py][px] && far_rpts_r_num >= FAR_POINTS_MAX_LEN) {
                // #region agent log
                static int s_dbg_far_right_overflow_cnt = 0;
                if (s_dbg_far_right_overflow_cnt < 12) {
                    fprintf(stderr, "[termdbg] cross_farline right truncated frame=%d py=%d px=%d far_rpts_r_num=%d cap=%d\n",
                            (int)g_dbg_frame_id, py, px, far_rpts_r_num, FAR_POINTS_MAX_LEN);
                    s_dbg_far_right_overflow_cnt++;
                }
                // #endregion
            } 
        }
    } 
    g_dbg_stage_id = 103;

    // -------- 3. 远线逆透视后左右边线等距采样 (增加点数安全判断) --------
    far_rpts_l_resample_num = FAR_POINTS_MAX_LEN;  
    far_rpts_r_resample_num = FAR_POINTS_MAX_LEN;   

    if (far_rpts_l_num > 2) {  
        blur_points(far_rpts_l, far_rpts_l_num, far_rpts_l_blur, blur_dist);  
        // 注：此处统一使用近线的 resample_dist。如果远线需要更稀疏的采样，可改回 resample_dist * pixel_per_meter
        resample_points(far_rpts_l_blur, far_rpts_l_num, far_rpts_l_resample, &far_rpts_l_resample_num, resample_dist);  
    } else {  
        far_rpts_l_resample_num = 0;  
    }  

    if (far_rpts_r_num > 2) {  
        blur_points(far_rpts_r, far_rpts_r_num, far_rpts_r_blur, blur_dist);  
        resample_points(far_rpts_r_blur, far_rpts_r_num, far_rpts_r_resample, &far_rpts_r_resample_num, resample_dist);  
    } else {  
        far_rpts_r_resample_num = 0;  
    }  
    g_dbg_stage_id = 104;

    // -------- 4. 远线角度变化率 --------
    // 统一使用固定窗口值 5 对齐近线，简化参数计算
    local_angle_points(far_rpts_l_resample, far_rpts_l_resample_num, far_angles_l, angle_dist);
    nms_angle(far_angles_l, far_rpts_l_resample_num, far_angles_nms_l, 5);
    // 注意：这里的 max 变量名我给你加上了 far_ 前缀，以防和近线的 angle_l_max 冲突
    max_angle(far_angles_l, 50, &far_angle_l_max, &far_angle_l_max_id);

    local_angle_points(far_rpts_r_resample, far_rpts_r_resample_num, far_angles_r, angle_dist);
    nms_angle(far_angles_r, far_rpts_r_resample_num, far_angles_nms_r, 5);
    max_angle(far_angles_r, 50, &far_angle_r_max, &far_angle_r_max_id);
    g_dbg_stage_id = 105;

    // 简化后的远端 L 角点寻找逻辑（NMS 局部峰值 + conf 过滤）
    far_Lpt_l_found = far_Lpt_r_found = false;
    far_Lpt_l_id = -1;
    far_Lpt_r_id = -1;

    // NMS conf 阈值范围（70°~110°），回退角度阈值沿用原逻辑
    const float angle_min_threshold = 45.0f / 180.0f * PI;
    const float angle_max_threshold = 110.0f / 180.0f * PI;
    const float conf_min_threshold = 40.0f / 180.0f * PI;
    const float conf_max_threshold = 110.0f / 180.0f * PI;
    const int max_scan = 50;
    const int idx_limit = 50;

    // 1) 优先使用 NMS 局部峰值筛选左远线角点
    far_Lpt_l_found = select_far_corner_from_nms(
        far_angles_l, far_angles_nms_l, far_rpts_l_resample_num,
        max_scan, idx_limit,
        conf_min_threshold, conf_max_threshold,
        &far_Lpt_l_id);


    // 2) 优先使用 NMS 局部峰值筛选右远线角点
    far_Lpt_r_found = select_far_corner_from_nms(
        far_angles_r, far_angles_nms_r, far_rpts_r_resample_num,
        max_scan, idx_limit,
        conf_min_threshold, conf_max_threshold,
        &far_Lpt_r_id);
}
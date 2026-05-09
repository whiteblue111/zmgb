#include "cross.hpp"  
#include "zf_common_headfile.hpp"  
#include <cmath>  
#include <cstdint>
#include <opencv2/opencv.hpp>
using namespace cv;
enum cross_type_e cross_type = CROSS_NONE;

const char *cross_type_name[CROSS_NUM] = {
        "CROSS_NONE",
        "CROSS_BEGIN", "CROSS_IN",
};

// 编码器值，用于防止一些重复触发等。
int64_t cross_encoder;

bool far_Lpt0_found, far_Lpt1_found;
int far_Lpt_l_id, far_Lpt_r_id;

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
int far_y1= 0,far_y2 = 0; 
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
  
    bool Xfound = (angle_l_max > 65.0f/180.0f*PI) &&  
                  (angle_r_max > 65.0f/180.0f*PI) &&  
                  (rpts_l_resample[angle_l_max_id][1] > 60.0f) &&  
                  (rpts_r_resample[angle_r_max_id][1] > 60.0f);  
  
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
 * @brief 十字状态机执行函数
 * @param img 传入原始图像用于寻远线
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
        
        // 开源思路：进十字前按照近线走。直接把角点之后的线切断，防止被拐角带偏
        if (l_ok) {
            rpts_l_resample_num = angle_l_max_id;
        }
        if (r_ok) {
            rpts_r_resample_num = angle_r_max_id;
        }

        // 根据你的前瞻控制逻辑设置 aim_dist (可选)
        // aim_dist = 0.4;

        // 判断角点是否逼近车头，决定是否进入 CROSS_IN
        // 结合开源代码的思想：索引距离 < (0.1米 / 采样间距)
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

        // 统计近线点数。如果两边点数都极少，说明此时车在十字正中间的“空白致盲区”
        if (rpts_l_resample_num < 5 && rpts_r_resample_num < 5) { 
            not_have_line++; 
        }
        
        // 退出十字条件：致盲期已过（连续多帧没线），且现在两边重新出现了长直近线
        if (not_have_line > 2 && rpts_l_resample_num > 20 && rpts_r_resample_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }

        // 状态机核心：在致盲期使用远线的角点来强制决定巡哪边的线 (单边巡线)
        if (far_Lpt1_found) { 
            track_type = TRACK_RIGHT;  // 远端发现右角点，说明需要贴右边线过十字
        }  
        else if (far_Lpt0_found) { 
            track_type = TRACK_LEFT;   // 远端发现左角点，说明需要贴左边线过十字
        }  
        // // 兜底防丢线逻辑：如果远角点没搜到，但是右边近线丢了，强制贴右
        // else if (not_have_line > 0 && rpts_r_resample_num < 5) { 
        //     track_type = TRACK_RIGHT; 
        // }  
        // else if (not_have_line > 0 && rpts_l_resample_num < 5) { 
        //     track_type = TRACK_LEFT; 
        // }  
    }  
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



void cross_farline(Mat img) 
{
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
        //先黑后白，先找white
        if (ptr[far_x2] >= 125) { white_found = true; }
        if (ptr[far_x2] < 125 ) {
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
            if (g_ipm_valid[py][px] && far_rpts_l_num < POINTS_MAX_LEN) 
            {   
                // 4. 直接映射到物理坐标
                far_rpts_l[far_rpts_l_num][0] = g_ipm_lut_u[py][px];  
                far_rpts_l[far_rpts_l_num][1] = g_ipm_lut_v[py][px];  
                far_rpts_l_num++;  
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
            if (g_ipm_valid[py][px] && far_rpts_r_num < POINTS_MAX_LEN) 
            {  
                far_rpts_r[far_rpts_r_num][0] = g_ipm_lut_u[py][px];  
                far_rpts_r[far_rpts_r_num][1] = g_ipm_lut_v[py][px];  
                far_rpts_r_num++;  
            } 
        }
    } 

    // -------- 3. 远线逆透视后左右边线等距采样 (增加点数安全判断) --------
    far_rpts_l_resample_num = EDGELINE_MAX;  
    far_rpts_r_resample_num = EDGELINE_MAX;   

    if (far_rpts_l_num > 2) {  
        blur_points(far_rpts_l, far_rpts_l_num, far_rpts_l_blur, 5);  
        // 注：此处统一使用近线的 resample_dist。如果远线需要更稀疏的采样，可改回 resample_dist * pixel_per_meter
        resample_points(far_rpts_l_blur, far_rpts_l_num, far_rpts_l_resample, &far_rpts_l_resample_num, resample_dist);  
    } else {  
        far_rpts_l_resample_num = 0;  
    }  

    if (far_rpts_r_num > 2) {  
        blur_points(far_rpts_r, far_rpts_r_num, far_rpts_r_blur, 5);  
        resample_points(far_rpts_r_blur, far_rpts_r_num, far_rpts_r_resample, &far_rpts_r_resample_num, resample_dist);  
    } else {  
        far_rpts_r_resample_num = 0;  
    }  

    // -------- 4. 远线角度变化率 --------
    // 统一使用固定窗口值 5 对齐近线，简化参数计算
    local_angle_points(far_rpts_l_resample, far_rpts_l_resample_num, far_angles_l, 5);
    nms_angle(far_angles_l, far_rpts_l_resample_num, far_angles_nms_l, 5);
    // 注意：这里的 max 变量名我给你加上了 far_ 前缀，以防和近线的 angle_l_max 冲突
    max_angle(far_angles_l, 50, &far_angle_l_max, &far_angle_l_max_id);

    local_angle_points(far_rpts_r_resample, far_rpts_r_resample_num, far_angles_r, 5);
    nms_angle(far_angles_r, far_rpts_r_resample_num, far_angles_nms_r, 5);
    max_angle(far_angles_r, 50, &far_angle_r_max, &far_angle_r_max_id);

    // 简化后的远端 L 角点寻找逻辑
    far_Lpt0_found = far_Lpt1_found = false;

    // 定义角点判定的阈值范围
    const float angle_min_threshold = 50.0f / 180.0f * PI;
    const float angle_max_threshold = 110.0f / 180.0f * PI;

    // 1. 直接判断左边线最大角点
    if (far_angle_l_max > angle_min_threshold && 
        far_angle_l_max < angle_max_threshold && 
        far_angle_l_max_id < 80) { // 限制在前半段
        
        far_Lpt_l_id = far_angle_l_max_id;
        far_Lpt0_found = true;
    }

    // 2. 直接判断右边线最大角点
    if (far_angle_r_max > angle_min_threshold && 
        far_angle_r_max < angle_max_threshold && 
        far_angle_r_max_id < 80) {
        
        far_Lpt_r_id = far_angle_r_max_id;
        far_Lpt1_found = true;
    }
    // --- 远线截断逻辑：保留角点及以后的点 ---

    // 1. 处理左边线截断
    if (far_Lpt0_found && far_Lpt_l_id < far_rpts_l_resample_num) {
        // 计算剩余点数
        int remaining_num = far_rpts_l_resample_num - far_Lpt_l_id;
        
        // 将角点之后的数据移到数组开头
        for (int i = 0; i < remaining_num; i++) {
            far_rpts_l_resample[i][0] = far_rpts_l_resample[far_Lpt_l_id + i][0];
            far_rpts_l_resample[i][1] = far_rpts_l_resample[far_Lpt_l_id + i][1];
        }
        far_rpts_l_resample_num = remaining_num;
    }

    // 2. 处理右边线截断
    if (far_Lpt1_found && far_Lpt_r_id < far_rpts_r_resample_num) {
        int remaining_num = far_rpts_r_resample_num - far_Lpt_r_id;
        
        for (int i = 0; i < remaining_num; i++) {
            far_rpts_r_resample[i][0] = far_rpts_r_resample[far_Lpt_r_id + i][0];
            far_rpts_r_resample[i][1] = far_rpts_r_resample[far_Lpt_r_id + i][1];
        }
        far_rpts_r_resample_num = remaining_num;
    }

    // 3. 远线循迹 (基于截断后的新起始点)
    if (far_track_type == TRACK_LEFT) {
        track_leftline(far_rpts_l_resample, far_rpts_l_resample_num,
                    far_rpts_c, far_rpts_c_num,
                    angle_dist / resample_dist,
                    HALF_ROAD_WIDTH ); // 跟踪时保持在车道中心，距离为车道宽度的一半
    } 
    else if (far_track_type == TRACK_RIGHT) {
        track_rightline(far_rpts_r_resample, far_rpts_r_resample_num,
                        far_rpts_c, far_rpts_c_num,
                        angle_dist / resample_dist,
                        HALF_ROAD_WIDTH );
    }

    // 4. 后续处理：归一化与重采样
    normalize_midline_with_anchor(far_rpts_c, far_rpts_c_num, far_rpts_c_same, &far_rpts_c_same_num);

    far_rpts_c_resample_num = FAR_POINTS_MAX_LEN; 
    if (far_rpts_c_same_num > 1) {
        resample_points(far_rpts_c_same, far_rpts_c_same_num, 
                        far_rpts_c_resample, &far_rpts_c_resample_num, 
                        resample_dist);
    } else {
        far_rpts_c_resample_num = 0;
    }
}
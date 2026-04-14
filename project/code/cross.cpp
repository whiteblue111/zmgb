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

int not_have_line = 0;

//找远线起始点
int far_y1, far_y2;
int far_x1 =UVC_WIDTH / 5,far_x2 =UVC_WIDTH - far_x1;
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
//双L角点,切十字模式
void check_cross() {
    bool Xfound = angle_l_max >65. /180.*PI && angle_r_max > 65. / 180. * PI ;
    if (cross_type == CROSS_NONE && Xfound) cross_type = CROSS_BEGIN;
}

void run_cross(){
    if (cross_type == CROSS_BEGIN)
    {
        if (angle_l_max >65. /180.*PI)
        {   //左补线
            supplement_line(rpts_l_resample,&rpts_l_resample_num,angle_l_max_id,resample_dist);
        }
        if (angle_r_max >65. /180.*PI)
        {   //右补线
            supplement_line(rpts_r_resample,&rpts_r_resample_num,angle_r_max_id,resample_dist);
        }
        //选择最长边线跟踪中线
        if(rpts_l_resample_num > rpts_r_resample_num){
            track_type = TRACK_LEFT; //左边线最长
        }else if(rpts_l_resample_num < rpts_r_resample_num){
            track_type = TRACK_RIGHT; //右边线最长
        }
        return ;  
    }
    
 
    
}

// void run_cross(Mat img,float pts_l[][2],int num_l,float pts_r[][2],int num_r) 
// {
//     bool Xfound = Lpt_l_found && Lpt_r_found;
//     int64_t current_encoder = get_dist();
//     float Lpt_l_y = rpts_l_resample[Lpt_l_id][1];
//     float Lpt_r_y = rpts_r_resample[Lpt_r_id][1];
//     //检测到十字，先按照近线走
//     if (cross_type == CROSS_BEGIN) 
//     {

//         aim_dist = 0.4;
//         //近角点过少，进入远线控制
//         if ((Xfound && (Lpt_l_id < 0.1 / resample_dist || Lpt_r_id < 0.1 / resample_dist))/* || (rpts1_num <30 && rpts0_num<30)*/) {
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



// void cross_farline(Mat img) 
// {
//     int cross_width = 4;//极限边界
// //    far_x1 = cross_width, far_x2 = img_raw.width -cross_width;
//     far_y1 = 0, far_y2 = 0;
//     int x1 = UVC_WIDTH / 2 - begin_x, y1 = begin_y;
//     uint8_t* ptr = nullptr;
//     bool white_found = false;
//     far_ipts_l_num = sizeof(far_ipts_l) / sizeof(far_ipts_l[0]);
//     //在begin_y向两边找黑线

//     //全白  far_x1 = 0,从边界找
//     for (; y1 > 0; y1--) {
//         //先黑后白，先找white
//         ptr = (uint8_t*)img.ptr(y1);
//         if (ptr [far_x1]>= 125) { white_found = true; }
//         if (ptr [far_x1] < 125 && (white_found || far_x1 == cross_width)) {
//             far_y1 = y1;
//             break;
//         }
//     }
//     ptr = (uint8_t*)img.ptr(far_y1+1);
//     //从找到角点位置开始寻找
//     if (ptr [far_x1] >= 125)
//         findline_lefthand_adaptive(img, far_x1, far_y1 +1 ,  far_ipts_l, &far_ipts_l_num);
//     else far_ipts_l_num = 0;

//     int x2 = UVC_WIDTH / 2 + begin_x, y2 = begin_y;
//     white_found = false;
//     far_ipts_r_num = sizeof(far_ipts_r) / sizeof(far_ipts_r[0]);


//     for (; y2 > 0; y2--) {
//         uint8_t* ptr=img.ptr(y2);
//         //先黑后白，先找white
//         if (ptr[far_x2] >= 125) { white_found = true; }
//         if (ptr[far_x2] < 125 && (white_found || far_x2 == UVC_WIDTH - cross_width)) {
//             far_y2 = y2;
//             break;
//         }
//     }

//     //从找到角点位置开始寻找
//      ptr=img.ptr(far_y2+1);
//     if (ptr[far_x2] >= 125)
//         findline_righthand_adaptive(img,  far_x2, far_y2 + 1, far_ipts_r, &far_ipts_r_num);
//     else far_ipts_r_num = 0;


//     // 去畸变+透视变换
//     for (int i = 0; i < far_ipts_l_num; i++) {
//         far_rpts_l[i][0] = mapx[(int) far_ipts_l[i][1]][(int) far_ipts_l[i][0]];
//         far_rpts_l[i][1] = mapy[(int) far_ipts_l[i][1]][(int) far_ipts_l[i][0]];
//     }
//     far_rpts_l_num = far_ipts_l_num;
//     for (int i = 0; i < far_ipts_r_num; i++) {
//         far_rpts_r[i][0] = mapx[(int) far_ipts_r[i][1]][(int) far_ipts_r[i][0]];
//         far_rpts_r[i][1] = mapy[(int) far_ipts_r[i][1]][(int) far_ipts_r[i][0]];
//     }


//     // 边线滤波
//     blur_points(far_rpts_l, far_rpts_l_num, far_rpts_l_blur, blur_kernel);
//     blur_points(far_rpts_r, far_rpts_r_num, far_rpts_r_blur, blur_kernel);


//     // 边线等距采样
//     far_rpts_l_resample_num = sizeof(far_rpts_l_resample) / sizeof(far_rpts_l_resample[0]);
//     resample_points(far_rpts_l_blur, far_rpts_l_blur_num, far_rpts_l_resample, &far_rpts_l_resample_num, resample_dist * pixel_per_meter);
//     far_rpts_r_resample_num = sizeof(far_rpts_r_resample) / sizeof(far_rpts_r_resample[0]);
//     resample_points(far_rpts_r_blur, far_rpts_r_blur_num, far_rpts_r_resample, &far_rpts_r_resample_num, resample_dist * pixel_per_meter);
//     // 边线局部角度变化率
//     local_angle_points(far_rpts_l_resample, far_rpts_l_resample_num, far_angles_l, (int) round(angle_dist / resample_dist));
//     far_angles_l_num = far_rpts_l_resample_num;
//     local_angle_points(far_rpts_r_resample, far_rpts_r_resample_num, far_angles_r, (int) round(angle_dist / resample_dist));
//     far_angles_r_num = far_rpts_r_resample_num;

//     // 角度变化率非极大抑制
//     nms_angle(far_angles_l, far_angles_l_num, far_angles_nms_l, (int) round(angle_dist / resample_dist) * 2 + 1);
//     far_angles_nms_l_num = far_angles_l_num;
//     nms_angle(far_angles_r, far_angles_r_num, far_angles_nms_r, (int) round(angle_dist / resample_dist) * 2 + 1);
//     far_angles_nms_r_num = far_angles_r_num;

//     // 找远线上的L角点
//     far_Lpt0_found = far_Lpt1_found = false;
//     for (int i = 0; i < MIN(far_rpts_l_resample_num, 80); i++) {
//         if (far_angles_nms_l[i] == 0) continue;
//         int im1 = limit_int(i - (int) round(angle_dist / resample_dist), 0, far_rpts_l_resample_num - 1);
//         int ip1 = limit_int(i + (int) round(angle_dist / resample_dist), 0, far_rpts_l_resample_num - 1);
//         float conf = fabs(far_angles_l[i]) - (fabs(far_angles_l[im1]) + fabs(far_angles_l[ip1])) / 2;
//         if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
//             far_Lpt_l_id = i;
//             far_Lpt0_found = true;
//             break;
//         }
//     }
//     for (int i = 0; i < MIN(far_rpts_r_resample_num, 80); i++) {
//         if (far_angles_nms_r[i] == 0) continue;
//         int im1 = limit_int(i - (int) round(angle_dist / resample_dist), 0, far_rpts_r_resample_num - 1);
//         int ip1 = limit_int(i + (int) round(angle_dist / resample_dist), 0, far_rpts_r_resample_num - 1);
//         float conf = fabs(far_angles_r[i]) - (fabs(far_angles_r[im1]) + fabs(far_angles_r[ip1])) / 2;

//         if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
//             far_Lpt_r_id = i;
//             far_Lpt1_found = true;
//             break;
//         }
//     }
// }
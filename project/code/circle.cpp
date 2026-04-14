#include "circle.hpp"
#include "zf_common_headfile.hpp" 
#include <opencv2/opencv.hpp>
using namespace cv;

int circle_enconder;
enum circle_type_e circle_type = CIRCLE_NONE;

float angle_begin;
float angle_current;

//方便串口收发
const char *circle_type_name[CIRCLE_NUM] = {
        "CIRCLE_NONE",
        "CIRCLE_LEFT_BEGIN", "CIRCLE_RIGHT_BEGIN",
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

// 编码器，用于防止一些重复触发等。


int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

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

//没加判定另一边是直道
void check_circle()
{
    if(circle_type == CIRCLE_NONE && angle_l_max > 75. / 180. *PI && angle_l_max_id < 45 && rpts_r_resample_num >195 ){
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

void run_circle() {


    // 左环开始，寻外直道右线
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        track_type = TRACK_RIGHT;

        //先丢左线后有线
        if (ipts_l[0][1] < 80) { none_left_line++; }
        if (ipts_l[0][1] > 93 && none_left_line > 2) {
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_LEFT_IN;
                none_left_line = 0;
                have_left_line = 0;

            }
        }
    }
    //入环，寻内圆左线
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_LEFT;

        //编码器打表过1/4圆   应修正为右线为转弯无拐点
        if (g_angle_yaw - angle_begin < -40 ) 
        { 
            circle_type = CIRCLE_LEFT_RUNNING; 
        }
    }
    //正常巡线，寻外圆右线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;

        // if (Lpt_r_found) rpts_r_resample_num = rpts_rc_num = Lpt_r_id;
        //外环拐点(右L点)
        if (angle_r_max > 75. / 180. * PI && angle_r_max_id < 30) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;

        //右线为长直道
        if (g_angle_yaw - angle_begin < -320) {
            circle_type = CIRCLE_LEFT_END;
        }
    }
    //走过圆环，寻右线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;

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


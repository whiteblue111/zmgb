#ifndef CIRCLE_HPP 
#define CIRCLE_HPP
#include <opencv2/opencv.hpp>
using namespace cv;

enum circle_type_e {
    CIRCLE_NONE = 0,                            // 非圆环模式
    CIRCLE_LEFT_BEGIN, CIRCLE_RIGHT_BEGIN,      // 圆环开始，识别到单侧L角点另一侧长直道。
    CIRCLE_LEFT_IN, CIRCLE_RIGHT_IN,            // 圆环进入，即走到一侧直道，一侧圆环的位置。
    CIRCLE_LEFT_RUNNING, CIRCLE_RIGHT_RUNNING,  // 圆环内部。
    CIRCLE_LEFT_OUT, CIRCLE_RIGHT_OUT,          // 准备出圆环，即识别到出环处的L角点。
    CIRCLE_LEFT_END, CIRCLE_RIGHT_END,          // 圆环结束，即再次走到单侧直道的位置。
    CIRCLE_NUM,                                 //
};
extern int   g_left_begin_turn_id;
extern const char *circle_type_name[CIRCLE_NUM];
extern enum circle_type_e circle_type;
extern float angle_begin;
extern void make_left_border_fallback(float pts[][2], int* num, int max_len)  ;
static bool find_monotonic_turn_x(  
    float pts[][2], int num,  
    int start_id, int end_id,  
    int mode,            // -1 valley, +1 peak  
    float dx_eps,        // 抑制抖动阈值，如 0.5~1.0  
    int min_run,         // 前后至少连续点数，如 3  
    int *out_id)  ;
void check_circle();
void run_circle();
#endif
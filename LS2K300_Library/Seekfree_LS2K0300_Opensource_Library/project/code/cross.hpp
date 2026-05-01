#ifndef CROSS_HPP 
#define CROSS_HPP
#include <opencv2/opencv.hpp>
using namespace cv;
enum cross_type_e {
    CROSS_NONE = 0,     // 非十字模式
    CROSS_BEGIN,        // 找到左右两个L角点
    CROSS_IN,           // 两个L角点很近，即进入十字内部(此时切换远线控制)
    CROSS_NUM,
};
extern enum cross_type_e cross_type ;
#define FAR_POINTS_MAX_LEN 100
extern const char *cross_type_name[CROSS_NUM];

extern float begin_x;               // 起始点距离图像中心的左右偏移量
extern float begin_y;               // 起始点距离图像底部的上下偏移
#define PI 3.14
void check_cross();
// void run_cross(Mat img,float pts_l[][2],int num_l,float pts_r[][2],int num_r);
void supplement_line_down(float pts_in[][2], int* num, int corner_index, float dist);
void run_cross();
void cross_farline(Mat img);
#endif
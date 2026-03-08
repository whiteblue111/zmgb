#ifndef IMGPROC_HPP  
#define IMGPROC_HPP  
  
#include <vector>  
#include <opencv2/opencv.hpp>  
using namespace cv;
  
#define EDGELINE_MAX   (200)
typedef struct Edge_line{
    int line[EDGELINE_MAX][2];
    int len;
}Edge_line;
void find_left_base(Mat img,int *x, int *y);
void find_right_base(Mat img,int *x, int *y);
void findline_lefthand_adaptive(Mat img, int x, int y, int pts[][2], int *num);
void findline_righthand_adaptive(const cv::Mat& img, int x, int y, int pts[][2], int *num);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) ;
#endif  
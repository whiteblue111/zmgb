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
void findline_lefthand_adaptive(Mat img, int x, int y, float pts[][2], int *num);
void findline_righthand_adaptive(Mat img, int x, int y, float pts[][2], int *num);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) ;
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) ;
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) ;
void map_points_to_ipm( float in_pts[][2], int in_num,  float out_pts[][2], int* out_num,  const float Hm[3][3]  )  ;
void add_black_border(cv::Mat &bin, int thickness);  
void compress_line_one_point_per_row(const float in_pts[][2], int in_num,  
                                     float out_pts[][2], int *out_num,  
                                     bool is_left_line)  ;
void build_midline_from_compressed_lr(const float left_pts[][2], int left_num,  
                                      const float right_pts[][2], int right_num,  
                                      float mid_pts[][2], int *mid_num)  ;                                    

#endif  
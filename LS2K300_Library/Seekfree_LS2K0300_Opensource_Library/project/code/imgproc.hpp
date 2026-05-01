#ifndef IMGPROC_HPP  
#define IMGPROC_HPP  
  
#include <vector>  
#include <opencv2/opencv.hpp>  
using namespace cv;
  
#define EDGELINE_MAX   (200)
#define POINTS_MAX_LEN (200)
#define ROAD_WIDTH      (76)
#define HALF_ROAD_WIDTH (38)

#define CENTER_BEGIN_X 80
#define CENTER_BEGIN_Y 112


typedef struct Edge_line{
    int line[EDGELINE_MAX][2];
    int len;
}Edge_line;
static inline uint8_t safe_pixel(const cv::Mat& img, int x, int y);
void find_left_base(Mat img,int *x, int *y);
void find_right_base(Mat img,int *x, int *y);
void findline_lefthand_adaptive(Mat img, int x, int y, float pts[][2], int *num);
void findline_righthand_adaptive(Mat img, int x, int y, float pts[][2], int *num);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) ;
void track_rightline(float pts_in[][2], int num_in, float pts_out[][2], int& num_out, int approx_num, float dist);
void track_leftline(float pts_in[][2], int num_in, float pts_out[][2],int& num_out, int approx_num, float dist)   ;
void normalize_midline_with_anchor(float pts_in[][2], int in_num, float pts_out[][2], int *out_num);
void map_points_to_ipm( float in_pts[][2], int in_num,  float out_pts[][2], int* out_num,  const float Hm[3][3]  )  ;
void add_black_border(cv::Mat &bin, int thickness);  
void add_black_border_half(cv::Mat &bin, int thickness);
void compress_line_one_point_per_row(const float in_pts[][2], int in_num,  
                                     float out_pts[][2], int *out_num,  
                                     bool is_left_line)  ;
void build_midline_from_compressed_lr(const float left_pts[][2], int left_num,  
                                      const float right_pts[][2], int right_num,  
                                      float mid_pts[][2], int *mid_num)  ; 
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);  
void max_angle(float angle_in[], int num, float *angle_max, int *idx);
void img_err_get();
bool calc_weighted_center_point(float Mline[][2], int num, float *out_x, float *out_y)  ;
void find_corners();
extern float aim_dist;          // 预锚点距离
extern float resample_dist;           //等距采样距离
extern float angle_dist;            //角度变化率采样距离
extern int aim_id;                   //预瞄点编号
enum track_type_e {
    TRACK_LEFT,
    TRACK_RIGHT,
};
extern enum track_type_e track_type;
extern float pixel_per_meter;       // 俯视图中，每个像素对应的长度
extern int blur_kernel;
extern bool Lpt_l_found,Lpt_r_found;//角点
extern int Lpt_l_id, Lpt_r_id;
extern bool is_straight_l,is_straight_r;



// ==================== 原图左线 ====================
extern float ipts_l[POINTS_MAX_LEN][2];
extern int ipts_l_num;

// ==================== 原图右线 ====================
extern float ipts_r[POINTS_MAX_LEN][2];
extern int ipts_r_num;

// ==================== 逆透视变换后 ====================
// 变换后左线
extern float rpts_l[POINTS_MAX_LEN][2];
extern int rpts_l_num;

// 变换后右线
extern float rpts_r[POINTS_MAX_LEN][2];
extern int rpts_r_num;

// ==================== 滤波后 ====================
// 滤波后左线
extern float rpts_l_blur[POINTS_MAX_LEN][2];
extern int rpts_l_blur_num;

// 滤波后右线
extern float rpts_r_blur[POINTS_MAX_LEN][2];
extern int rpts_r_blur_num;

// ==================== 等距采样后 ====================
// 采样后左线
extern float rpts_l_resample[POINTS_MAX_LEN][2];
extern int rpts_l_resample_num;

// 采样后右线
extern float rpts_r_resample[POINTS_MAX_LEN][2];
extern int rpts_r_resample_num;

// ==================== 局部角度变化率 ====================
// 左线角度变化率
extern float angles_l[POINTS_MAX_LEN];
extern int angles_l_num;

// 右线角度变化率
extern float angles_r[POINTS_MAX_LEN];
extern int angles_r_num;

// ==================== 非极大抑制后角度 ====================
// 左线角度（NMS后）
extern float angles_nms_l[POINTS_MAX_LEN];
extern int angles_nms_l_num;

// 右线角度（NMS后）
extern float angles_nms_r[POINTS_MAX_LEN];
extern int angles_nms_r_num;

//=======================角度最大值===================================
extern float angle_l_max;
extern float angle_r_max;
extern int   angle_l_max_id;
extern int   angle_r_max_id;

// ==================== 推中线 ====================
// 从左线推出的中线
extern float rpts_lc[POINTS_MAX_LEN][2];
extern int rpts_lc_num;

// 从右线推出的中线
extern float rpts_rc[POINTS_MAX_LEN][2];
extern int rpts_rc_num;

// 中线
extern float rpts_c[POINTS_MAX_LEN][2];
extern int rpts_c_num;
//归一化中线
extern float rpts_c_same[POINTS_MAX_LEN][2];
extern int rpts_c_same_num;
//等距采样中线
extern float rpts_c_resample[POINTS_MAX_LEN][2];
extern int rpts_c_resample_num;


extern float img_err;

#endif  
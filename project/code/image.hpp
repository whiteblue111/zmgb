#ifndef IMAGE_HPP  
#define IMAGE_HPP  
  
#include <vector>  
#include <opencv2/opencv.hpp>  
class ImageProcessor {  
private:  
    // 预计算的畸变映射表  
    cv::Mat map1;  
    cv::Mat map2;  
  
public:  
    ImageProcessor() = default;  
    ~ImageProcessor() = default;  
  
    /**  
     * @brief 初始化图像处理模块（计算标定矩阵和映射表）  
     * @param width 当前运行的图像宽度 (UVC_WIDTH)  
     * @param height 当前运行的图像高度 (UVC_HEIGHT)  
     * @param calib_base_width 标定时使用的图像宽度（默认假设为2048.0）  
     */  
    void init(int width, int height, double calib_base_width = 2048.0);  
  
    /**  
     * @brief 执行图像处理核心逻辑  
     * @param input_rgb565 输入的 RGB565 图像原始指针  
     * @param output_binary 输出的 二值化 图像目标缓存  
     * @param width 图像宽度  
     * @param height 图像高度  
     */  
    void process(uint16_t* input_rgb565, uint8_t* output_binary, int width, int height);  
};  
  

// #define POINTS_MAX_LEN 500     //待定！！！
//     const int dir_front[4][2] = {{0, -1},
//                                  {1, 0},
//                                  {0, 1},
//                                  {-1, 0}};
//     const int dir_frontleft[4][2] = {{-1, -1},
//                                      {1, -1},
//                                      {1, 1},
//                                      {-1, 1}};
//     const int dir_frontright[4][2] = {{1, -1},
//                                       {1, 1},
//                                       {-1, 1},
//                                       {-1, -1}};
  
// // 定义坐标点结构体  
// struct POINT {  
//     int x, y;  
//     POINT(int _x, int _y) : x(_x), y(_y) {}  
// };  
  
// // 函数声明  
// void findline_righthand_adaptive(cv::Mat &img, int block_size,  
//                                  int clip_value, int x, int y,  
//                                  std::vector<POINT> &pointsEdgeRight,  
//                                  int &pointsEdgeRight_size);  
  
 #endif  

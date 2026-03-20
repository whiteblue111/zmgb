 #include "zf_common_headfile.hpp"
 #include "image.hpp"
 #include <opencv2/opencv.hpp>  
 #include <cstdint>  
 using namespace cv;

  
void image_process(uint16_t* input_rgb565, uint8_t* output_binary, int width, int height) {  
    // A. 获取原始数据并转灰度  
    cv::Mat rgb565_frame(height, width, CV_8UC2, input_rgb565);  
    cv::Mat gray_frame;  
    cv::cvtColor(rgb565_frame, gray_frame, cv::COLOR_BGR5652GRAY);  
  
    // B. 滤波去噪  
    cv::GaussianBlur(gray_frame, gray_frame, cv::Size(3, 3), 0);  
  
    // C. 大津法二值化  
    cv::Mat binary_frame;  
    cv::threshold(gray_frame, binary_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);  
  
    // D. 拷贝到输出缓存区  
    memcpy(output_binary, binary_frame.data, width * height);  
}  

// // 逆透视变换矩阵(原图->俯视)

// float H[3][3] =  
// {  
//  {     2.1517,     1.9278,  -104.4764},
// {    -0.3644,     4.9223,  -134.9300},
// {    -0.0033,     0.0245,     1.0000}
// };  



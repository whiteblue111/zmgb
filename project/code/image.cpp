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

// 逆透视变换矩阵(原图->俯视)

// ====================== 逆透视矩阵（原图 -> 俯视） ======================  
const float H_IPM[3][3] = {
    { 10.6452f,  12.7419f, -735.0000f },
    { -0.0000f,  25.3226f, -855.0000f },
    {  0.0000f,   0.1613f,    1.0000f }
};  
//更改后重新标定赛道宽度

  
// 原图点 -> 俯视图点  
 bool warp_point_ipm(float x, float y, float &u, float &v)  
{  
    float w = H_IPM[2][0] * x + H_IPM[2][1] * y + H_IPM[2][2];  
    if (fabsf(w) < 1e-6f) return false;  
  
    u = (H_IPM[0][0] * x + H_IPM[0][1] * y + H_IPM[0][2]) / w;  
    v = (H_IPM[1][0] * x + H_IPM[1][1] * y + H_IPM[1][2]) / w;  
    return true;  
}  




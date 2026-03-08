 #include "zf_common_headfile.hpp"
 #include "image.hpp"
 #include <opencv2/opencv.hpp>  
 #include <cstdint>  
 using namespace cv;
 void ImageProcessor::init(int width, int height, double calib_base_width) {  
    // 计算缩放比例  
    double scale = (double)width / calib_base_width;  
  
    // 配置相机内参矩阵 (根据传入的缩放比例自适应)  
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<       
        690.7573 * scale, 0.0,              1025.8 * scale,       
        0.0,              688.5519 * scale, 676.0573 * scale,     
        0.0,              0.0,              1.0        
    );      
      
    // 畸变系数  
    Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.2038, -0.1764, 0.0, 0.0, 0.0);      
  
    // 预计算映射表  
    initUndistortRectifyMap(  
        cameraMatrix, distCoeffs, cv::Mat(),     
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(width, height), 1, Size(width, height), 0),    
        Size(width, height), CV_16SC2, map1, map2  
    );  
}  
  
void ImageProcessor::process(uint16_t* input_rgb565, uint8_t* output_binary, int width, int height) {  
    // A. 获取原始数据并转灰度    
    cv::Mat rgb565_frame(height, width, CV_8UC2, input_rgb565);        
    cv::Mat gray_frame;        
    cv::cvtColor(rgb565_frame, gray_frame, cv::COLOR_BGR5652GRAY);        
      
    // B. 去畸变（查表法，速度快）    
    // cv::Mat undistorted_frame;    
    // cv::remap(gray_frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);    
  
    // C. 滤波去噪（在拉直后的图上做）    
    cv::GaussianBlur(gray_frame, gray_frame, cv::Size(3, 3), 0);      
  
    // D. 大津法二值化    
    cv::Mat binary_frame;        
    cv::threshold(gray_frame, binary_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);      
  
    // E. 拷贝到输出缓存区  
    memcpy(output_binary, binary_frame.data, width * height);   
}  

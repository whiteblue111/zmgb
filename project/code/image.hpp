#ifndef IMAGE_HPP  
#define IMAGE_HPP  
  
#include <vector>  
#include <opencv2/opencv.hpp>  
void image_process(uint16_t* input_rgb565, uint8_t* output_binary, int width, int height);
extern const float H_IPM[3][3];  
bool warp_point_ipm(float x, float y, float &u, float &v);  
 
#endif  

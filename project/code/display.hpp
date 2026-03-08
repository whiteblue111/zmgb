#ifndef DISPLAY_HPP_  
#define DISPLAY_HPP_  
  
#include "zf_common_headfile.hpp"  
  
// 初始化显示模块（保存屏幕对象指针）  
void display_init(zf_device_ips200* ips);  
  
// 刷新显示：灰度图、二值图、左右线、计数与起点  
void display_show(  
    uint8 gray[][UVC_WIDTH],  
    uint8 bin[][UVC_WIDTH],  
    int left_pts[][2],  int left_num,  
    int right_pts[][2], int right_num,  
    int sx_l, int sy_l,  
    int sx_r, int sy_r  
);  
  
#endif  

#ifndef DISPLAY_HPP_  
#define DISPLAY_HPP_  
  
#include "zf_common_headfile.hpp"  
  
// 初始化显示模块（保存屏幕对象指针）  
void display_init(zf_device_ips200* ips);  
void draw_cross(int x, int y, uint16 color);
void display_show_overlay(int left_org_num, int right_org_num,  int sx_l, int sy_l, int sx_r, int sy_r,  float err_img);  

  
#endif  

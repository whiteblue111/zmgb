#include "display.hpp"  
#include "zf_common_headfile.hpp"
  
static zf_device_ips200* s_ips = nullptr;  
  
void display_init(zf_device_ips200* ips)  
{  
    s_ips = ips;  
}  
  
void display_show(  
    uint8 gray[][UVC_WIDTH],  
    uint8 bin[][UVC_WIDTH],  
    int left_pts[][2],  int left_num,  
    int right_pts[][2], int right_num,  
    int sx_l, int sy_l,  
    int sx_r, int sy_r  
)  
{  
    if (s_ips == nullptr) return;  
  
    s_ips->show_gray_image(0, 0,   gray[0], UVC_WIDTH, UVC_HEIGHT, UVC_WIDTH, UVC_HEIGHT, 0);  
    s_ips->show_gray_image(0, 125, bin[0],  UVC_WIDTH, UVC_HEIGHT, UVC_WIDTH, UVC_HEIGHT, 0);  
  
    s_ips->show_string(0, 250, "L:");  
    s_ips->show_int(20, 250, left_num, 3);  
    s_ips->show_string(70, 250, "R:");  
    s_ips->show_int(90, 250, right_num, 3);  
  
    for (int i = 0; i < left_num; i++) {  
        int x = left_pts[i][0], y = left_pts[i][1];  
        if (x <= 1 || x >= UVC_WIDTH - 2 || y <= 1 || y >= UVC_HEIGHT - 2) continue;  
        s_ips->draw_point(x, y, RGB565_RED);  
        s_ips->draw_point(x, y + 125, RGB565_RED);  
    }  
  
    for (int i = 0; i < right_num; i++) {  
        int x = right_pts[i][0], y = right_pts[i][1];  
        if (x <= 1 || x >= UVC_WIDTH - 2 || y <= 1 || y >= UVC_HEIGHT - 2) continue;  
        s_ips->draw_point(x, y, RGB565_BLUE);  
        s_ips->draw_point(x, y + 125, RGB565_BLUE);  
    }  
  
    if (sx_l > 2 && sx_l < UVC_WIDTH - 3 && sy_l > 2 && sy_l < UVC_HEIGHT - 3) {  
        s_ips->draw_line(sx_l - 2, sy_l, sx_l + 2, sy_l, RGB565_GREEN);  
        s_ips->draw_line(sx_l, sy_l - 2, sx_l, sy_l + 2, RGB565_GREEN);  
    }  
  
    if (sx_r > 2 && sx_r < UVC_WIDTH - 3 && sy_r > 2 && sy_r < UVC_HEIGHT - 3) {  
        s_ips->draw_line(sx_r - 2, sy_r, sx_r + 2, sy_r, RGB565_66CCFF);  
        s_ips->draw_line(sx_r, sy_r - 2, sx_r, sy_r + 2, RGB565_66CCFF);  
    }  
}  

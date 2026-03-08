#include "zf_common_headfile.hpp"  
#include "image.hpp"  
#include "imgproc.hpp"  
#include "display.hpp"  
#include <opencv2/opencv.hpp>  
#include <cstdio>  
  
// ====================== 配置项 ======================  
#define SERVER_IP "192.168.196.230"  
#define PORT      8080  
  
// ====================== 全局设备对象 ======================  
zf_driver_tcp_client tcp_client_dev;  
zf_device_uvc        uvc_dev;  
zf_device_ips200     ips200;  
  
uint16_t* rgb_ptr = nullptr;  
  
// 图像缓冲  
uint8 image_gray[UVC_HEIGHT][UVC_WIDTH];  
uint8 image_bin [UVC_HEIGHT][UVC_WIDTH];  
  
// 巡线点  
int left_pts[EDGELINE_MAX][2];  
int right_pts[EDGELINE_MAX][2];  
int left_num  = 0;  
int right_num = 0;  
  
// ====================== TCP包装（可选） ======================  
uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }  
uint32 tcp_read_wrap(uint8 *buf, uint32 len)       { return tcp_client_dev.read_data(buf, len); }  
  
int main()  
{  
    bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);  
    if (tcp_ok)  
    {  
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);  
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_gray[0], UVC_WIDTH, UVC_HEIGHT);  
    }  
  
    ips200.init(FB_PATH);  
    display_init(&ips200);   // 显示模块初始化  
  
    if (uvc_dev.init(UVC_PATH) < 0) return -1;  
  
    ImageProcessor img_processor;  
    img_processor.init(UVC_WIDTH, UVC_HEIGHT, 640);  
  
    while (1)  
    {  
        if (uvc_dev.wait_image_refresh() < 0) break;  
  
        rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();  
        if (rgb_ptr == nullptr) continue;  
  
        // 1) 图像处理输出灰度  
        img_processor.process(rgb_ptr, (uint8_t*)image_gray[0], UVC_WIDTH, UVC_HEIGHT);  
  
        // 2) 二值化  
        const uint8 th = 128;  
        for (int y = 0; y < UVC_HEIGHT; y++)  
        {  
            for (int x = 0; x < UVC_WIDTH; x++)  
            {  
                image_bin[y][x] = (image_gray[y][x] > th) ? 255 : 0;  
            }  
        }  
  
        // 3) 左右巡线  
        cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]);  
  
        int sx_l = UVC_WIDTH / 2, sy_l = UVC_HEIGHT - 5;  
        int sx_r = UVC_WIDTH / 2, sy_r = UVC_HEIGHT - 5;  
  
        find_left_base(bin_mat,  &sx_l, &sy_l);  
        find_right_base(bin_mat, &sx_r, &sy_r);  
  
        left_num  = EDGELINE_MAX;  
        right_num = EDGELINE_MAX;  
  
        findline_lefthand_adaptive (bin_mat, sx_l, sy_l, left_pts,  &left_num);  
        findline_righthand_adaptive(bin_mat, sx_r, sy_r, right_pts, &right_num);  
  
        // 4) 统一调用 display 库显示  
        display_show(image_gray, image_bin,  
                     left_pts, left_num,  
                     right_pts, right_num,  
                     sx_l, sy_l, sx_r, sy_r);  
  
        // 5) 上位机发送（可选）  
        if (tcp_ok) seekfree_assistant_camera_send();  
  
        system_delay_ms(20);  
    }  
  
    return 0;  
}  

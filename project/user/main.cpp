

// 纯视觉-----------------------------------------------------------------------------------------------------------------------------
// #include "zf_common_headfile.hpp"  
// #include "image.hpp"  
// #include "imgproc.hpp"  
// #include "display.hpp"  
// #include <opencv2/opencv.hpp>  
// #include <cstdio>  
// #include <cmath>  
  
// // ====================== 配置项 ======================  
// #define SERVER_IP "192.168.196.230"  
// #define PORT      8086  
  
// // ====================== 全局设备对象 ======================  
// zf_driver_tcp_client tcp_client_dev;  
// zf_device_uvc        uvc_dev;  
// zf_device_ips200     ips200;  
  
// uint16_t* rgb_ptr = nullptr;  
  
// // 图像缓冲  
// uint8 image_gray[UVC_HEIGHT][UVC_WIDTH];  
// uint8 image_bin [UVC_HEIGHT][UVC_WIDTH];  
// uint8 image_ipm [UVC_HEIGHT][UVC_WIDTH];   // 逆透视后二值图  
  
// // 巡线点（原图）  
// float left_pts[EDGELINE_MAX][2];  
// float right_pts[EDGELINE_MAX][2];  
// int left_num  = 0;  
// int right_num = 0;  
  
// // 巡线点（逆透视后）  
// float left_ipm_pts [EDGELINE_MAX][2];  
// float right_ipm_pts[EDGELINE_MAX][2];  
// int left_ipm_num  = 0;  
// int right_ipm_num = 0;  

// //中线
 
// int  dist_num = 1; //切线间隔点
// float   half_len = 40;
// float middle_ipm_pts[EDGELINE_MAX][2];  
// int   middle_ipm_num = 0;     

  
// // 来自 image.cpp 的逆透视矩阵（原图 -> 俯视）  
// extern float H[3][3];  
  
// // ====================== TCP包装（可选） ======================  
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }  
// uint32 tcp_read_wrap(uint8 *buf, uint32 len)       { return tcp_client_dev.read_data(buf, len); }  
  

  
// int main()  
// {  
//     bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);  
//     if (tcp_ok)  
//     {  
//         seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);  
//         seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_gray[0], UVC_WIDTH, UVC_HEIGHT);  
//     }  
  
//     ips200.init(FB_PATH);  
//     display_init(&ips200);  
  
//     if (uvc_dev.init(UVC_PATH) < 0) return -1;  
  
//     // OpenCV矩阵形式的H（只初始化一次）  
//     cv::Mat H_mat = (cv::Mat_<float>(3, 3) <<  
//         H[0][0], H[0][1], H[0][2],  
//         H[1][0], H[1][1], H[1][2],  
//         H[2][0], H[2][1], H[2][2]);  
  
//     while (1)  
//     {  
//         if (uvc_dev.wait_image_refresh() < 0) break;  
  
//         rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();  
//         if (rgb_ptr == nullptr) continue;  
  
//         // 1) 图像处理（你当前image_process里已做灰度+OTSU）  
//         image_process(rgb_ptr, (uint8_t*)image_gray[0], UVC_WIDTH, UVC_HEIGHT);  
  
//         // 2) 二值化（保留你原逻辑；如需可删）  
//         const uint8 th = 128;  
//         for (int y = 0; y < UVC_HEIGHT; y++)  
//         {  
//             for (int x = 0; x < UVC_WIDTH; x++)  
//             {  
//                 image_bin[y][x] = (image_gray[y][x] > th) ? 255 : 0;  
//             }  
//         }  
  
//         // 3) 原图巡线  
//         cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]);  
  
//         int sx_l = UVC_WIDTH / 2, sy_l = UVC_HEIGHT - 5;  
//         int sx_r = UVC_WIDTH / 2, sy_r = UVC_HEIGHT - 5;  
  
//         find_left_base(bin_mat,  &sx_l, &sy_l);  
//         find_right_base(bin_mat, &sx_r, &sy_r);  
  
//         left_num  = EDGELINE_MAX;  
//         right_num = EDGELINE_MAX;  
  
//         findline_lefthand_adaptive (bin_mat, sx_l, sy_l, left_pts,  &left_num);  
//         findline_righthand_adaptive(bin_mat, sx_r, sy_r, right_pts, &right_num);  
  
//         // 4) 逆透视图（用于下半屏显示）  
//         cv::Mat ipm_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_ipm[0]);  
//         cv::Mat H_inv = H_mat.inv();  
//         cv::warpPerspective(bin_mat, ipm_mat, H_mat, cv::Size(UVC_WIDTH, UVC_HEIGHT),  
//                     cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));  


  
//         // 5) 巡线点做逆透视映射（先巡线后逆透视）  
//         map_points_to_ipm(left_pts,  left_num,  left_ipm_pts,  &left_ipm_num,  H);  
//         map_points_to_ipm(right_pts, right_num, right_ipm_pts, &right_ipm_num, H); 
//         middle_ipm_num = left_ipm_num;  
//         track_leftline(left_ipm_pts, middle_ipm_num, middle_ipm_pts, dist_num, half_len);  

  
//         // 6) 显示：上原图二值+原边线，下IPM图+IPM边线  
//        display_show(  
//     image_bin, image_ipm,  
//     left_pts, left_num,  
//     right_pts, right_num,  
//     left_ipm_pts, left_ipm_num,  
//     right_ipm_pts, right_ipm_num,  
//     middle_ipm_pts, middle_ipm_num,   // 新增  
//     sx_l, sy_l, sx_r, sy_r  
// );  

  
//         // 7) 上位机发送（可选）  
//         if (tcp_ok) seekfree_assistant_camera_send();  
  
//         system_delay_ms(20);  
//     }  
  
//     return 0;  
// }  






// //陀螺仪pid

// #include "zf_common_headfile.hpp"  
// #include <csignal>  
// #include <cstdlib>  
// #include <cstdio>  
  
// /* ========================= 电机 ========================= */  
// #define PWM_1_PATH        ZF_PWM_MOTOR_1  
// #define DIR_1_PATH        ZF_GPIO_MOTOR_1  
// #define PWM_2_PATH        ZF_PWM_MOTOR_2  
// #define DIR_2_PATH        ZF_GPIO_MOTOR_2  
  
// zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);  
// zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);  
// zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);  
// zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);  
// struct pwm_info drv8701e_pwm_1_info;  
// struct pwm_info drv8701e_pwm_2_info;  
  
// #define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)  
// #define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)  
// #define MAX_DUTY               (30)  
  
// /* ========================= IMU ========================= */  
// zf_device_imu imu_dev;  
// int16 imu_gyro_z = 0;  
  
// /* ========================= PIT ========================= */  
// zf_driver_pit pit_timer;  
  
// /* ========================= 参数（先保证能动） ========================= */  
// float target_yaw_rate = 160.0f;   //   
// float kp = 0.18f;                // 小P  
// float ki = 0.0f;  
// float kd = 0.02f;  
// int base_duty = 10;              // 提高基础占空比，先保证能转  
// int dead_duty = 6;               // 最小有效占空比  
  
// float gyro_z_bias = 0.0f;  
// int   bias_cnt = 0;  
// bool  bias_ok = false;  
  
// float yaw_f = 0.0f;  
// float err_last = 0.0f;  
// float integ = 0.0f;  
  
// static inline int clamp_int(int x, int lo, int hi)  
// {  
//     if (x < lo) return lo;  
//     if (x > hi) return hi;  
//     return x;  
// }  
// static inline float clamp_float(float x, float lo, float hi)  
// {  
//     if (x < lo) return lo;  
//     if (x > hi) return hi;  
//     return x;  
// }  
  
// void motor_set_percent(int duty_l, int duty_r)  
// {  
//     duty_l = clamp_int(duty_l, -MAX_DUTY, MAX_DUTY);  
//     duty_r = clamp_int(duty_r, -MAX_DUTY, MAX_DUTY);  
  
//     // 死区补偿（0不补）  
//     if (duty_l > 0 && duty_l < dead_duty) duty_l = dead_duty;  
//     if (duty_l < 0 && duty_l > -dead_duty) duty_l = -dead_duty;  
//     if (duty_r > 0 && duty_r < dead_duty) duty_r = dead_duty;  
//     if (duty_r < 0 && duty_r > -dead_duty) duty_r = -dead_duty;  
  
//     // 左轮（PWM2）  
//     if (duty_l >= 0) {  
//         drv8701e_dir_2.set_level(1);  
//         drv8701e_pwm_2.set_duty(duty_l * MOTOR2_PWM_DUTY_MAX / 100);  
//     } else {  
//         drv8701e_dir_2.set_level(0);  
//         drv8701e_pwm_2.set_duty((-duty_l) * MOTOR2_PWM_DUTY_MAX / 100);  
//     }  
  
//     // 右轮（PWM1）  
//     if (duty_r >= 0) {  
//         drv8701e_dir_1.set_level(1);  
//         drv8701e_pwm_1.set_duty(duty_r * MOTOR1_PWM_DUTY_MAX / 100);  
//     } else {  
//         drv8701e_dir_1.set_level(0);  
//         drv8701e_pwm_1.set_duty((-duty_r) * MOTOR1_PWM_DUTY_MAX / 100);  
//     }  
// }  
  
// void pit_callback()  
// {  
//     imu_gyro_z = imu_dev.get_gyro_z();  
  
//     // 零偏标定（上电静止2秒）  
//     if (!bias_ok) {  
//         gyro_z_bias += (float)imu_gyro_z;  
//         bias_cnt++;  
//         if (bias_cnt >= 200) {  
//             gyro_z_bias /= (float)bias_cnt;  
//             bias_ok = true;  
//         }  
//         motor_set_percent(0, 0);  
//         return;  
//     }  
  
//     // 去偏置+低通  
//     float yaw_raw = (float)imu_gyro_z - gyro_z_bias;  
//     yaw_f = 0.9f * yaw_f + 0.1f * yaw_raw;  
  
//     // 位置式PID（先PI）  
//     float err = target_yaw_rate - yaw_f;   // 若发散，改成 yaw_f - target_yaw_rate  
//     integ += err;  
//     integ = clamp_float(integ, -300.0f, 300.0f);  
  
//     float deriv = err - err_last;  
//     err_last = err;  
  
//     float u = kp * err + ki * integ + kd * deriv;  
//     u = clamp_float(u, -3.0f, 3.0f);  // 临时小限幅，防止抵消过头  
  
//     int duty_l = (int)( base_duty + u );  
//     int duty_r = (int)(-base_duty - u );  
  
//     // 保底不熄火  
//     if (duty_l >= 0 && duty_l < dead_duty) duty_l = dead_duty;  
//     if (duty_l <  0 && duty_l > -dead_duty) duty_l = -dead_duty;  
//     if (duty_r >= 0 && duty_r < dead_duty) duty_r = dead_duty;  
//     if (duty_r <  0 && duty_r > -dead_duty) duty_r = -dead_duty;  
  
//     motor_set_percent(duty_l, duty_r);  
// }  
  
// void sigint_handler(int)  
// {  
//     printf("收到Ctrl+C，程序即将退出\n");  
//     exit(0);  
// }  
// void cleanup()  
// {  
//     pit_timer.stop();  
//     drv8701e_pwm_1.set_duty(0);  
//     drv8701e_pwm_2.set_duty(0);  
//     printf("cleanup done\n");  
// }  
  
// int main(int, char**)  
// {  
//     atexit(cleanup);  
//     signal(SIGINT, sigint_handler);  
  
//     drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);  
//     drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);  
//     printf("duty_max1=%d duty_max2=%d\n", MOTOR1_PWM_DUTY_MAX, MOTOR2_PWM_DUTY_MAX);  
  
//     imu_dev.init();  
//     if (imu_dev.imu_type == DEV_IMU660RA)      printf("IMU DEV IS IMU660RA\r\n");  
//     else if (imu_dev.imu_type == DEV_IMU660RB) printf("IMU DEV IS IMU660RB\r\n");  
//     else if (imu_dev.imu_type == DEV_IMU963RA) printf("IMU DEV IS IMU963RA\r\n");  
//     else {  
//         printf("NO FIND IMU DEV\r\n");  
//         return -1;  
//     }  
  
//     pit_timer.init_ms(5, pit_callback);  
  
//     while (1)  
//     {  
//         printf("gyro_z=%d bias=%.2f target=%.1f yaw_f=%.1f\n",  
//                imu_gyro_z, gyro_z_bias, target_yaw_rate, yaw_f);  
//         system_delay_ms(100);  
//     }  
  
//     return 0;  
// }  


// //速度环pid

// #include "zf_common_headfile.hpp"  
// #include "seekfree_assistant_interface.hpp"  
// #include "seekfree_assistant.hpp"  
// #include <csignal>  
// #include <cstdlib>  
// #include <cstdio>  
  
// /* ========================= 电机 ========================= */  
// #define PWM_1_PATH        ZF_PWM_MOTOR_1  
// #define DIR_1_PATH        ZF_GPIO_MOTOR_1  
// #define PWM_2_PATH        ZF_PWM_MOTOR_2  
// #define DIR_2_PATH        ZF_GPIO_MOTOR_2  
  
// zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);  
// zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);  
// zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);  
// zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);  
// struct pwm_info drv8701e_pwm_1_info;  
// struct pwm_info drv8701e_pwm_2_info;  
  
// #define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)  
// #define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)  
// #define MAX_DUTY               (30)  
// #define DEAD_DUTY              (5)  
  
// /* ========================= 编码器 ========================= */  
// #define ENCODER_QUAD_1_PATH    ZF_ENCODER_QUAD_1  
// #define ENCODER_QUAD_2_PATH    ZF_ENCODER_QUAD_2  
// zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);  
// zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);  
  
// #define ENCODER_LINE_NUM       2340.0f  
// #define WHEEL_RADIUS           0.0325f  
// #define CONTROL_DT_S           0.01f  // 10ms  
  
// volatile int16 enc_cnt_l = 0;  
// volatile int16 enc_cnt_r = 0;  
  
// volatile float speed_l = 0.0f, speed_r = 0.0f, speed_avg = 0.0f;  
// volatile float speed_f = 0.0f;  
  
// /* ========================= PIT ========================= */  
// zf_driver_pit pit_timer;  
  
// /* ========================= 速度环参数 ========================= */  
// volatile float target_speed = 3;   // 建议先 0.3~0.8  
// float kp = 9.0f;  
// float ki = 2.0f;  
// float kd = 0.0f;  
  
// float err_last = 0.0f;  
// float integ = 0.0f;  
// volatile int pwm_cmd = 0;  
  
// /* ========================= 逐飞助手(TCP) ========================= */  
// #define SERVER_IP "192.168.196.230"  
// #define PORT      8086  
  
// zf_driver_tcp_client tcp_client_dev;  
// bool tcp_ok = false;  
  
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }  
// uint32 tcp_read_wrap(uint8 *buf, uint32 len)       { return tcp_client_dev.read_data(buf, len); }  
  
// /* 使用库里的全局示波器结构体 */  
// extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;  
  
// /* ========================= 工具函数 ========================= */  
// static inline int clamp_int(int x, int lo, int hi)  
// {  
//     if (x < lo) return lo;  
//     if (x > hi) return hi;  
//     return x;  
// }  
// static inline float clamp_float(float x, float lo, float hi)  
// {  
//     if (x < lo) return lo;  
//     if (x > hi) return hi;  
//     return x;  
// }  
  
// void motor_set_percent_lr(int duty_l, int duty_r)  
// {  
//     duty_l = clamp_int(duty_l, -MAX_DUTY, MAX_DUTY);  
//     duty_r = clamp_int(duty_r, -MAX_DUTY, MAX_DUTY);  
  
//     if (duty_l > 0 && duty_l < DEAD_DUTY) duty_l = DEAD_DUTY;  
//     if (duty_l < 0 && duty_l > -DEAD_DUTY) duty_l = -DEAD_DUTY;  
//     if (duty_r > 0 && duty_r < DEAD_DUTY) duty_r = DEAD_DUTY;  
//     if (duty_r < 0 && duty_r > -DEAD_DUTY) duty_r = -DEAD_DUTY;  
  
//     if (duty_l >= 0) {  
//         drv8701e_dir_2.set_level(1);  
//         drv8701e_pwm_2.set_duty(duty_l * MOTOR2_PWM_DUTY_MAX / 100);  
//     } else {  
//         drv8701e_dir_2.set_level(0);  
//         drv8701e_pwm_2.set_duty((-duty_l) * MOTOR2_PWM_DUTY_MAX / 100);  
//     }  
  
//     if (duty_r >= 0) {  
//         drv8701e_dir_1.set_level(1);  
//         drv8701e_pwm_1.set_duty(duty_r * MOTOR1_PWM_DUTY_MAX / 100);  
//     } else {  
//         drv8701e_dir_1.set_level(0);  
//         drv8701e_pwm_1.set_duty((-duty_r) * MOTOR1_PWM_DUTY_MAX / 100);  
//     }  
// }  
  
// void read_encoder_speed()  
// {  
//     enc_cnt_l = encoder_quad_1.get_count();  
//     enc_cnt_r = -encoder_quad_2.get_count();  
  
//     encoder_quad_1.clear_count();  
//     encoder_quad_2.clear_count();  
  
//     float wl = ((float)enc_cnt_l / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / CONTROL_DT_S;  
//     float wr = ((float)enc_cnt_r / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / CONTROL_DT_S;  
  
//     speed_l = wl * WHEEL_RADIUS;  
//     speed_r = wr * WHEEL_RADIUS;  
//     speed_avg = 0.5f * (speed_l + speed_r);  
  
//     speed_f = 0.85f * speed_f + 0.15f * speed_avg;  
// }  
  
// void pit_callback()  
// {  
//     read_encoder_speed();  
  
//     float err = target_speed - speed_f;  
//     integ += err * CONTROL_DT_S;  
//     integ = clamp_float(integ, -2.0f, 2.0f);  
  
//     float deriv = (err - err_last) / CONTROL_DT_S;  
//     err_last = err;  
  
//     float u = kp * err + ki * integ + kd * deriv;  
//     u = clamp_float(u, -MAX_DUTY, MAX_DUTY);  
  
//     int target_pwm = (int)u;  
//     int step = target_pwm - pwm_cmd;  
//     if (step > 1) step = 1;  
//     if (step < -1) step = -1;  
//     pwm_cmd += step;  
  
//     motor_set_percent_lr(pwm_cmd, pwm_cmd);  
// }  
  
// /* ========================= 退出处理 ========================= */  
// void sigint_handler(int)  
// {  
//     printf("收到Ctrl+C，程序即将退出\n");  
//     exit(0);  
// }  
// void cleanup()  
// {  
//     pit_timer.stop();  
//     drv8701e_pwm_1.set_duty(0);  
//     drv8701e_pwm_2.set_duty(0);  
//     printf("cleanup done\n");  
// }  
  
// /* ========================= main ========================= */  
// int main(int, char**)  
// {  
//     atexit(cleanup);  
//     signal(SIGINT, sigint_handler);  
  
//     drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);  
//     drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);  
//     printf("duty_max1=%d duty_max2=%d\r\n", MOTOR1_PWM_DUTY_MAX, MOTOR2_PWM_DUTY_MAX);  
  
//     tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);  
//     if (tcp_ok)  
//     {  
//         seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);  
//         printf("assistant tcp ok\r\n");  
//     }  
//     else  
//     {  
//         printf("assistant tcp fail\r\n");  
//     }  
  
//     pit_timer.init_ms(10, pit_callback);  
  
//     while (1)  
//     {  
//         printf("tar=%.3f vL=%.3f vR=%.3f vf=%.3f pwm=%d cntL=%d cntR=%d\r\n",  
//                target_speed, speed_l, speed_r, speed_f, pwm_cmd, enc_cnt_l, enc_cnt_r);  
  
//         if (tcp_ok)  
//         {  
//             // 只发 speed_l 和 speed_r 两路波形  
//             seekfree_assistant_oscilloscope_data.channel_num = 2;  
//             seekfree_assistant_oscilloscope_data.data[0] = speed_l;  
//             seekfree_assistant_oscilloscope_data.data[1] = speed_r;  
//             seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);  
  
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)  
//             seekfree_assistant_data_analysis();  
// #endif  
//         }  
  
//         system_delay_ms(20);  
//     }  
  
//     return 0;  
// }  





//逐飞助手调试pid



#include "zf_common_headfile.hpp"  
#include "seekfree_assistant_interface.hpp"  
#include "seekfree_assistant.hpp"  
#include <cstdio>  
#include <csignal>  
#include <cstdlib>  
  
/* ====================== 配置项 ====================== */  
#define SERVER_IP "192.168.196.230"  
#define PORT 8086  
  
/* ====================== 全局设备对象 ====================== */  
zf_driver_tcp_client tcp_client_dev;  
zf_driver_pit pit_timer;  
  
/* ====================== TCP包装 ====================== */  
uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }  
uint32 tcp_read_wrap(uint8 *buf, uint32 len) { return tcp_client_dev.read_data(buf, len); }  
  
extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;  
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)  
extern float seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];  
extern vuint8 seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];  
#endif  
  
/* ====================== 电机/编码器 ====================== */  
#define PWM_1_PATH ZF_PWM_MOTOR_1  
#define DIR_1_PATH ZF_GPIO_MOTOR_1  
#define PWM_2_PATH ZF_PWM_MOTOR_2  
#define DIR_2_PATH ZF_GPIO_MOTOR_2  
  
zf_driver_gpio drv8701e_dir_1(DIR_1_PATH, O_RDWR); // 右  
zf_driver_gpio drv8701e_dir_2(DIR_2_PATH, O_RDWR); // 左  
zf_driver_pwm drv8701e_pwm_1(PWM_1_PATH);  
zf_driver_pwm drv8701e_pwm_2(PWM_2_PATH);  
struct pwm_info drv8701e_pwm_1_info;  
struct pwm_info drv8701e_pwm_2_info;  
  
#define MOTOR1_PWM_DUTY_MAX (drv8701e_pwm_1_info.duty_max)  
#define MOTOR2_PWM_DUTY_MAX (drv8701e_pwm_2_info.duty_max)  
#define MAX_DUTY 30  
#define DEAD_DUTY 5  
  
#define ENCODER_QUAD_1_PATH ZF_ENCODER_QUAD_1  
#define ENCODER_QUAD_2_PATH ZF_ENCODER_QUAD_2  
zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH); // 左  
zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH); // 右  
  
#define ENCODER_LINE_NUM 2340.0f  
#define WHEEL_RADIUS 0.0325f  
#define DT_CTRL 0.01f // 10ms控制  
  
volatile int16 g_enc_l = 0, g_enc_r = 0;  
volatile float g_speed_l = 0.0f, g_speed_r = 0.0f, g_speed = 0.0f;  
  
/* ====================== 左右轮独立速度环 ====================== */  
volatile float g_target_speed = 0.0f;  
  
volatile float spd_kp = 0.0f, spd_ki = 0.0f, spd_kd = 0.0f;  
  
// 左轮PID状态  
float spd_last_l = 0.0f, spd_i_l = 0.0f;  
// 右轮PID状态  
float spd_last_r = 0.0f, spd_i_r = 0.0f;  
  
static inline int clamp_int(int x, int lo, int hi){ return x < lo ? lo : (x > hi ? hi : x); }  
static inline float clamp_f(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }  
  
/* 通道号保持不变 */  
enum  
{  
SA_IDX_TARGET_SPEED = 0, // 通道1  
SA_IDX_DIR_KP, // 通道2（占位）  
SA_IDX_DIR_KD, // 通道3（占位）  
SA_IDX_SPD_KP, // 通道4  
SA_IDX_SPD_KI, // 通道5  
SA_IDX_SPD_KD // 通道6  
};  
  
void motor_set_lr(int duty_l, int duty_r)  
{  
duty_l = clamp_int(duty_l, -MAX_DUTY, MAX_DUTY);  
duty_r = clamp_int(duty_r, -MAX_DUTY, MAX_DUTY);  
  
if (duty_l > 0 && duty_l < DEAD_DUTY) duty_l = DEAD_DUTY;  
if (duty_l < 0 && duty_l > -DEAD_DUTY) duty_l = -DEAD_DUTY;  
if (duty_r > 0 && duty_r < DEAD_DUTY) duty_r = DEAD_DUTY;  
if (duty_r < 0 && duty_r > -DEAD_DUTY) duty_r = -DEAD_DUTY;  
  
if (duty_l >= 0){ drv8701e_dir_2.set_level(1); drv8701e_pwm_2.set_duty(duty_l * MOTOR2_PWM_DUTY_MAX / 100); }      
else            { drv8701e_dir_2.set_level(0); drv8701e_pwm_2.set_duty((-duty_l) * MOTOR2_PWM_DUTY_MAX / 100); }      
  
if (duty_r >= 0){ drv8701e_dir_1.set_level(1); drv8701e_pwm_1.set_duty(duty_r * MOTOR1_PWM_DUTY_MAX / 100); }      
else            { drv8701e_dir_1.set_level(0); drv8701e_pwm_1.set_duty((-duty_r) * MOTOR1_PWM_DUTY_MAX / 100); }  
}  
  
void read_encoder_speed_10ms()  
{  
g_enc_l = -encoder_quad_1.get_count();  
g_enc_r = encoder_quad_2.get_count();  
encoder_quad_1.clear_count();  
encoder_quad_2.clear_count();  
  
float wl = ((float)g_enc_l / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_CTRL;  
float wr = ((float)g_enc_r / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_CTRL;  
g_speed_l = wl * WHEEL_RADIUS;  
g_speed_r = wr * WHEEL_RADIUS;  
g_speed = 0.5f * (g_speed_l + g_speed_r);  
  
}  
  
void control_callback_10ms()  
{  
read_encoder_speed_10ms();  
  
float target_l = g_target_speed;  
float target_r = g_target_speed;  
  
// 左轮PID      
float e_l = target_l - g_speed_l;      
spd_i_l += e_l * DT_CTRL;      
spd_i_l = clamp_f(spd_i_l, -2.0f, 2.0f);      
float d_l = (e_l - spd_last_l) / DT_CTRL;      
spd_last_l = e_l;      
float u_l = spd_kp * e_l + spd_ki * spd_i_l + spd_kd * d_l;      
  
// 右轮PID      
float e_r = target_r - g_speed_r;      
spd_i_r += e_r * DT_CTRL;      
spd_i_r = clamp_f(spd_i_r, -2.0f, 2.0f);      
float d_r = (e_r - spd_last_r) / DT_CTRL;      
spd_last_r = e_r;      
float u_r = spd_kp * e_r + spd_ki * spd_i_r + spd_kd * d_r;      
  
int duty_l = (int)clamp_f(u_l, -MAX_DUTY, MAX_DUTY);      
int duty_r = (int)clamp_f(u_r, -MAX_DUTY, MAX_DUTY);      
  
motor_set_lr(duty_l, duty_r);  
}  
  
static inline void apply_assistant_pid_params()  
{  
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)  
if (seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED]) {  
g_target_speed = clamp_f(seekfree_assistant_parameter[SA_IDX_TARGET_SPEED], 0.0f, 3.0f);  
spd_i_l = 0.0f;  
spd_i_r = 0.0f;  
spd_last_l = 0.0f;  
spd_last_r = 0.0f;  
seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED] = 0;  
}  
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP]) {  
spd_kp = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KP], 0.0f, 40.0f);  
spd_i_l = 0.0f;  
spd_i_r = 0.0f;  
spd_last_l = 0.0f;  
spd_last_r = 0.0f;  
seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP] = 0;  
}  
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI]) {  
spd_ki = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KI], 0.0f, 10.0f);  
spd_i_l = 0.0f;  
spd_i_r = 0.0f;  
spd_last_l = 0.0f;  
spd_last_r = 0.0f;  
seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI] = 0;  
}  
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD]) {  
spd_kd = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KD], 0.0f, 5.0f);  
spd_i_l = 0.0f;  
spd_i_r = 0.0f;  
spd_last_l = 0.0f;  
spd_last_r = 0.0f;  
seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD] = 0;  
}  
#endif  
}  
  
/* ====================== 退出处理 ====================== */  
void sigint_handler(int){ exit(0); }  
void cleanup()  
{  
pit_timer.stop();  
drv8701e_pwm_1.set_duty(0);  
drv8701e_pwm_2.set_duty(0);  
}  
  
int main()  
{  
atexit(cleanup);  
signal(SIGINT, sigint_handler);  
  
bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);  
if (tcp_ok)  
{  
seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);  
  
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)  
seekfree_assistant_parameter[SA_IDX_TARGET_SPEED] = g_target_speed;  
seekfree_assistant_parameter[SA_IDX_DIR_KP] = 0.0f;  
seekfree_assistant_parameter[SA_IDX_DIR_KD] = 0.0f;  
seekfree_assistant_parameter[SA_IDX_SPD_KP] = spd_kp;  
seekfree_assistant_parameter[SA_IDX_SPD_KI] = spd_ki;  
seekfree_assistant_parameter[SA_IDX_SPD_KD] = spd_kd;  
#endif  
}  
  
drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);  
drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);  
  
pit_timer.init_ms(10, control_callback_10ms);  
  
while (1)  
{  
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)  
if (tcp_ok) {  
seekfree_assistant_data_analysis();  
apply_assistant_pid_params();  
}  
#endif  
  
if (tcp_ok)  
{  
seekfree_assistant_oscilloscope_data.channel_num = 8;  
seekfree_assistant_oscilloscope_data.data[0] = 0.0f;  
seekfree_assistant_oscilloscope_data.data[1] = g_speed;  
seekfree_assistant_oscilloscope_data.data[2] = g_target_speed;  
seekfree_assistant_oscilloscope_data.data[3] = g_speed_l;  
seekfree_assistant_oscilloscope_data.data[4] = g_speed_r;  
seekfree_assistant_oscilloscope_data.data[5] = spd_kp;  
seekfree_assistant_oscilloscope_data.data[6] = spd_ki;  
seekfree_assistant_oscilloscope_data.data[7] = spd_kd;  
seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);  
}  
  
    printf("tar=%.2f, v=%.2f, vl=%.2f, vr=%.2f, kp=%.2f, ki=%.2f, kd=%.2f\n",      
           g_target_speed, g_speed, g_speed_l, g_speed_r, spd_kp, spd_ki, spd_kd);      
  
    system_delay_ms(50);      
}  
  
return 0;  
}  











// // ========================== 最终跑车代码（视觉外环10ms + 角速度内环5ms + 速度并环5ms） ==========================  
// #include "zf_common_headfile.hpp"    
// #include "image.hpp"    
// #include "imgproc.hpp"    
// #include "display.hpp"    
// #include "seekfree_assistant_interface.hpp"    
// #include "seekfree_assistant.hpp"    
// #include <opencv2/opencv.hpp>    
// #include <cstdio>    
// #include <cmath>    
// #include <csignal>    
// #include <cstdlib>    
// using namespace cv;    
    
// /* ====================== 配置项 ====================== */    
// #define SERVER_IP "192.168.196.230"    
// #define PORT      8086    
    
// /* ====================== 全局设备对象 ====================== */    
// zf_driver_tcp_client tcp_client_dev;    
// zf_device_uvc        uvc_dev;    
// zf_device_ips200     ips200;    
// zf_driver_pit        pit_timer;    
    
// uint16_t* rgb_ptr = nullptr;    
    
// /* ====================== 图像缓冲 ====================== */    
// uint8 image_gray[UVC_HEIGHT][UVC_WIDTH];    
// uint8 image_bin [UVC_HEIGHT][UVC_WIDTH];    
// uint8 image_ipm [UVC_HEIGHT][UVC_WIDTH];    
    
// // 巡线点（原图）    
// float left_pts[EDGELINE_MAX][2];    
// float right_pts[EDGELINE_MAX][2];    
// float middle_pts[EDGELINE_MAX][2];    
// int left_num  = 0;    
// int right_num = 0;    
    
// // 每行一个点    
// float left_row_pts[EDGELINE_MAX][2], right_row_pts[EDGELINE_MAX][2];    
// int left_row_num = EDGELINE_MAX, right_row_num = EDGELINE_MAX;    
    
// // 用于逐飞助手看边线    
// uint8 left_x[UVC_HEIGHT], right_x[UVC_HEIGHT], middle_x[UVC_HEIGHT];    
// uint8 row_y[UVC_HEIGHT];    
    
// /* ====================== TCP包装 ====================== */    
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }    
// uint32 tcp_read_wrap(uint8 *buf, uint32 len)       { return tcp_client_dev.read_data(buf, len); }    
    
// extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
// extern float  seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
// extern vuint8 seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
// #endif    
    
// /* ====================== 电机/编码器 ====================== */    
// #define PWM_1_PATH ZF_PWM_MOTOR_1    
// #define DIR_1_PATH ZF_GPIO_MOTOR_1    
// #define PWM_2_PATH ZF_PWM_MOTOR_2    
// #define DIR_2_PATH ZF_GPIO_MOTOR_2    
    
// zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR); // 右    
// zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR); // 左    
// zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);    
// zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);    
// struct pwm_info drv8701e_pwm_1_info;    
// struct pwm_info drv8701e_pwm_2_info;    
    
// #define MOTOR1_PWM_DUTY_MAX (drv8701e_pwm_1_info.duty_max)    
// #define MOTOR2_PWM_DUTY_MAX (drv8701e_pwm_2_info.duty_max)    
// #define MAX_DUTY  30    
// #define DEAD_DUTY 5    
    
// #define ENCODER_QUAD_1_PATH ZF_ENCODER_QUAD_1    
// #define ENCODER_QUAD_2_PATH ZF_ENCODER_QUAD_2    
// zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH); // 左    
// zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH); // 右    
    
// #define ENCODER_LINE_NUM 2340.0f    
// #define WHEEL_RADIUS     0.0325f    
// #define DT_SPEED         0.005f   // 速度环 5ms  
// #define DT_DIR           0.01f    // 角度环 10ms  
    
// volatile int16 g_enc_l = 0, g_enc_r = 0;    
// volatile float g_speed_l = 0.0f, g_speed_r = 0.0f, g_speed = 0.0f;    
    
// /* ====================== 双环控制（无陀螺仪） ====================== */    
// volatile float g_err_img = 0.0f;    
// volatile float dir_kp = 0.8f, dir_kd = 0.2f;    
// float dir_last = 0.0f;    
// volatile float g_u_yaw = 0.0f;   // 角度环输出(10ms更新)  
    
// volatile float g_target_speed = 1.0f;    
// volatile float spd_kp = 9.0f, spd_ki = 0.01f, spd_kd = 0.0f;    
// float spd_last = 0.0f, spd_i = 0.0f;    
    
// static inline int clamp_int(int x, int lo, int hi){ return x < lo ? lo : (x > hi ? hi : x); }    
// static inline float clamp_f(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }    
    
// /* 注意：你的库里保存参数时是 seekfree_assistant_parameter[channel-1]    
//  * 所以下标应从0开始定义 */    
// enum    
// {    
//     SA_IDX_TARGET_SPEED = 0,   // 通道1    
//     SA_IDX_DIR_KP,             // 通道2    
//     SA_IDX_DIR_KD,             // 通道3    
//     SA_IDX_SPD_KP,             // 通道4    
//     SA_IDX_SPD_KI,             // 通道5    
//     SA_IDX_SPD_KD              // 通道6    
// };    
    
// void motor_set_lr(int duty_l, int duty_r)    
// {    
//     duty_l = clamp_int(duty_l, -MAX_DUTY, MAX_DUTY);    
//     duty_r = clamp_int(duty_r, -MAX_DUTY, MAX_DUTY);    
    
//     if (duty_l > 0 && duty_l < DEAD_DUTY) duty_l = DEAD_DUTY;    
//     if (duty_l < 0 && duty_l > -DEAD_DUTY) duty_l = -DEAD_DUTY;    
//     if (duty_r > 0 && duty_r < DEAD_DUTY) duty_r = DEAD_DUTY;    
//     if (duty_r < 0 && duty_r > -DEAD_DUTY) duty_r = -DEAD_DUTY;    
    
//     if (duty_l >= 0){ drv8701e_dir_2.set_level(0); drv8701e_pwm_2.set_duty(duty_l * MOTOR2_PWM_DUTY_MAX / 100); }    
//     else            { drv8701e_dir_2.set_level(1); drv8701e_pwm_2.set_duty((-duty_l) * MOTOR2_PWM_DUTY_MAX / 100); }    
    
//     if (duty_r >= 0){ drv8701e_dir_1.set_level(0); drv8701e_pwm_1.set_duty(duty_r * MOTOR1_PWM_DUTY_MAX / 100); }    
//     else            { drv8701e_dir_1.set_level(1); drv8701e_pwm_1.set_duty((-duty_r) * MOTOR1_PWM_DUTY_MAX / 100); }    
// }    
  
// // 速度环：5ms    
// static inline void speed_loop_5ms()    
// {    
//     g_enc_l =  - encoder_quad_1.get_count();    
//     g_enc_r =  encoder_quad_2.get_count();    
//     encoder_quad_1.clear_count();    
//     encoder_quad_2.clear_count();    
    
//     float wl = ((float)g_enc_l / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_SPEED;    
//     float wr = ((float)g_enc_r / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_SPEED;    
//     g_speed_l = wl * WHEEL_RADIUS;    
//     g_speed_r = wr * WHEEL_RADIUS;    
//     g_speed   = 0.5f * (g_speed_l + g_speed_r);    
    
//     float e_spd = g_target_speed - g_speed;    
//     spd_i += e_spd;                              // [仅改动] 去掉 *DT_SPEED  
//     spd_i = clamp_f(spd_i, -2.0f, 2.0f);    
//     float d_spd = (e_spd - spd_last);            // [仅改动] 去掉 /DT_SPEED  
//     spd_last = e_spd;    
    
//     float u_spd = spd_kp * e_spd + spd_ki * spd_i + spd_kd * d_spd;    
//     u_spd = clamp_f(u_spd, -MAX_DUTY, MAX_DUTY);    
    
//     int duty_l = (int)(u_spd + g_u_yaw);    
//     int duty_r = (int)(u_spd - g_u_yaw);    
//     motor_set_lr(duty_l, duty_r);    
// }  

  
// // 角度环：10ms  
// static inline void dir_loop_10ms()  
// {  
//     float e_dir = g_err_img;  
//     g_u_yaw = -(dir_kp * e_dir + dir_kd * (e_dir - dir_last));  
//     dir_last = e_dir;  
//     g_u_yaw = clamp_f(g_u_yaw, -12.0f, 12.0f);  
// }  
  
// // 定时器回调：5ms触发，内部调度速度环/角度环  
// void pit_callback_control(void)  
// {  
//     static uint8 div2 = 0; // 2*5ms=10ms  
  
//     speed_loop_5ms();      // 每5ms执行  
  
//     div2++;  
//     if (div2 >= 2)  
//     {  
//         div2 = 0;  
//         dir_loop_10ms();   // 每10ms执行  
//     }  
// }  
    
// static inline void apply_assistant_pid_params()    
// {    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED]) {    
//         g_target_speed = clamp_f(seekfree_assistant_parameter[SA_IDX_TARGET_SPEED], 0.0f, 3.0f);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP]) {    
//         dir_kp = clamp_f(seekfree_assistant_parameter[SA_IDX_DIR_KP], 0.0f, 5.0f);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD]) {    
//         dir_kd = clamp_f(seekfree_assistant_parameter[SA_IDX_DIR_KD], 0.0f, 2.0f);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP]) {    
//         spd_kp = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KP], 0.0f, 20.0f);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI]) {    
//         spd_ki = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KI], 0.0f, 5.0f);    
//         spd_i = 0.0f;    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD]) {    
//         spd_kd = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KD], 0.0f, 2.0f);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD] = 0;    
//     }    
// #endif    
// }    
    
// /* ====================== 退出处理 ====================== */    
// void sigint_handler(int){ exit(0); }    
// void cleanup()    
// {    
//     pit_timer.stop();    
//     drv8701e_pwm_1.set_duty(0);    
//     drv8701e_pwm_2.set_duty(0);    
// }    
    
// int main()    
// {    
//     atexit(cleanup);    
//     signal(SIGINT, sigint_handler);    
    
//     bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);    
//     if (tcp_ok)    
//     {    
//         seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);    
//         seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_gray[0], UVC_WIDTH, UVC_HEIGHT);    
    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//         // 同步初值到参数数组（便于你自己查看）    
//         seekfree_assistant_parameter[SA_IDX_TARGET_SPEED] = g_target_speed;    
//         seekfree_assistant_parameter[SA_IDX_DIR_KP]       = dir_kp;    
//         seekfree_assistant_parameter[SA_IDX_DIR_KD]       = dir_kd;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KP]       = spd_kp;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KI]       = spd_ki;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KD]       = spd_kd;    
// #endif    
//     }    
    
//     drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);    
//     drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);    
    
//     ips200.init(FB_PATH);    
//     display_init(&ips200);    
    
//     if (uvc_dev.init(UVC_PATH) < 0) return -1;    
    
//     // 改为5ms定时器线程，内部调度5ms速度环 + 10ms角度环  
//     pit_timer.init_ms(5, pit_callback_control);    
    
//     while (1)    
//     {    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//         if (tcp_ok) {    
//             seekfree_assistant_data_analysis();  // 解析助手下发参数    
//             apply_assistant_pid_params();        // 应用新参数    
//         }    
// #endif    
    
//         if (uvc_dev.wait_image_refresh() < 0) break;    
//         rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();    
//         if (rgb_ptr == nullptr) continue;    
    
//         image_process(rgb_ptr, (uint8_t*)image_gray[0], UVC_WIDTH, UVC_HEIGHT);    
    
//         const uint8 th = 128;    
//         for (int y = 0; y < UVC_HEIGHT; y++)    
//             for (int x = 0; x < UVC_WIDTH; x++)    
//                 image_bin[y][x] = (image_gray[y][x] > th) ? 255 : 0;    
    
//         cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]);    
//         add_black_border(bin_mat, 3);    
    
//         int sx_l = UVC_WIDTH / 2, sy_l = UVC_HEIGHT - 5;    
//         int sx_r = UVC_WIDTH / 2, sy_r = UVC_HEIGHT - 5;    
    
//         find_left_base(bin_mat,  &sx_l, &sy_l);    
//         find_right_base(bin_mat, &sx_r, &sy_r);    
    
//         left_num  = EDGELINE_MAX;    
//         right_num = EDGELINE_MAX;    
//         findline_lefthand_adaptive (bin_mat, sx_l, sy_l, left_pts,  &left_num);    
//         findline_righthand_adaptive(bin_mat, sx_r, sy_r, right_pts, &right_num);    
    
//         left_row_num = EDGELINE_MAX;    
//         right_row_num = EDGELINE_MAX;    
//         compress_line_one_point_per_row(left_pts,  left_num,  left_row_pts,  &left_row_num,  true);    
//         compress_line_one_point_per_row(right_pts, right_num, right_row_pts, &right_row_num, false);    
    
//         int mid_num = 0;    
//         build_midline_from_compressed_lr(    
//             left_row_pts, left_row_num,    
//             right_row_pts, right_row_num,    
//             middle_pts, &mid_num    
//         );    
    
//         for (int i = 0; i < UVC_HEIGHT; i++) {    
//             left_x[i] = right_x[i] = middle_x[i] = 0;    
//             row_y[i] = (uint8)i;    
//         }    
    
//         for (int i = 0; i < left_row_num; i++) {    
//             int x = (int)(left_row_pts[i][0] + 0.5f), y = (int)(left_row_pts[i][1] + 0.5f);    
//             if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT) left_x[y] = (uint8)x;    
//         }    
//         for (int i = 0; i < right_row_num; i++) {    
//             int x = (int)(right_row_pts[i][0] + 0.5f), y = (int)(right_row_pts[i][1] + 0.5f);    
//             if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT) right_x[y] = (uint8)x;    
//         }    
//         for (int i = 0; i < mid_num; i++) {    
//             int x = (int)(middle_pts[i][0] + 0.5f), y = (int)(middle_pts[i][1] + 0.5f);    
//             if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT) middle_x[y] = (uint8)x;    
//         }    
    
//         if (mid_num > 10) {    
//             float s = 0.0f;    
//             for (int i = 5; i < 10; i++) s += middle_pts[i][0];    
//             g_err_img = s / 5.0f - UVC_WIDTH * 0.5f;    
//         } else g_err_img = 0.0f;    
    
//         ips200.show_gray_image(0, 0, image_bin[0], UVC_WIDTH, UVC_HEIGHT, UVC_WIDTH, UVC_HEIGHT, 0);    
    
//         if (tcp_ok)    
//         {    
//             seekfree_assistant_camera_boundary_config(    
//                 X_BOUNDARY, UVC_HEIGHT,    
//                 left_x, right_x, middle_x,    
//                 nullptr, nullptr, nullptr    
//             );    
//             seekfree_assistant_camera_send();    
    
//             seekfree_assistant_oscilloscope_data.channel_num = 5;    
//             seekfree_assistant_oscilloscope_data.data[0] = g_err_img;    
//             seekfree_assistant_oscilloscope_data.data[1] = g_speed;    
//             seekfree_assistant_oscilloscope_data.data[2] = g_target_speed;    
//             seekfree_assistant_oscilloscope_data.data[3] = g_speed_l;   // 左轮速度  
//             seekfree_assistant_oscilloscope_data.data[4] = g_speed_r;   // 右轮速度     
//             seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);    
//         }    
    
//         system_delay_ms(20);    
//     }    
    
//     return 0;    
// }  








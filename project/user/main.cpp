#include "zf_common_headfile.hpp"      
#include <opencv2/opencv.hpp>    
#include <cstdio>    
#include <cmath>    
#include <csignal>    
#include <cstdlib>    
using namespace cv;    
    
/* ====================== 配置项 ====================== */    
#define SERVER_IP "192.168.196.230"    
#define PORT      8086    
    
#define KEY_1_PATH ZF_GPIO_KEY_1  
#define KEY_2_PATH ZF_GPIO_KEY_2
zf_driver_gpio key_1(KEY_1_PATH, O_RDWR);   // 或 O_RDONLY，按你驱动支持  
zf_driver_gpio key_2(KEY_2_PATH, O_RDWR);


/* ====================== 全局设备对象 ====================== */    
zf_driver_tcp_client tcp_client_dev;    
zf_device_uvc        uvc_dev;    
zf_device_ips200     ips200;    
/*=========================定时器设置=================================*/  
volatile uint32_t g_speed_loop_cnt = 0;   // 1秒内速度环执行次数    
volatile uint32_t g_last_speed_hz  = 0;   // 上一秒统计值    
  
float yaw_diff = 0.0f;
zf_driver_pit        pit_timer;    
zf_driver_pit        fps_timer;  
zf_driver_pit        img_timer;  
    
uint16_t* rgb_ptr = nullptr;   
// ===== 基础参数 =====    
float aim_dist         = 0.4f;    
float resample_dist    = 1.0f;    
float angle_dist       = 2.0f; 
int aim_id = 30;

   
float pixel_per_meter= 100.0f;    
int   blur_kernel    = 5;    
    
// ===== 跟踪状态 =====    
track_type_e track_type = TRACK_LEFT;    
    
// ===== 角点与直线标志 =====    
bool Lpt_l_found = false;    
bool Lpt_r_found = false;    
int  Lpt_l_id = 0;    
int  Lpt_r_id = 0;    
bool is_straight_l = false;    
bool is_straight_r = false;    
    
// ===== 线数据 =====    
float rpts_l_resample[POINTS_MAX_LEN][2] = {0};    
float rpts_r_resample[POINTS_MAX_LEN][2] = {0};    
int   rpts_l_resample_num = 0;    
int   rpts_r_resample_num = 0;    
    
float rpts_lc[POINTS_MAX_LEN][2] = {0};    
float rpts_rc[POINTS_MAX_LEN][2] = {0};    
int   rpts_lc_num = 0;    
int   rpts_rc_num = 0;    
    
// ===== 十字用 =====    
float begin_x = 0.f;    
float begin_y = 0.f;    
    
// 你 cross.cpp 里是 extern float mapx[120][160], mapy[120][160];    
float mapx[120][160] = {0};    
float mapy[120][160] = {0};     
    
/* ====================== 图像缓冲 ====================== */    
uint8 image_gray[UVC_HEIGHT][UVC_WIDTH];    
uint8 image_bin [UVC_HEIGHT][UVC_WIDTH];    
uint8 image_ipm [UVC_HEIGHT][UVC_WIDTH];    
    
//查看帧率  
int my_fps = 0 ;  
    
/* 用于逐飞助手看边线 */    
uint8 left_x  [UVC_HEIGHT];    
uint8 right_x [UVC_HEIGHT];    
uint8 center_x[UVC_HEIGHT];    
uint8 row_y   [UVC_HEIGHT];    
// 图像与中线相关  
  float ipts_l[EDGELINE_MAX][2] = {0};  
float ipts_r[EDGELINE_MAX][2] = {0};  
int   ipts_l_num = 0;  
int   ipts_r_num = 0;  


  
float rpts_l[POINTS_MAX_LEN][2] = {0};  
float rpts_r[POINTS_MAX_LEN][2] = {0};  
int   rpts_l_num = 0;  
int   rpts_r_num = 0;  
  
float rpts_l_blur[POINTS_MAX_LEN][2] = {0};  
float rpts_r_blur[POINTS_MAX_LEN][2] = {0};  
  
float rpts_c[POINTS_MAX_LEN][2] = {0};  
int   rpts_c_num = 0;  

float rpts_c_same[POINTS_MAX_LEN][2] = {0};  
int rpts_c_same_num = 0;

float rpts_c_resample[POINTS_MAX_LEN][2] = {0};  
int rpts_c_resample_num = 0;
// 偏差相关  

float img_err = 0.0f;  
  
// 控制量（motor.cpp在用）  
volatile float g_target_speed = 1.0f;  
volatile float g_u_yaw        = 0.0f;  

    
/* ====================== TCP包装 ====================== */    
uint32 tcp_send_wrap(const uint8 *buf, uint32 len){ return tcp_client_dev.send_data(buf, len); }    
uint32 tcp_read_wrap(uint8 *buf, uint32 len)      { return tcp_client_dev.read_data(buf, len); }    
    
extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;    
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
extern float  seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
extern vuint8 seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
#endif    
//查看帧率  
void fps_callback()  
{  
    // 打印角度和圆环状态
    if (circle_type >= 0 && circle_type < CIRCLE_NUM) {
        printf("dangle: %.2f | circle: %s\n", 
               yaw_diff, 
               circle_type_name[circle_type]);
    } else {
        printf("dangle: %.2f | circle: UNKNOWN(%d)\n", 
               g_angle_yaw, 
               circle_type);
    }
} 
void period_print_callback()    
{    
    g_last_speed_hz = g_speed_loop_cnt;    
    g_speed_loop_cnt = 0;    
    
    float period_ms = 0.0f;    
    if (g_last_speed_hz > 0) {    
        period_ms = 1000.0f / (float)g_last_speed_hz;    
    }    
    
    printf("[loop] speed_hz=%u, period=%.3f ms\n", g_last_speed_hz, period_ms);    
}    
  
  
/* ====================== 参数通道索引 ====================== */    
enum    
{    
    SA_IDX_TARGET_SPEED = 0,  // 通道1    
    SA_IDX_DIR_KP,            // 通道2    
    SA_IDX_DIR_KD,            // 通道3    
    SA_IDX_SPD_KP,            // 通道4    
    SA_IDX_SPD_KI,            // 通道5    
    SA_IDX_SPD_KD             // 通道6    
};    
    
/* ====================== 助手参数同步 ====================== */    
/*    
 * 从逐飞助手上位机接收新参数，更新到 motor.cpp 中的 PID 结构体    
 * 参数变更时同时重置对应 PID 积分状态，防止积分跳变    
 */    
static inline void apply_assistant_pid_params()    
{    
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
    auto clamp_f = [](float x, float lo, float hi){    
        return x < lo ? lo : (x > hi ? hi : x);    
    };    
    
    /* 目标速度变更时，调用speed_reset()清除增量式PID历史和累计占空比 */    
if (seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED])    
{    
    g_target_speed = clamp_f(    
        seekfree_assistant_parameter[SA_IDX_TARGET_SPEED], 0.0f, 5.0f);    
    speed_reset();     
    seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED] = 0;    
}    
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP])    
{    
    pid_speed_l.P = pid_speed_r.P =    
        clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KP], 0.0f, 1500.0f);    
    speed_reset();       
    seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP] = 0;    
}    
// 同步 速度环 I    
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI])      
{      
    pid_speed_l.I = pid_speed_r.I = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KI], 0.0f, 30.0f);      
    speed_reset();       
    seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI] = 0;      
}    
    
// 同步 速度环 D    
if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD])      
{      
    pid_speed_l.D = pid_speed_r.D = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KD], 0.0f, 5.0f);      
    speed_reset();       
    seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD] = 0;      
}    
/* 角度环变为位置式，重置用 PID_Pos_Reset */    
if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP])    
{    
    pid_dir.P = clamp_f(    
        seekfree_assistant_parameter[SA_IDX_DIR_KP], 0.0f, 5.0f);    
    PID_Pos_Reset(&pid_dir);   // ← 替代原来的 PID_Inc_Reset    
    seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP] = 0;    
}    
if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD])    
{    
    pid_dir.D = clamp_f(    
        seekfree_assistant_parameter[SA_IDX_DIR_KD], 0.0f, 2.0f);    
    PID_Pos_Reset(&pid_dir);   // ← 替代原来的 PID_Inc_Reset    
    seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD] = 0;    
}    
  
#endif    
}    
    
/* ====================== 退出处理 ====================== */    
void sigint_handler(int)    
{    
    printf("收到Ctrl+C，程序即将退出\n");    
    exit(0);    
}    
    
void cleanup()    
{    
    printf("程序退出，执行清理操作\n");    
    pit_timer.stop();   // 先停定时器，再停电机    
    fps_timer.stop();
    img_timer.stop();    
    motor_stop();       // PWM清零    
}    
    
/* ====================== 主函数 ====================== */    
int main()    
{    
    // 放在 while 外（main里静态变量）  
    static uint8_t key1_last = 1;   // 通常上拉按键：松开=1，按下=0  
    static uint8_t key2_last = 1;

    track_type = TRACK_RIGHT ;
    motor_init();   
    imu_init(); 
    
    /* ---------- 注册退出处理 ---------- */    
    atexit(cleanup);    
    signal(SIGINT, sigint_handler);    
    
    /* ---------- TCP连接 ---------- */    
    bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);    
    if (tcp_ok)    
    {    
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);    
        seekfree_assistant_camera_information_config(    
            SEEKFREE_ASSISTANT_MT9V03X, image_bin[0], UVC_WIDTH, UVC_HEIGHT);    
    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//         /* 将当前PID初始值推送到助手界面显示 */    
//         seekfree_assistant_parameter[SA_IDX_TARGET_SPEED] = g_target_speed;    
//         seekfree_assistant_parameter[SA_IDX_DIR_KP]       = pid_dir.P;    
//         seekfree_assistant_parameter[SA_IDX_DIR_KD]       = pid_dir.D;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KP]       = pid_speed_l.P;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KI]       = pid_speed_l.I;    
//         seekfree_assistant_parameter[SA_IDX_SPD_KD]       = pid_speed_l.D;    
// #endif    
     }    
//     else    
//     {    
//         printf("[main] TCP连接失败，助手功能不可用\n");    
//     }    
    
    /* ---------- 屏幕初始化 ---------- */    
    ips200.init(FB_PATH);    
    display_init(&ips200);    
    
    /* ---------- 摄像头初始化 ---------- */    
    if (uvc_dev.init(UVC_PATH) < 0)    
    {    
        printf("[main] 摄像头初始化失败！\n");    
        return -1;    
    }    
    
    /* ---------- 启动速度环定时器（5ms） ---------- */     
    pit_timer.init_ms(5, speed_cascaded_5ms); //串环  
    fps_timer.init_ms(20, fps_callback);    
  
    /*------------角度环（10ms)---------------------*/  
    img_timer.init_ms(10,yaw_callback_speed);  
    /*------------定时器查看帧率---------------------*/  
    // fps_timer.init_ms(1000,fps_callback) ;  
    
    printf("[main] 初始化完成，开始主循环\n");    
    
    /* ====================== 主循环 ====================== */    
    while (1)    
    {    
        uint8_t key1_now = key_1.get_level();  
        uint8_t key2_now = key_2.get_level();
  
        // 检测“按下沿”：1 -> 0  
        if (key1_last == 1 && key1_now == 0)  
        {  
            circle_type = CIRCLE_NONE;      // 你项目里的“无环岛/无十字”状态枚举名  
            // 如果你的枚举名是 cross_none，就改成 cross_none  
            printf("[key] KEY1 pressed, circle reset to CROSS_NONE\n");  
        }  
        
        key1_last = key1_now;  

        if (key2_last == 1 && key2_now == 0)
        {
            cross_type = CROSS_NONE;
        }
        key2_last = key2_now;

        // printf("[imu] yaw_speed=%.3f deg/s\n", g_yaw_speed);  
        /* -------- 1. 接收助手参数并同步到PID结构体 -------- */    
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
        if (tcp_ok)    
        {    
            seekfree_assistant_data_analysis();    
            apply_assistant_pid_params();    
        }    
#endif    
    
        if (uvc_dev.wait_image_refresh() < 0) break;    
        rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();    
        if (rgb_ptr == nullptr) continue;    
          
        /* -------- 3. 直接得到二值图 -------- */    
        image_process(rgb_ptr, (uint8_t*)image_bin[0], UVC_WIDTH, UVC_HEIGHT);   
        memset(left_x, 0, sizeof(left_x));  
        memset(right_x, 0, sizeof(right_x));  
        memset(center_x, 0, sizeof(center_x));  
 
          
        /* -------- 4. 构造二值Mat -------- */    
        cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]); 
        //构造逆透视Mat//
        // cv::Mat ipm_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_ipm[0]);  
        // 逆透视二值图（原二值图 -> IPM二值图）  
        // cv::warpPerspective(  
        //     bin_mat, ipm_mat,  
        //     cv::Mat(3, 3, CV_32F, (void*)H_IPM),  
        //     cv::Size(UVC_WIDTH, UVC_HEIGHT),  
        //     cv::INTER_NEAREST,  
        //     cv::BORDER_CONSTANT,  
        //     cv::Scalar(0));  
        /* -------- 5. 添加黑边 -------- */    
        // add_black_border(bin_mat, 2);   
        // add_black_border_half(bin_mat, 2);
        /* -------- 6. 寻找左右边线起点 -------- */    
        int sx_l = 2, sy_l = UVC_HEIGHT - 5;    
        int sx_r = UVC_WIDTH - 3, sy_r = UVC_HEIGHT - 5;    
        find_left_base (bin_mat, &sx_l, &sy_l);    
        find_right_base(bin_mat, &sx_r, &sy_r);    
    
        /* -------- 7. 提取左右边线 -------- */  
        ipts_l_num = EDGELINE_MAX;  
        ipts_r_num = EDGELINE_MAX;      
        findline_lefthand_adaptive (bin_mat, sx_l, sy_l, ipts_l,  &ipts_l_num);    
        findline_righthand_adaptive(bin_mat, sx_r, sy_r, ipts_r,  &ipts_r_num);  
         // -------- 9.5 边线逆透视（映射到IPM平面）--------  
        //清零防止溢出
        rpts_l_num = 0 ;
        rpts_r_num = 0;
        for (int i = 0; i < ipts_l_num; i++)  
        {  
            float u, v;  
            if (warp_point_ipm(ipts_l[i][0], ipts_l[i][1], u, v) && rpts_l_num < POINTS_MAX_LEN) 
            {   
                rpts_l[rpts_l_num][0] = u;  
                rpts_l[rpts_l_num][1] = v;  
                rpts_l_num++;  
            }  
        }  
  
        for (int i = 0; i < ipts_r_num; i++)  
        {  
            float u, v;  
            if (warp_point_ipm(ipts_r[i][0], ipts_r[i][1], u, v) && rpts_r_num < POINTS_MAX_LEN)
            {  
                rpts_r[rpts_r_num][0] = u;  
                rpts_r[rpts_r_num][1] = v;  
                rpts_r_num++;  
            }  
        }  

        // 9) 逆透视后左右边线等距采样（新增核心）  
        rpts_l_resample_num  = EDGELINE_MAX;  
        rpts_r_resample_num = EDGELINE_MAX;   
  
        if (rpts_l_num > 2) {  
            blur_points(rpts_l, rpts_l_num, rpts_l_blur, 5);  
            resample_points(rpts_l_blur, rpts_l_num, rpts_l_resample, &rpts_l_resample_num, resample_dist);  
        } else {  
            rpts_l_resample_num = 0;  
        }  
  
        if (rpts_r_num > 2) {  
            blur_points(rpts_r, rpts_r_num, rpts_r_blur, 5);  
            resample_points(rpts_r_blur, rpts_r_num, rpts_r_resample, &rpts_r_resample_num, resample_dist);  
        } else {  
            rpts_r_resample_num = 0;  
        }  
        // 根据左右等距采样点数选择跟踪边  
        // 可加一个最小差值，避免来回抖动  
        const int switch_margin = 3;  
        
        if (rpts_l_resample_num > rpts_r_resample_num + switch_margin) {  
            track_type = TRACK_LEFT;  
        }  
        else if (rpts_r_resample_num > rpts_l_resample_num + switch_margin) {  
            track_type = TRACK_RIGHT;  
        }  
        // 否则保持当前 track_type 不变，防抖  

        //角度变化率
        local_angle_points(rpts_l_resample,rpts_l_resample_num,angles_l,5);
        nms_angle(angles_l,rpts_l_resample_num,angles_nms_l,5);
        max_angle(angles_l,rpts_l_resample_num,&angle_l_max,&angle_l_max_id);
        local_angle_points(rpts_r_resample,rpts_r_resample_num,angles_r,5);
        nms_angle(angles_r,rpts_r_resample_num,angles_nms_r,5);
        max_angle(angles_r,rpts_r_resample_num,&angle_r_max,&angle_r_max_id);
        find_corners();
        check_circle();
        run_circle();
        // check_cross();
        // run_cross();
        float follow_offset = HALF_ROAD_WIDTH;  
  
        //环内/环运行/出环阶段，向内靠 5 像素  
        if (circle_type == CIRCLE_LEFT_IN ||  circle_type == CIRCLE_LEFT_OUT )  
        {  
            follow_offset = HALF_ROAD_WIDTH - 12.0f;  
            if (follow_offset < 0.0f) follow_offset = 0.0f;  
        } 
        if (circle_type == CIRCLE_LEFT_RUNNING ) 
        {
            follow_offset = HALF_ROAD_WIDTH + 12.0f;
        }
        if (circle_type == CIRCLE_LEFT_BEGIN)
        {
            follow_offset = HALF_ROAD_WIDTH + 15.0f;
        }

        // if (track_type == TRACK_LEFT && rpts_l_resample_num <= 0) {  
        //     make_left_border_fallback(rpts_l_resample, &rpts_l_resample_num, 80); // 80可调  
        // }  

        
        if (track_type == TRACK_LEFT) {  
            track_leftline(rpts_l_resample, rpts_l_resample_num,  
                        rpts_c, rpts_c_num,  
                        angle_dist / resample_dist,  
                        follow_offset);  
        }  
        else if (track_type == TRACK_RIGHT) {  
            track_rightline(rpts_r_resample, rpts_r_resample_num,  
                            rpts_c, rpts_c_num,  
                            angle_dist / resample_dist,  
                            follow_offset);  
}  

        //中线归一化
        normalize_midline_with_anchor(rpts_c,rpts_c_num,rpts_c_same,&rpts_c_same_num);
        //中线等距采样
        rpts_c_resample_num = POINTS_MAX_LEN;  
        if (rpts_c_same_num > 1)  
            resample_points(rpts_c_same, rpts_c_same_num, rpts_c_resample, &rpts_c_resample_num, resample_dist);  
        else  
            rpts_c_resample_num = 0;  

        //计算图像中线偏差提供差速
        img_err_get();
            // 上半区：原二值图  
        ips200.show_gray_image(  
            0, 0, image_bin[0],  
            UVC_WIDTH, UVC_HEIGHT,  
            UVC_WIDTH, UVC_HEIGHT, 0);  
        char info[64];  
        snprintf(info, sizeof(info), "img_err: %.2f", img_err);  
        ips200.show_string(4, UVC_HEIGHT + 10, info);  
        
        ips200.show_uint(30, UVC_HEIGHT + 30, rpts_l_resample_num, 3);  
        ips200.show_uint(80, UVC_HEIGHT + 30, rpts_r_resample_num, 3);  
        
        ips200.show_string(4,   UVC_HEIGHT + 45, (char*)"S_l:");  
        ips200.show_string(50,  UVC_HEIGHT + 45, (char*)(is_straight_l ? "YES" : "NO "));  
        ips200.show_string(90,  UVC_HEIGHT + 45, (char*)"S_r:");  
        ips200.show_string(136, UVC_HEIGHT + 45, (char*)(is_straight_r ? "YES" : "NO "));  
        
        ips200.show_string(4, UVC_HEIGHT + 60, (char*)"Circle:");  
        ips200.show_string(60, UVC_HEIGHT + 60, (char*)"                ");  
        ips200.show_string(60, UVC_HEIGHT + 60,  
            (char*)((circle_type >= 0 && circle_type < CIRCLE_NUM) ? circle_type_name[circle_type] : "UNKNOWN"));  
        
        ips200.show_string(4,   UVC_HEIGHT + 75, (char*)"aL:");  
        ips200.show_float (30,  UVC_HEIGHT + 75, angle_l_max, 3, 7);  
        ips200.show_string(90,  UVC_HEIGHT + 75, (char*)"idL:");  
        ips200.show_uint  (128, UVC_HEIGHT + 75, (uint32)angle_l_max_id, 3);  
        
        ips200.show_string(4, UVC_HEIGHT + 90, (char*)"r0x:");  
        if (rpts_r_resample_num > 0) ips200.show_float(40, UVC_HEIGHT + 90, rpts_r_resample[0][0], 2, 6);  
        else                         ips200.show_string(40, UVC_HEIGHT + 90, (char*)"--");  
        
        ips200.show_string(90, UVC_HEIGHT + 90, (char*)"r0y:");  
        if (rpts_r_resample_num > 0) ips200.show_float(126, UVC_HEIGHT + 90, rpts_r_resample[0][1], 2, 6);  
        else                         ips200.show_string(126, UVC_HEIGHT + 90, (char*)"--");  
        
        // 新增 turn_id / turn_y  
        extern int g_left_begin_turn_id;   // 若你在circle.cpp里是static，需要去掉static或提供getter  
        ips200.show_string(4, UVC_HEIGHT + 105, (char*)"tid:");  
        if (g_left_begin_turn_id >= 0) ips200.show_uint(40, UVC_HEIGHT + 105, (uint32)g_left_begin_turn_id, 3);  
        else                            ips200.show_string(40, UVC_HEIGHT + 105, (char*)"--");  
        
        ips200.show_string(90, UVC_HEIGHT + 105, (char*)"ty:");  
        if (g_left_begin_turn_id >= 0 && g_left_begin_turn_id < ipts_l_num)  
            ips200.show_float(116, UVC_HEIGHT + 105, ipts_l[g_left_begin_turn_id][1], 2, 6);  
        else  
            ips200.show_string(116, UVC_HEIGHT + 105, (char*)"--");  
        
        ips200.show_string(4, UVC_HEIGHT + 120, (char*)"Cross:");  
        ips200.show_string(60, UVC_HEIGHT + 120, (char*)"                ");  
        ips200.show_string(60, UVC_HEIGHT + 120,  
            (char*)((cross_type >= 0 && cross_type < CROSS_NUM) ? cross_type_name[cross_type] : "UNKNOWN"));
        ips200.show_string(4, UVC_HEIGHT + 150, (char*)"yaw:");  
        ips200.show_float (40, UVC_HEIGHT + 150, g_angle_yaw, 2, 7);  
        ips200.show_string(90, UVC_HEIGHT + 165, (char*)"beg:");  
        ips200.show_float (126,UVC_HEIGHT + 165, angle_begin, 2, 7);  
        yaw_diff = g_angle_yaw - angle_begin;  
        ips200.show_string(4, UVC_HEIGHT + 135, (char*)"dyaw:");  
        ips200.show_float (50, UVC_HEIGHT + 135, yaw_diff, 2, 7);  

  

        







        
        // // 下半区：IPM二值图（y偏移120）  
        // ips200.show_gray_image(  
        //     0, UVC_HEIGHT, image_ipm[0],  
        //     UVC_WIDTH, UVC_HEIGHT,  
        //     UVC_WIDTH, UVC_HEIGHT, 0);  
        /*=====================================画线================================================*/
        // for (int y = 0; y < UVC_HEIGHT; y++)  
        // {  
        //     if (left_x[y]  > 0 && left_x[y]  < UVC_WIDTH) ips200.draw_point(left_x[y],  y, RGB565_GREEN);  
        //     if (right_x[y] > 0 && right_x[y] < UVC_WIDTH) ips200.draw_point(right_x[y], y, RGB565_RED);  
        //     if (center_x[y]> 0 && center_x[y]< UVC_WIDTH) ips200.draw_point(center_x[y],y, RGB565_YELLOW);  
        // }  
        for (int i = 0; i < rpts_l_num; i++) {  
            int x = (int)(rpts_l[i][0] + 0.5f);  
            int y = (int)(rpts_l[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_GREEN);  
        }  
        for (int i = 0; i < rpts_r_num; i++) {  
            int x = (int)(rpts_r[i][0] + 0.5f);  
            int y = (int)(rpts_r[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_RED);  
        }  
        for (int i = 0; i < rpts_c_num; i++) {  
            int x = (int)(rpts_c[i][0] + 0.5f);  
            int y = (int)(rpts_c[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_YELLOW);  
        }
        if (Lpt_l_found && Lpt_l_id >= 0 && Lpt_l_id < rpts_l_resample_num) {  
            int x = (int)(rpts_l_resample[Lpt_l_id][0] + 0.5f);  
            int y = (int)(rpts_l_resample[Lpt_l_id][1] + 0.5f);  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT) {  
                ips200.draw_point(x, y, RGB565_WHITE); // 左角点白色  
            }  
        }  
    
        if (Lpt_r_found && Lpt_r_id >= 0 && Lpt_r_id < rpts_r_resample_num) {  
            int x = (int)(rpts_r_resample[Lpt_r_id][0] + 0.5f);  
            int y = (int)(rpts_r_resample[Lpt_r_id][1] + 0.5f);  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT) {  
                ips200.draw_point(x, y, RGB565_BLUE);  // 右角点蓝色  
            }  
        }  

    
        // my_fps++;  
        // /* -------- 13. 屏幕显示二值图 -------- */    
        // ips200.show_gray_image(    
        //     0, 0, image_bin[0],    
        //     UVC_WIDTH, UVC_HEIGHT,    
        //     UVC_WIDTH, UVC_HEIGHT, 0); 
           
    
        /* -------- 14. 逐飞助手发送数据 -------- */    
        if (tcp_ok)    
        {    
            /* 发送摄像头边线（左/右/中） */    
            // seekfree_assistant_camera_boundary_config(    
            //     X_BOUNDARY, UVC_HEIGHT,    
            //     left_x, right_x, center_x,    
            //     nullptr, nullptr, nullptr);    
            seekfree_assistant_camera_send();    
    
            /* 发送示波器：偏差/均速/目标速/左轮速/右轮速 */    
            seekfree_assistant_oscilloscope_data.channel_num = 5;    
            seekfree_assistant_oscilloscope_data.data[0] = img_err;    
            seekfree_assistant_oscilloscope_data.data[1] = g_speed;    
            seekfree_assistant_oscilloscope_data.data[2] = g_target_speed;    
            seekfree_assistant_oscilloscope_data.data[3] = g_speed_l;    
            seekfree_assistant_oscilloscope_data.data[4] = g_speed_r;    
            seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);    
        }    
        
      
    }    


    
    return 0;    
}  



  
// #include "zf_common_headfile.hpp"    
// #include "display.hpp"    
// #include "imgproc.hpp"  
// #include <opencv2/opencv.hpp>    
// #include <cstdio>    
// #include <cmath>    
// #include <csignal>    
// #include <cstdlib>    
// #include <cstdint>  
// #include <cstring>  
// #include <cassert>  
    
// using namespace cv;    
    
// /* ====================== 配置项 ====================== */    
// #define SERVER_IP "192.168.196.230"    
// #define PORT      8086    
    
// /* ====================== 全局设备对象 ====================== */    
// zf_driver_tcp_client tcp_client_dev;    
// zf_device_uvc        uvc_dev;    
// zf_device_ips200     ips200;    
    
// /*=========================定时器设置=================================*/    
// volatile uint32_t g_speed_loop_cnt = 0;   // 1秒内速度环执行次数    
// volatile uint32_t g_last_speed_hz  = 0;   // 上一秒统计值    
    
// zf_driver_pit pit_timer;    
// zf_driver_pit fps_timer;    
// zf_driver_pit img_timer;    
    
// uint16_t* rgb_ptr = nullptr;    
    
// // ===== 基础参数 =====    
// float aim_distance    = 0.4f;    
// float sample_dist     = 0.02f;    
// float angle_dist      = 0.1f;    
// float pixel_per_meter = 100.0f;    
// int   blur_kernel     = 5;    
    
// // ===== 跟踪状态 =====    
// track_type_e track_type = TRACK_LEFT;    
    
// // ===== 角点与直线标志 =====    
// bool Lpt_l_found   = false;    
// bool Lpt_r_found   = false;    
// int  Lpt_l_id      = 0;    
// int  Lpt_r_id      = 0;    
// bool is_straight_l = false;    
// bool is_straight_r = false;    
    
// // ===== 线数据 =====    
// float rpts_l_resample[POINTS_MAX_LEN][2] = {0};    
// float rpts_r_resample[POINTS_MAX_LEN][2] = {0};    
// int   rpts_l_resample_num = 0;    
// int   rpts_r_resample_num = 0;    
    
// float rpts_lc[POINTS_MAX_LEN][2] = {0};    
// float rpts_rc[POINTS_MAX_LEN][2] = {0};    
// int   rpts_lc_num = 0;    
// int   rpts_rc_num = 0;    
    
// // ===== 十字用 =====    
// float begin_x = 0.f;    
// float begin_y = 0.f;    
    
// // 你 cross.cpp 里是 extern float mapx[120][160], mapy[120][160];    
// float mapx[120][160] = {0};    
// float mapy[120][160] = {0};    
    
// /* ====================== 图像缓冲 ====================== */    
// uint8 image_gray[UVC_HEIGHT][UVC_WIDTH];    
// uint8 image_bin [UVC_HEIGHT][UVC_WIDTH];    
// uint8 image_ipm [UVC_HEIGHT][UVC_WIDTH];    
    
// /* 巡线点（原图） */    
// float left_pts  [EDGELINE_MAX][2];    
// float ipts_r [EDGELINE_MAX][2];    
// float middle_pts[EDGELINE_MAX][2];    
// int   left_num  = 0;    
// int   right_num = 0;    
    
// /* 每行一个点 */    
// float rpts_l [EDGELINE_MAX][2];    
// float right_row_pts[EDGELINE_MAX][2];    
// int   ipts_l_num  = EDGELINE_MAX;    
// int   right_row_num = EDGELINE_MAX;    
  
// /* IPM点集 */  
// float rpts_l [EDGELINE_MAX][2];  
// float rpts_r[EDGELINE_MAX][2];  
// int   rpts_l_num  = 0;  
// int   rpts_r_num = 0;  
  
// /* IPM等距采样点集 */  
// float rpts_l_resample [EDGELINE_MAX][2] = {0};  
// float rpts_r_resample[EDGELINE_MAX][2] = {0};  
// int   rpts_l_resample_num  = 0;  
// int   rpts_r_resample_num = 0;  
// float ipm_sample_dist = 0.5f;  
    
// // 查看帧率    
// int my_fps = 0;    
    
// /* 用于逐飞助手看边线 */    
// uint8 left_x  [UVC_HEIGHT];    
// uint8 right_x [UVC_HEIGHT];    
// uint8 center_x[UVC_HEIGHT];    
// uint8 row_y   [UVC_HEIGHT];    
    
// /* ====================== TCP包装 ====================== */    
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len){ return tcp_client_dev.send_data(buf, len); }    
// uint32 tcp_read_wrap(uint8 *buf, uint32 len)      { return tcp_client_dev.read_data(buf, len); }    
    
// extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
// extern float  seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
// extern vuint8 seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    
// #endif    
  
    
// void fps_callback()    
// {    
//     printf("FPS: %d\n", my_fps);    
//     my_fps = 0;    
// }    
    
// void period_print_callback()    
// {    
//     g_last_speed_hz = g_speed_loop_cnt;    
//     g_speed_loop_cnt = 0;    
    
//     float period_ms = 0.0f;    
//     if (g_last_speed_hz > 0) {    
//         period_ms = 1000.0f / (float)g_last_speed_hz;    
//     }    
    
//     printf("[loop] speed_hz=%u, period=%.3f ms\n", g_last_speed_hz, period_ms);    
// }    
    
// /* ====================== 参数通道索引 ====================== */    
// enum    
// {    
//     SA_IDX_TARGET_SPEED = 0,    
//     SA_IDX_DIR_KP,    
//     SA_IDX_DIR_KD,    
//     SA_IDX_SPD_KP,    
//     SA_IDX_SPD_KI,    
//     SA_IDX_SPD_KD    
// };    
    
// /* ====================== 助手参数同步 ====================== */    
// static inline void apply_assistant_pid_params()    
// {    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//     auto clamp_f = [](float x, float lo, float hi){    
//         return x < lo ? lo : (x > hi ? hi : x);    
//     };    
    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED])    
//     {    
//         g_target_speed = clamp_f(seekfree_assistant_parameter[SA_IDX_TARGET_SPEED], 0.0f, 5.0f);    
//         speed_reset();    
//         seekfree_assistant_parameter_update_flag[SA_IDX_TARGET_SPEED] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP])    
//     {    
//         pid_speed_l.P = pid_speed_r.P = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KP], 0.0f, 20.0f);    
//         speed_reset();    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KP] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI])    
//     {    
//         pid_speed_l.I = pid_speed_r.I = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KI], 0.0f, 10.0f);    
//         speed_reset();    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KI] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD])    
//     {    
//         pid_speed_l.D = pid_speed_r.D = clamp_f(seekfree_assistant_parameter[SA_IDX_SPD_KD], 0.0f, 5.0f);    
//         speed_reset();    
//         seekfree_assistant_parameter_update_flag[SA_IDX_SPD_KD] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP])    
//     {    
//         pid_dir.P = clamp_f(seekfree_assistant_parameter[SA_IDX_DIR_KP], 0.0f, 5.0f);    
//         PID_Pos_Reset(&pid_dir);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KP] = 0;    
//     }    
//     if (seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD])    
//     {    
//         pid_dir.D = clamp_f(seekfree_assistant_parameter[SA_IDX_DIR_KD], 0.0f, 2.0f);    
//         PID_Pos_Reset(&pid_dir);    
//         seekfree_assistant_parameter_update_flag[SA_IDX_DIR_KD] = 0;    
//     }    
// #endif    
// }    
    
// /* ====================== 退出处理 ====================== */    
// void sigint_handler(int)    
// {    
//     printf("收到Ctrl+C，程序即将退出\n");    
//     exit(0);    
// }    
    
// void cleanup()    
// {    
//     printf("程序退出，执行清理操作\n");    
//     pit_timer.stop();    
//     fps_timer.stop();    
//     img_timer.stop();    
//     motor_stop();    
// }    
    
// /* ====================== 主函数 ====================== */    
// int main()    
// {    
//     motor_init();    
    
//     atexit(cleanup);    
//     signal(SIGINT, sigint_handler);    
    
//     // ---------- TCP连接 ----------    
//     bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);    
//     if (tcp_ok)    
//     {    
//         seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);    
//         seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_bin[0], UVC_WIDTH, UVC_HEIGHT);    
//     }    
    
//     // ---------- 屏幕初始化 ----------    
//     ips200.init(FB_PATH);    
//     display_init(&ips200);    
    
//     // ---------- 摄像头初始化 ----------    
//     if (uvc_dev.init(UVC_PATH) < 0)    
//     {    
//         printf("[main] 摄像头初始化失败！\n");    
//         return -1;    
//     }    
    
//     // ---------- 启动定时器 ----------    
//     pit_timer.init_ms(5, speed_parallel_5ms);    
//     img_timer.init_ms(10, yaw_callback_speed);    
//     // fps_timer.init_ms(1000, fps_callback);    
    
//     printf("[main] 初始化完成，开始主循环\n");    
    
//     while (1)    
//     {    
// #if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
//         if (tcp_ok)    
//         {    
//             seekfree_assistant_data_analysis();    
//             apply_assistant_pid_params();    
//         }    
// #endif    
    
//         if (uvc_dev.wait_image_refresh() < 0) break;    
//         rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();    
//         if (rgb_ptr == nullptr) continue;    
    
//         // 1) 二值化    
//         image_process(rgb_ptr, (uint8_t*)image_bin[0], UVC_WIDTH, UVC_HEIGHT);    
    
//         // 2) Mat封装    
//         cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]);    
//         cv::Mat ipm_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_ipm[0]);    
    
//         // 3) IPM逆透视    
//         cv::warpPerspective(  
//             bin_mat, ipm_mat,  
//             cv::Mat(3, 3, CV_32F, (void*)H_IPM),  
//             cv::Size(UVC_WIDTH, UVC_HEIGHT),  
//             cv::INTER_NEAREST,  
//             cv::BORDER_CONSTANT,  
//             cv::Scalar(0));    
    
//         // 4) 黑边 + 起点    
//         add_black_border(bin_mat, 2);    
    
//         int sx_l = UVC_WIDTH / 2, sy_l = UVC_HEIGHT - 5;    
//         int sx_r = UVC_WIDTH / 2, sy_r = UVC_HEIGHT - 5;    
//         find_left_base(bin_mat, &sx_l, &sy_l);    
//         find_right_base(bin_mat, &sx_r, &sy_r);    
    
//         // 5) 提线    
//         left_num  = EDGELINE_MAX;    
//         right_num = EDGELINE_MAX;    
//         findline_lefthand_adaptive (bin_mat, sx_l, sy_l, left_pts,  &left_num);    
//         findline_righthand_adaptive(bin_mat, sx_r, sy_r, ipts_r, &right_num);    
    
//         // // 6) 每行压缩    
//         // ipts_l_num  = EDGELINE_MAX;    
//         // right_row_num = EDGELINE_MAX;    
//         // compress_line_one_point_per_row(left_pts,  left_num,  rpts_l,  &ipts_l_num,  true);    
//         // compress_line_one_point_per_row(ipts_r, right_num, right_row_pts, &right_row_num, false);    
    
//         // 7) 中线（原图）    
//         int mid_num = 0;    
//         build_midline_from_compressed_lr(rpts_l, ipts_l_num, right_row_pts, right_row_num, middle_pts, &mid_num);    
    
//         // 8) 原图点映射到IPM平面    
//         rpts_l_num = 0;  
//         rpts_r_num = 0;  
        
//         for (int i = 0; i < left_num && rpts_l_num < EDGELINE_MAX; i++)  
//         {  
//             float u, v;  
//             if (warp_point_ipm(left_pts[i][0], left_pts[i][1], u, v))  
//             {  
//                 rpts_l[rpts_l_num][0] = u;  
//                 rpts_l[rpts_l_num][1] = v;  
//                 rpts_l_num++;  
//             }  
//         }  
//         for (int i = 0; i < right_num && rpts_r_num < EDGELINE_MAX; i++)  
//         {  
//             float u, v;  
//             if (warp_point_ipm(ipts_r[i][0], ipts_r[i][1], u, v))  
//             {  
//                 rpts_r[rpts_r_num][0] = u;  
//                 rpts_r[rpts_r_num][1] = v;  
//                 rpts_r_num++;  
//             }  
//         }  

  
//         // 9) 逆透视后左右边线等距采样（新增核心）  
//         rpts_l_resample_num  = EDGELINE_MAX;  
//         rpts_r_resample_num = EDGELINE_MAX;  
  
//         float rpts_l_blur [EDGELINE_MAX][2] = {0};  
//         float rpts_r_blur[EDGELINE_MAX][2] = {0};  
  
//         if (rpts_l_num > 2) {  
//             blur_points(rpts_l, rpts_l_num, rpts_l_blur, 5);  
//             resample_points(rpts_l_blur, rpts_l_num, rpts_l_resample, &rpts_l_resample_num, ipm_sample_dist);  
//         } else {  
//             rpts_l_resample_num = 0;  
//         }  
  
//         if (rpts_r_num > 2) {  
//             blur_points(rpts_r, rpts_r_num, rpts_r_blur, 5);  
//             resample_points(rpts_r_blur, rpts_r_num, rpts_r_resample, &rpts_r_resample_num, ipm_sample_dist);  
//         } else {  
//             rpts_r_resample_num = 0;  
//         }  
    
//         // 10) 屏幕显示：上半原二值，下半IPM二值    
//         ips200.show_gray_image(0, 0, image_bin[0], UVC_WIDTH, UVC_HEIGHT, UVC_WIDTH, UVC_HEIGHT, 0);    
//         ips200.show_gray_image(0, UVC_HEIGHT, image_ipm[0], UVC_WIDTH, UVC_HEIGHT, UVC_WIDTH, UVC_HEIGHT, 0);    
    
//         // 11) 叠加边线：  
//         // 上半原图左右边线  
//         draw_points(left_pts,  left_num,  0, RGB565_GREEN);  
//         draw_points(ipts_r, right_num, 0, RGB565_RED);  

//         // 下半IPM显示“等距采样后”左右边线（新增）  
//         draw_points(rpts_l_resample,  rpts_l_resample_num,  UVC_HEIGHT, RGB565_GREEN);    
//         draw_points(rpts_r_resample, rpts_r_resample_num, UVC_HEIGHT, RGB565_RED);    
    
//         // 12) 误差计算 + 文字/十字    
//         float err_img = 0.0f;    
//         if (mid_num > 10)    
//         {    
//             float s = 0.0f;    
//             for (int i = 5; i < 10; i++) s += middle_pts[i][0];    
//             err_img = s / 5.0f - (float)UVC_WIDTH * 0.5f;    
//         }    
    
//         display_show_overlay(left_num, right_num, sx_l, sy_l, sx_r, sy_r, err_img);  
 
    
//         // 13) 控制更新    
//         update_direction();    
//         my_fps++;    
    
//         // 14) 助手发送    
//         if (tcp_ok)    
//         {    
//             seekfree_assistant_camera_send();    
    
//             seekfree_assistant_oscilloscope_data.channel_num = 5;    
//             seekfree_assistant_oscilloscope_data.data[0] = g_err_img;    
//             seekfree_assistant_oscilloscope_data.data[1] = g_speed;    
//             seekfree_assistant_oscilloscope_data.data[2] = g_target_speed;    
//             seekfree_assistant_oscilloscope_data.data[3] = g_speed_l;    
//             seekfree_assistant_oscilloscope_data.data[4] = g_speed_r;    
//             seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);    
//         }    
//     }    
    
//     return 0;    
// }  

// #include "zf_common_headfile.hpp"  
// #include <opencv2/opencv.hpp>  
// #include <cstdio>  
// #include <csignal>  
// #include <cstdlib>  
  
// using namespace cv;  
  
// /* ====================== 配置项 ====================== */  
// #define SERVER_IP "192.168.196.230"  
// #define PORT      8086  
  
// /* ====================== 全局设备对象 ====================== */  
// zf_driver_tcp_client tcp_client_dev;  
// zf_device_uvc        uvc_dev;  
// // ===== 给 cross/circle/motor 的 extern 变量补定义 =====  
// volatile uint32_t g_speed_loop_cnt = 0;  
// volatile uint32_t g_last_speed_hz  = 0;  
  
// float aim_distance    = 0.4f;  
// float sample_dist     = 0.02f;  
// float angle_dist      = 0.1f;  
// float pixel_per_meter = 100.0f;  
// int   blur_kernel     = 5;  
  
// track_type_e track_type = TRACK_LEFT;  

// float resample_dist = 0.0f;  
// float aim_dist      = 0.0f;  
  
// int   rpts_c_num = 0;  
// float img_err    = 0.0f;  
// int   aim_id     = 0;  
  
// float rpts_c[POINTS_MAX_LEN][2] = {0};  
  
// volatile float g_target_speed = 0.0f;  
// volatile float g_u_yaw        = 0.0f;  


  
// bool Lpt_l_found = false;  
// bool Lpt_r_found = false;  
// int  Lpt_l_id    = 0;  
// int  Lpt_r_id    = 0;  
// bool is_straight_l = false;  
// bool is_straight_r = false;  

// float ipts_l[POINTS_MAX_LEN][2] = {0};
  
// float rpts_l_resample[POINTS_MAX_LEN][2] = {0};  
// float rpts_r_resample[POINTS_MAX_LEN][2] = {0};  
// int   rpts_l_resample_num = 0;  
// int   rpts_r_resample_num = 0;  
  
// float rpts_lc[POINTS_MAX_LEN][2] = {0};  
// float rpts_rc[POINTS_MAX_LEN][2] = {0};  
// int   rpts_lc_num = 0;  
// int   rpts_rc_num = 0;  
  
// float begin_x = 0.f;  
// float begin_y = 0.f;  
  
// float mapx[120][160] = {0};  
// float mapy[120][160] = {0};  

  
// /* ====================== 图像缓冲 ====================== */  
// uint8 image_bin[UVC_HEIGHT][UVC_WIDTH];  
// uint8 image_ipm[UVC_HEIGHT][UVC_WIDTH];  
  
// /* ====================== TCP包装 ====================== */  
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }  
// uint32 tcp_read_wrap(uint8 *buf, uint32 len)       { return tcp_client_dev.read_data(buf, len); }  
  
// /* ====================== 退出处理 ====================== */  
// void sigint_handler(int)  
// {  
//     printf("收到Ctrl+C，程序退出\n");  
//     exit(0);  
// }  
  
// int main()  
// {  
//     signal(SIGINT, sigint_handler);  
  
//     /* ---------- TCP连接 + 助手初始化 ---------- */  
//     bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);  
//     if (tcp_ok)  
//     {  
//         seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);  
  
//         // 把“发送图像源”指向 IPM 图像缓冲  
//         seekfree_assistant_camera_information_config(  
//             SEEKFREE_ASSISTANT_MT9V03X,  
//             image_ipm[0],  
//             UVC_WIDTH,  
//             UVC_HEIGHT  
//         );  
//         printf("[main] TCP连接成功，助手已初始化\n");  
//     }  
//     else  
//     {  
//         printf("[main] TCP连接失败，无法发送到助手\n");  
//     }  
  
//     /* ---------- 摄像头初始化 ---------- */  
//     if (uvc_dev.init(UVC_PATH) < 0)  
//     {  
//         printf("[main] 摄像头初始化失败！\n");  
//         return -1;  
//     }  
  
//     printf("[main] 开始主循环\n");  
  
//     while (1)  
//     {  
//         if (uvc_dev.wait_image_refresh() < 0) break;  
  
//         uint16_t* rgb_ptr = (uint16_t*)uvc_dev.get_rgb_image_ptr();  
//         if (rgb_ptr == nullptr) continue;  
  
//         // 1) 二值化  
//         image_process(rgb_ptr, (uint8_t*)image_bin[0], UVC_WIDTH, UVC_HEIGHT);  
  
//         // 2) 逆透视  
//         cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]);  
//         cv::Mat ipm_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_ipm[0]);  
  
//         cv::warpPerspective(  
//             bin_mat, ipm_mat,  
//             cv::Mat(3, 3, CV_32F, (void*)H_IPM),   // 你的逆透视矩阵  
//             cv::Size(UVC_WIDTH, UVC_HEIGHT),  
//             cv::INTER_NEAREST,  
//             cv::BORDER_CONSTANT,  
//             cv::Scalar(0)  
//         );  
  
//         // 3) 发送到逐飞助手（发送的是 image_ipm）  
//         if (tcp_ok)  
//         {  
//             seekfree_assistant_data_analysis(); // 可留可不留  
//             seekfree_assistant_camera_send();  
//         }  
//     }  
  
//     return 0;  
// }  























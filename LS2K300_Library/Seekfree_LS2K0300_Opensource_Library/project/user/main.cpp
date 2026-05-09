
// #include "zf_common_headfile.hpp"
// #include <iostream>
// #include <opencv2/opencv.hpp>



// // ====================== 网络与摄像头配置宏定义 ======================
// #define SERVER_IP "192.168.137.1"   // TCP服务端IP地址（你的电脑IP）
// #define PORT 8086                  // TCP通信端口号

// // 定义摄像头分辨率 (替代原来的 UVC_WIDTH 和 UVC_HEIGHT)
// #define CAM_WIDTH  320
// #define CAM_HEIGHT 240

// // ====================== 全局设备对象定义 ======================
// zf_driver_tcp_client tcp_client_dev;   // 定义TCP客户端设备对象
// // 注：删除了原有的 zf_device_uvc uvc_dev，改用局部实例化的龙邱驱动

// // ====================== 函数声明与封装 ======================
// uint32 tcp_send_wrap(const uint8 *buf, uint32 len) {
//     return tcp_client_dev.send_data(buf, len);
// }

// uint32 tcp_read_wrap(uint8 *buf, uint32 len) {
//     return tcp_client_dev.read_data(buf, len);
// }

// // ====================== 边界信息配置宏定义与全局数组 ======================
// #define INCLUDE_BOUNDARY_TYPE   0

// #define BOUNDARY_NUM            (CAM_HEIGHT * 4 / 2)

// // 全局边界坐标数组定义
// uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];  
// uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];  
// uint8 x1_boundary[CAM_HEIGHT], x2_boundary[CAM_HEIGHT], x3_boundary[CAM_HEIGHT];                
// uint8 y1_boundary[CAM_WIDTH], y2_boundary[CAM_WIDTH], y3_boundary[CAM_WIDTH];                    

// // 图像数据拷贝缓冲区
// // 此处统一采用一维数组分配内存，方便后续不同类型的数据强转
// uint8_t image_copy[CAM_WIDTH * CAM_HEIGHT * 2]; 

// // **************************** 主函数入口 ****************************
// int main() 
// {
//     // ====================== 1.初始化TCP客户端 ======================
//     if(tcp_client_dev.init(SERVER_IP, PORT) == 0) {
//         printf("tcp_client ok\r\n");
//     } else {
//         printf("tcp_client error\r\n"); 
//         return -1;                      
//     }

//     // ====================== 2.初始化逐飞助手通信接口 ======================
//     seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

//     // ====================== 3.初始化龙邱摄像头 ======================
//     // 实例化：宽, 高, 帧率, 格式, 节点路径
//     lq_camera_ex camera(CAM_WIDTH, CAM_HEIGHT, 30, LQ_CAMERA_HIGH_MJPG, "/dev/video0");
//     if (!camera.is_cam_opened()) {
//         printf("摄像头打开失败！\r\n");
//         return -1;
//     }
//     // camera.set_exposure_manual(60); // 如需锁定曝光可取消注释

//     // ====================== 4.根据边界类型配置逐飞助手 ======================
// #if(0 == INCLUDE_BOUNDARY_TYPE)
//     // 类型0：发送彩色图像。SCC8660对应的是16位RGB565格式
//     seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, image_copy, CAM_WIDTH, CAM_HEIGHT);

// #elif(1 == INCLUDE_BOUNDARY_TYPE)
//     for(int i = 0; i < CAM_HEIGHT; i++) {
//         x1_boundary[i] = 50 - (50 - 20) * i / CAM_HEIGHT;
//         x2_boundary[i] = CAM_WIDTH / 2;
//         x3_boundary[i] = 70 + (148 - 70) * i / CAM_HEIGHT;
//     }
//     // 类型1-4通常使用灰度图。MT9V03X对应的是8位单通道灰度格式
//     seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy, CAM_WIDTH, CAM_HEIGHT);
//     seekfree_assistant_camera_boundary_config(X_BOUNDARY, CAM_HEIGHT, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);

//     // ...(为了代码简洁，省略2, 3, 4的边界赋值，与你原代码逻辑一致，仅替换UVC_HEIGHT/WIDTH即可)
// #endif

//     // OpenCV 图像接收容器
//     cv::Mat raw_img;
//     cv::Mat gray_img;

//     // ====================== 主循环：持续采集图像+发送数据 ======================
//     while (true) 
//     {
//         // 调用龙邱库，同时获取彩色原始帧和灰度帧
//         if(camera.get_frame_raw_gray(raw_img, gray_img) && !raw_img.empty()) 
//         {
//             // ------------------------------------------------------------------
//             // 【图像数据转换核心逻辑】
//             cv::flip(raw_img, raw_img, -1);   // 将彩图翻转180度，并覆写回原变量
//             cv::flip(gray_img, gray_img, -1); // 将灰度图翻转180度，并覆写回原变量
//             // ------------------------------------------------------------------
            
// #if(0 == INCLUDE_BOUNDARY_TYPE)
//             // 模式0：发送彩色RGB图
//             // 1. 利用OpenCV将默认的BGR格式转为RGB565格式
//             cv::Mat rgb565_img;
//             cv::cvtColor(raw_img, rgb565_img, cv::COLOR_BGR2BGR565);
            
//             // 2. 逐飞助手要求的RGB565高低位是相反的，需要手动交换字节
//             uint16_t* src_ptr = (uint16_t*)rgb565_img.data;
//             uint16_t* dst_ptr = (uint16_t*)image_copy;
//             for(uint32_t i = 0; i < CAM_WIDTH * CAM_HEIGHT; i++) {
//                 dst_ptr[i] = (src_ptr[i] >> 8) | (src_ptr[i] << 8);
//             }
// #else
//             // 模式1/2/3/4：发送灰度图
//             // 灰度图直接就是8位单通道数据，无需高低位交换，直接内存拷贝即可，效率极高！
//             // 这里你也可以把自己处理后的二值化图像传给助手，方便查看寻线算法的效果
//             memcpy(image_copy, gray_img.data, CAM_WIDTH * CAM_HEIGHT);
// #endif

//             // 调用核心发送函数，推送到电脑
//             seekfree_assistant_camera_send();
//         }
//     }

//     return 0; 
// }

#include "zf_common_headfile.hpp"      
#include "lq_camera_ex.hpp"        // [修改点1] 引入龙邱摄像头头文件
#include <opencv2/opencv.hpp>    
#include <cstdio>    
#include <cmath>    
#include <csignal>    
#include <cstdlib>    
using namespace cv;    
   
/* ====================== 配置项 ====================== */    
#define SERVER_IP "192.168.196.230"    
#define PORT      8086   

ncnn::Net my_net;
   
#define KEY_1_PATH ZF_GPIO_KEY_1  
#define KEY_2_PATH ZF_GPIO_KEY_2
zf_driver_gpio key_1(KEY_1_PATH, O_RDWR);   // 或 O_RDONLY，按你驱动支持  
zf_driver_gpio key_2(KEY_2_PATH, O_RDWR);

extern Mat bin_mat; // [新增] 声明全局二值化图像变量，供十字状态机使用


/* ====================== 全局设备对象 ====================== */    
zf_driver_tcp_client tcp_client_dev;    
// zf_device_uvc        uvc_dev;   // [修改点2] 删除逐飞UVC对象 
zf_device_ips200     ips200;    

/*=========================定时器设置=================================*/  
volatile uint32_t g_speed_loop_cnt = 0;   // 1秒内速度环执行次数    
volatile uint32_t g_last_speed_hz  = 0;   // 上一秒统计值    
  
float yaw_diff = 0.0f;
zf_driver_pit        pit_timer;    
zf_driver_pit        fps_timer;  
zf_driver_pit        img_timer;  
   
uint16_t* rgb_ptr = nullptr;   
uint16_t* gray_ptr = nullptr;

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
float begin_x = 10.0f;  
float begin_y = 80.0f;   

// ====================== 视觉绕行全局控制 ======================
bool  g_is_bypassing_binoculars = false; // 是否正在执行望远镜绕行
int   g_bypass_timer = 0;                // 绕行持续帧数计时器
const int BYPASS_MAX_FRAMES = 80;        // 绕行持续时间（假设30帧/秒，80帧大约2.5秒，根据你的车速调）
const float BYPASS_OFFSET = 35.0f;       // 向左绕行的偏移量（像素），越大绕得越宽
// =============================================================

RedBlockAvoider g_brick_avoider; // [新增] 全局避障器对象
   
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

// 辅助函数：防止索引越界
inline int limit_index(int val, int min_val, int max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// 辅助函数：防止画图坐标超出屏幕导致段错误崩溃
inline uint16 clamp_coord(float coord, uint16 max_limit) {
    if (coord < 0) return 0;
    if (coord >= max_limit) return max_limit - 1;
    return (uint16)coord;
}

// -------------------------------------------------------------------------
// 新增函数：绘制最大角度点的前后计算向量
// -------------------------------------------------------------------------
void draw_max_angle_vectors(zf_device_ips200 &ips, float pts_in[][2], int num, int max_id, int dist, uint16 color) 
{
    // 点数太少或索引不合法时跳过
    if (num < 2 || max_id < 0 || max_id >= num) return;

    // 获取前后距离 dist 的参考点（带边界保护）
    int idx_prev = limit_index(max_id - dist, 0, num - 1);
    int idx_next = limit_index(max_id + dist, 0, num - 1);

    // 转换坐标并防止越界，这里假设你的图像有效区域是 UVC_WIDTH x UVC_HEIGHT
    uint16 px = clamp_coord(pts_in[idx_prev][0], UVC_WIDTH);
    uint16 py = clamp_coord(pts_in[idx_prev][1], UVC_HEIGHT);
    
    uint16 cx = clamp_coord(pts_in[max_id][0], UVC_WIDTH);
    uint16 cy = clamp_coord(pts_in[max_id][1], UVC_HEIGHT);
    
    uint16 nx = clamp_coord(pts_in[idx_next][0], UVC_WIDTH);
    uint16 ny = clamp_coord(pts_in[idx_next][1], UVC_HEIGHT);

    // 用特定颜色画出组成角度的两根向量
    ips.draw_line(px, py, cx, cy, color);
    ips.draw_line(cx, cy, nx, ny, color);

    // 在最大角度点(拐点)中心画一个 3x3 的加粗色块，使其更醒目
    for(int i = -1; i <= 1; i++) {
        for(int j = -1; j <= 1; j++) {
            int draw_x = cx + i;
            int draw_y = cy + j;
            if (draw_x >= 0 && draw_x < UVC_WIDTH && draw_y >= 0 && draw_y < UVC_HEIGHT) {
                ips.draw_point(draw_x, draw_y, color);
            }
        }
    }
}
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

/* ====================== 小车状态机枚举 ====================== */
enum CarState {
    STATE_INIT,         // 初始化/停车准备状态（等待按键发车）
    STATE_TRACKING,     // 正常循迹状态（直道、普通弯道）
    STATE_CIRCLE,       // 环岛处理状态
    STATE_CROSS,        // 十字处理状态
    STATE_STOP          // 紧急停车/冲线停车保护状态
};

// 当前小车状态，开机默认为等待发车
CarState g_car_state = STATE_INIT; 
  
  
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
static inline void apply_assistant_pid_params()    
{    
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
    auto clamp_f = [](float x, float lo, float hi){    
        return x < lo ? lo : (x > hi ? hi : x);    
    };    
   
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
    printf("[main] 正在加载 NCNN 模型...\n");
    my_net.opt.num_threads = 2; 
    my_net.opt.use_fp16_arithmetic = false;
    my_net.opt.use_fp16_storage = false;
    my_net.load_param("tiny_classifier_fp32.ncnn.param");
    my_net.load_model("tiny_classifier_fp32.ncnn.bin");
    printf("[main] NCNN 模型加载完毕！\n");
   
    /* ---------- 注册退出处理 ---------- */    
    atexit(cleanup);    
    signal(SIGINT, sigint_handler);    
   
    /* ---------- TCP连接 ---------- */    
    // bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);    
    bool tcp_ok = false;
    if (tcp_ok)    
    {    
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);    
        seekfree_assistant_camera_information_config(    
            SEEKFREE_ASSISTANT_MT9V03X, image_bin[0], UVC_WIDTH, UVC_HEIGHT);    
     }    
   
    /* ---------- 屏幕初始化 ---------- */    
    ips200.init(FB_PATH);    
    display_init(&ips200);    
   
    /* ---------- 摄像头初始化 ---------- */    
    // [修改点3] 初始化龙邱摄像头类，宽高参数使用UVC默认宏
    lq_camera_ex camera(320, 240, 60, LQ_CAMERA_0CPU_MJPG, "/dev/video0");
    if (!camera.is_cam_opened())    
    {    
        printf("[main] 龙邱摄像头初始化失败！\n");    
        return -1;    
    }    
   
    /* ---------- 启动速度环定时器（5ms） ---------- */     
    pit_timer.init_ms(5, speed_cascaded_5ms); //串环  
    // fps_timer.init_ms(50, fps_callback);    
  
    /*------------角度环（10ms)---------------------*/  
    img_timer.init_ms(10,yaw_callback_speed);  
   
    printf("[main] 初始化完成，开始主循环\n");    
   
    /* ====================== 主循环 ====================== */    
    cv::Mat raw_img, gray_img; // [修改点4] 声明OpenCV Mat对象用于接收图像
    cv::Mat rgb565_img;

    while (1)    
    {    
        uint8_t key1_now = key_1.get_level();  
        uint8_t key2_now = key_2.get_level();
  
        // 检测“按下沿”：1 -> 0  
        if (key1_last == 1 && key1_now == 0)  
        {  
            circle_type = CIRCLE_NONE;      // 你项目里的“无环岛/无十字”状态枚举名  
            printf("[key] KEY1 pressed, circle reset to CROSS_NONE\n");  
        }  
        
        key1_last = key1_now;  

        if (key2_last == 1 && key2_now == 0)
        {
            cross_type = CROSS_NONE;
        }
        key2_last = key2_now;

        /* -------- 1. 接收助手参数并同步到PID结构体 -------- */    
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)    
        if (tcp_ok)    
        {    
            seekfree_assistant_data_analysis();    
            apply_assistant_pid_params();    
        }    
#endif    
   
        // [修改点5] 替换逐飞UVC获取图像的逻辑，使用龙邱驱动获取并翻转
        if (!camera.get_frame_raw_gray(raw_img, gray_img) || raw_img.empty()) {
            continue;    
        }
        cv::flip(raw_img, raw_img, -1);
        cv::flip(gray_img, gray_img, -1);
        process_car_vision(raw_img);//模型
        //压缩图像，只处理灰度
        cv::resize(raw_img, raw_img, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);
        cv::resize(gray_img, gray_img, cv::Size(160, 120), 0, 0,cv::INTER_NEAREST);


        // 传入翻转后的彩色图，识别红砖并更新内部状态机
        g_brick_avoider.process(raw_img, false); 
        
        // 获取避障指令
        // float avoid_offset = g_brick_avoider.get_avoid_offset();
        float avoid_offset = 0.0f;
        // TrackForceType force_dir = g_brick_avoider.get_force_track_type();
        TrackForceType force_dir = FORCE_NONE;




        // --- 转换为逐飞原有的RGB565格式 ---
        cv::cvtColor(raw_img, rgb565_img, cv::COLOR_BGR2BGR565);
        rgb_ptr = (uint16_t*)rgb565_img.data; 

        /* -------- 3. 直接得到二值图 -------- */    
        image_process(rgb_ptr, (uint8_t*)image_bin[0], UVC_WIDTH, UVC_HEIGHT);   
        memset(left_x, 0, sizeof(left_x));  
        memset(right_x, 0, sizeof(right_x));  
        memset(center_x, 0, sizeof(center_x));  
 
          
        /* -------- 4. 构造二值Mat -------- */    
        cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, image_bin[0]); 

        /* -------- 6. 寻找左右边线起点 -------- */    
        int sx_l = 80, sy_l = UVC_HEIGHT - 5;    
        int sx_r = 80, sy_r = UVC_HEIGHT - 5;    
        find_left_base (bin_mat,  &sx_l, &sy_l);    
        find_right_base(bin_mat, &sx_r, &sy_r);    
   
        /* -------- 7. 提取左右边线 -------- */  
        ipts_l_num = EDGELINE_MAX;  
        ipts_r_num = EDGELINE_MAX;      
        findline_lefthand_adaptive (bin_mat, sx_l, sy_l, ipts_l,  &ipts_l_num);    
        findline_righthand_adaptive(bin_mat, sx_r, sy_r, ipts_r,  &ipts_r_num);  
        // ====================================================================
        // ✨ 新增：高度截断逻辑 (裁掉高于图像顶端 10 行的边线点)
        // ====================================================================
        int cut_y_threshold = 5; // 设定阈值，10代表屏蔽顶端 0~9 行区域

        // 截断左线
        int valid_l_num = 0;
        for (int i = 0; i < ipts_l_num; i++) {
            // Y坐标大于阈值，说明点在画面下方（有效区）
            if (ipts_l[i][1] >= cut_y_threshold) { 
                ipts_l[valid_l_num][0] = ipts_l[i][0];
                ipts_l[valid_l_num][1] = ipts_l[i][1];
                valid_l_num++;
            } else {
                // 因为巡线是从下往上找的，一旦碰到高于阈值的点，直接打断，后面的点全扔掉
                break; 
            }
        }
        ipts_l_num = valid_l_num; // 更新真实有效的点数

        // 截断右线
        int valid_r_num = 0;
        for (int i = 0; i < ipts_r_num; i++) {
            if (ipts_r[i][1] >= cut_y_threshold) {
                ipts_r[valid_r_num][0] = ipts_r[i][0];
                ipts_r[valid_r_num][1] = ipts_r[i][1];
                valid_r_num++;
            } else {
                break; 
            }
        }
        ipts_r_num = valid_r_num; // 更新真实有效的点数
        // -------- 9.5 边线逆透视（查表极速版） --------  
        rpts_l_num = 0;
        rpts_r_num = 0;

        // 左边线查表映射
        for (int i = 0; i < ipts_l_num; i++) 
        { 
            // 1. 获取原始图像坐标并取整（确保坐标在 160x120 范围内）
            int px = (int)(ipts_l[i][0] + 0.5f); 
            int py = (int)(ipts_l[i][1] + 0.5f);

            // 2. 边界检查，防止索引越界导致程序崩溃
            if (px >= 0 && px < 160 && py >= 0 && py < 120) 
            {
                // 3. 核心步骤：使用 g_ipm_valid 过滤掉无效点（如地平线以上的噪点）
                if (g_ipm_valid[py][px] && rpts_l_num < POINTS_MAX_LEN) 
                {   
                    // 4. 直接映射到物理坐标
                    rpts_l[rpts_l_num][0] = g_ipm_lut_u[py][px];  
                    rpts_l[rpts_l_num][1] = g_ipm_lut_v[py][px];  
                    rpts_l_num++;  
                } 
            }
        }

        // 右边线处理逻辑同上
        for (int i = 0; i < ipts_r_num; i++) 
        { 
            int px = (int)(ipts_r[i][0] + 0.5f);
            int py = (int)(ipts_r[i][1] + 0.5f);

            if (px >= 0 && px < 160 && py >= 0 && py < 120) 
            {
                if (g_ipm_valid[py][px] && rpts_r_num < POINTS_MAX_LEN) 
                {  
                    rpts_r[rpts_r_num][0] = g_ipm_lut_u[py][px];  
                    rpts_r[rpts_r_num][1] = g_ipm_lut_v[py][px];  
                    rpts_r_num++;  
                } 
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

        // find_corners();
        check_circle();
        run_circle();
        check_cross();
        run_cross(bin_mat);
        float follow_offset = HALF_ROAD_WIDTH;  
  
        // //环内/环运行/出环阶段，向内靠 5 像素  
        // if (circle_type == CIRCLE_LEFT_IN ||  circle_type == CIRCLE_LEFT_OUT )  
        // {  
        //     follow_offset = HALF_ROAD_WIDTH - 3.0f;  
        //     if (follow_offset < 0.0f) follow_offset = 0.0f;  
        // } 
        // if (circle_type == CIRCLE_LEFT_RUNNING ) 
        // {
        //     follow_offset = HALF_ROAD_WIDTH + 3.0f;
        // }
        // if (circle_type == CIRCLE_LEFT_BEGIN)
        // {
        //     follow_offset = HALF_ROAD_WIDTH + 3.0f;
        // }
       // 在 main() 函数的 while(1) 中修改这部分：

       // ====================== 边线分配与绕行控制 ======================
        

        // 🥇 最高优先级：特定视觉目标绕行 (望远镜左绕行)
        if (g_is_bypassing_binoculars) {
            
            track_type = TRACK_LEFT; // 左侧绕行，必须死死咬住左边线！
            
            // 偏移量减小，意味着车体中线向左侧边线靠拢，实现左绕
            follow_offset = HALF_ROAD_WIDTH - BYPASS_OFFSET; 
            if (follow_offset < 0.0f) follow_offset = 0.0f; // 极限保护
            
            g_target_speed = 1.0f; // 绕行时强制降速求稳，避免甩尾出界
            
            // 计时器自增
            g_bypass_timer++;
            if (g_bypass_timer >= BYPASS_MAX_FRAMES) {
                // 绕行时间结束，恢复正常状态
                g_is_bypassing_binoculars = false;
                std::cout << "✅ 望远镜绕行结束，恢复正常巡线！" << std::endl;
                // 可选：beep_off();
            }
        }
        // 🥈 第二优先级：红砖避障 (原有逻辑)
        else if (force_dir == FORCE_LEFT_LINE) {
            track_type = TRACK_LEFT;
            follow_offset = HALF_ROAD_WIDTH - avoid_offset;
            if (follow_offset < 0.0f) follow_offset = 0.0f;
            g_target_speed = 1.0f; 
        }
        else if (force_dir == FORCE_RIGHT_LINE) {
            track_type = TRACK_RIGHT;
            follow_offset = HALF_ROAD_WIDTH - avoid_offset; // 这里注意，如果是向右靠，可能是 + avoid_offset，取决于你的底层推算
            if (follow_offset < 0.0f) follow_offset = 0.0f;
            g_target_speed = 1.0f;
        }
        // 🥉 第三优先级：正常巡线 (无特殊目标、无红砖、无元素)
        else if (force_dir == FORCE_NONE && circle_type == CIRCLE_NONE) { 
            const int switch_margin = 3;  
            if (rpts_l_resample_num > rpts_r_resample_num + switch_margin) {  
                track_type = TRACK_LEFT;  
            } else if (rpts_r_resample_num > rpts_l_resample_num + switch_margin) {  
                track_type = TRACK_RIGHT;  
            }  
            // 恢复正常巡线速度（如果你的系统会自动从助手同步速度，这里可以不写，或者写成默认速度）
            // g_target_speed = 2.0f; 
        }

        
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

        // 动态调整预瞄点
        aim_id = 30; // 直道默认看 30

        if (circle_type == CIRCLE_LEFT_IN || circle_type == CIRCLE_RIGHT_IN) 
        {
            // 尝试增大 aim_id，利用远处弯道的曲率产生大偏差
            aim_id = 80;  // 可以尝试 45, 50, 甚至 60
        }
        if (cross_type == CROSS_IN) 
        {
            aim_id = 40;  // 十字看远一点，避免干扰
        }

        //计算图像中线偏差提供差速
        img_err_get();
        if(cross_type == CROSS_IN || cross_type == CROSS_IN)
        {
            limit_float(img_err, -5.0f, 5.0f); // 十字时限制偏差，避免过度纠正
        
        }
            // 上半区：原二值图  
        ips200.show_gray_image(  
            0, 0, image_bin[0],  
            UVC_WIDTH, UVC_HEIGHT,  
            UVC_WIDTH, UVC_HEIGHT, 0);  
        char info[64];  
        snprintf(info, sizeof(info), "img_err: %.2f", img_err);  
        ips200.show_string(4, UVC_HEIGHT + 10, info);  
        
        // ips200.show_uint(30, UVC_HEIGHT + 30, rpts_l_resample_num, 3);  
        // ips200.show_uint(80, UVC_HEIGHT + 30, rpts_r_resample_num, 3);  
         
        ips200.show_string(4, UVC_HEIGHT + 30, (char*)"block_w:");
        ips200.show_uint(80, UVC_HEIGHT + 30, block_w, 5);
        ips200.show_string(4, UVC_HEIGHT + 45, (char*)"block_h:");  
        ips200.show_float(80, UVC_HEIGHT + 45, block_h, 2, 6);  

        // ips200.show_string(4, UVC_HEIGHT + 60, (char*)"Circle:");  
        // ips200.show_string(60, UVC_HEIGHT + 60, (char*)"                ");  
        // ips200.show_string(60, UVC_HEIGHT + 60,  
        //     (char*)((circle_type >= 0 && circle_type < CIRCLE_NUM) ? circle_type_name[circle_type] : "UNKNOWN"));  
        
        ips200.show_string(4,   UVC_HEIGHT + 75, (char*)"aL:");  
        ips200.show_float (30,  UVC_HEIGHT + 75, angle_l_max, 3, 3);  
        ips200.show_string(90,  UVC_HEIGHT + 75, (char*)"idL:");  
        ips200.show_uint  (128, UVC_HEIGHT + 75, (uint32)angle_l_max_id, 3); 

        ips200.show_string(4,   UVC_HEIGHT + 60, (char*)"aR:");  
        ips200.show_float (30,  UVC_HEIGHT + 60, angle_r_max, 3, 3);  
        ips200.show_string(90,  UVC_HEIGHT  + 60, (char*)"idR:");  
        ips200.show_uint  (128, UVC_HEIGHT + 60, (uint32)angle_r_max_id, 3);

        
        ips200.show_string(4, UVC_HEIGHT + 90, (char*)"r0x:");  
        if (rpts_r_resample_num > 0) ips200.show_float(40, UVC_HEIGHT + 90, rpts_r_resample[0][0], 2, 6);  
        else                         ips200.show_string(40, UVC_HEIGHT + 90, (char*)"--");  
        
        ips200.show_string(90, UVC_HEIGHT + 90, (char*)"r0y:");  
        if (rpts_r_resample_num > 0) ips200.show_float(126, UVC_HEIGHT + 90, rpts_r_resample[0][1], 2, 6);  
        else                         ips200.show_string(126, UVC_HEIGHT + 90, (char*)"--");  
        
        // 新增 turn_id / turn_y  
        // ========== 屏幕状态显示对称补全 ==========
        extern int g_left_begin_turn_id;   
        extern int g_right_begin_turn_id; // [新增] 声明右环的拐点ID

        // 判断当前是否应该显示右环相关状态
        // 如果左侧未命中且右侧有拐点，或者你当前循迹模式判定为在跑右环，则切换为显示右环信息
        bool show_right_status = false; 
        if (g_left_begin_turn_id < 0 && g_right_begin_turn_id >= 0) {
            show_right_status = true;
        }

        // 1. 最大角度与ID显示补全 (左环看aL, 右环看aR)
        if (!show_right_status) {
            ips200.show_string(4,   UVC_HEIGHT + 75, (char*)"aL:");  
            ips200.show_float (30,  UVC_HEIGHT + 75, angle_l_max, 3, 7);  
            ips200.show_string(90,  UVC_HEIGHT + 75, (char*)"idL:");  
            ips200.show_uint  (128, UVC_HEIGHT + 75, (uint32)angle_l_max_id, 3);  
        } else {
            ips200.show_string(4,   UVC_HEIGHT + 75, (char*)"aR:");  
            ips200.show_float (30,  UVC_HEIGHT + 75, angle_r_max, 3, 7);  
            ips200.show_string(90,  UVC_HEIGHT + 75, (char*)"idR:");  
            ips200.show_uint  (128, UVC_HEIGHT + 75, (uint32)angle_r_max_id, 3);  
        }
        
        // 2. 边线第0点坐标显示补全 (进左环通常看外侧右线r0x，进右环对应看外侧左线l0x)
        if (!show_right_status) {
            ips200.show_string(4, UVC_HEIGHT + 90, (char*)"r0x:");  
            if (rpts_r_resample_num > 0) ips200.show_float(40, UVC_HEIGHT + 90, rpts_r_resample[0][0], 2, 6);  
            else                         ips200.show_string(40, UVC_HEIGHT + 90, (char*)"--");  
            
            ips200.show_string(90, UVC_HEIGHT + 90, (char*)"r0y:");  
            if (rpts_r_resample_num > 0) ips200.show_float(126, UVC_HEIGHT + 90, rpts_r_resample[0][1], 2, 6);  
            else                         ips200.show_string(126, UVC_HEIGHT + 90, (char*)"--");  
        } else {
            ips200.show_string(4, UVC_HEIGHT + 90, (char*)"l0x:");  
            if (rpts_l_resample_num > 0) ips200.show_float(40, UVC_HEIGHT + 90, rpts_l_resample[0][0], 2, 6);  
            else                         ips200.show_string(40, UVC_HEIGHT + 90, (char*)"--");  
            
            ips200.show_string(90, UVC_HEIGHT + 90, (char*)"l0y:");  
            if (rpts_l_resample_num > 0) ips200.show_float(126, UVC_HEIGHT + 90, rpts_l_resample[0][1], 2, 6);  
            else                         ips200.show_string(126, UVC_HEIGHT + 90, (char*)"--");  
        }
        
        // 3. 转折点 turn_id / turn_y 显示补全 (Lid vs Rid)
        if (!show_right_status) {
            ips200.show_string(4, UVC_HEIGHT + 105, (char*)"Lid:");  
            if (g_left_begin_turn_id >= 0) ips200.show_uint(40, UVC_HEIGHT + 105, (uint32)g_left_begin_turn_id, 3);  
            else                           ips200.show_string(40, UVC_HEIGHT + 105, (char*)"--");  
            
            ips200.show_string(90, UVC_HEIGHT + 105, (char*)"ty:");  
            if (g_left_begin_turn_id >= 0 && g_left_begin_turn_id < ipts_l_num)  
                ips200.show_float(116, UVC_HEIGHT + 105, ipts_l[g_left_begin_turn_id][1], 2, 6);  
            else  
                ips200.show_string(116, UVC_HEIGHT + 105, (char*)"--"); 
        } else {
            ips200.show_string(4, UVC_HEIGHT + 105, (char*)"Rid:");  
            if (g_right_begin_turn_id >= 0) ips200.show_uint(40, UVC_HEIGHT + 105, (uint32)g_right_begin_turn_id, 3);  
            else                            ips200.show_string(40, UVC_HEIGHT + 105, (char*)"--");  
            
            ips200.show_string(90, UVC_HEIGHT + 105, (char*)"ty:");  
            if (g_right_begin_turn_id >= 0 && g_right_begin_turn_id < ipts_r_num)  
                ips200.show_float(116, UVC_HEIGHT + 105, ipts_r[g_right_begin_turn_id][1], 2, 6);  
            else  
                ips200.show_string(116, UVC_HEIGHT + 105, (char*)"--"); 
        }
        // ==========================================================
        
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

        /*=====================================画线================================================*/
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
        /*===============================画远线=============================*/
        for (int i = 0; i < far_rpts_l_num; i++) {  
            int x = (int)(far_rpts_l[i][0] + 0.5f);  
            int y = (int)(far_rpts_l[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_GREEN);  
        }  
        for (int i = 0; i < far_rpts_r_num; i++) {  
            int x = (int)(far_rpts_r[i][0] + 0.5f);  
            int y = (int)(far_rpts_r[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_RED);  
        }  
        for (int i = 0; i < far_rpts_c_num; i++) {  
            int x = (int)(far_rpts_c[i][0] + 0.5f);  
            int y = (int)(far_rpts_c[i][1] + 0.5f) ;  
            if (x >= 0 && x < UVC_WIDTH && y >= 0 && y <  UVC_HEIGHT) ips200.draw_point(x, y, RGB565_YELLOW);  
        }


        /* -------- 14. 逐飞助手发送数据 -------- */    
        if (tcp_ok)    
        {    
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
// #include <iostream>
// #include <opencv2/opencv.hpp>
// int main()
// {
//     my_ncnn_env_test();      // 这是刚才跑通的自检
//     my_ncnn_photo_demo();    // 加上这行！真正的图片分类推理！
//     return 0;
// }
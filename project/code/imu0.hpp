#ifndef IMU0_HPP  
#define IMU0_HPP  
  
#include "zf_common_headfile.hpp"  
  
/* ============ 配置 ============ */  
#define IMU_ZERO_COUNT  200    // 零漂采样次数（200×5ms = 1秒）  
#define IMU_DT_MS       10      // 读取周期  
#define IMU_LPF_ALPHA   0.1f   // 低通滤波系数（参考代码用的0.1）  
  
/* ============ 全局变量 ============ */  
extern volatile float g_yaw_speed;     // 校准+滤波后的 yaw 角速度（raw单位）  
extern volatile float g_angle_yaw;     // 积分航向角（raw×ms 单位，环岛用）  
extern volatile bool  g_imu_ready;     // 零漂校准完成标志  

extern volatile uint32_t g_imu_call_cnt;      // 当前1秒内调用次数  
extern volatile uint32_t g_imu_call_hz;
  
/* ============ 接口 ============ */  
void  imu_init(void);  
void  imu_read(void);         // 5ms 定时器回调  
void  imu_reset_angle(void);  
float imu_get_yaw_speed(void);  
float imu_get_angle(void);  
  
#endif  

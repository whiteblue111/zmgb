
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




















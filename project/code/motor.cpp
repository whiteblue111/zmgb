// #include "zf_common_headfile.hpp"
// #include "motor.hpp"
// /* ---------------------------------- imu相关 --------------------------------- */
// int16 imu_acc_x,imu_acc_y,imu_acc_z;
// int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
// zf_device_imu imu_dev;
// float angle_yaw = 0;       // 角度
// float zero_point = 0;      // imu零点
// int8 zero_point_count = 0; // imu零点计数
// float yaw_speed = 0;       // yaw角速度 °/s
// #define IMU_ZERO_COUNT 50  // imu零点计数
// /* ---------------------------------- 编码器相关 --------------------------------- */
// #define ENCODER_LINE_NUM 2340 //  确定编码器线数
// #define WHEEL_RADIUS 0.0325   // 确定 轮子半径
// int16 encoder_count_l = 0;    // 左电机编码器计数
// int16 encoder_count_r = 0;    // 右电机编码器计数
// float encoder_distance = 0.0; // 编码器记录行驶距离
// /* ---------------------------------- PID相关 --------------------------------- */
// #define dead_least_r 0  //待确认！！！
// #define dead_least_l 0  //待确认！！！
// int16 pwm_r = dead_least_r; // 右电机PWM值
// int16 pwm_l = dead_least_l; // 左电机PWM值
// bool stop = 0;
// /*------------------------------------电机相关-----------------------------------*/
// #define MAX_DUTY        (30 )   // 最大 MAX_DUTY% 占空比
// // 在设备树中，设置的10000。如果要修改，需要与设备树对应。
// #define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
// // 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
// #define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)   
// #define PWM_1_PATH        ZF_PWM_MOTOR_1
// #define DIR_1_PATH        ZF_GPIO_MOTOR_1

// #define PWM_2_PATH        ZF_PWM_MOTOR_2
// #define DIR_2_PATH        ZF_GPIO_MOTOR_2
// zf_driver_pit pit_timer;

// struct pwm_info drv8701e_pwm_1_info;
// struct pwm_info drv8701e_pwm_2_info;


// zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
// zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
// zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
// zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);
// // PID初始化函数
// void PID_Init(PID_Datatypedef *sptr)
// {
//     sptr->P = 0;         // 初始化比例系数
//     sptr->I = 0;         // 初始化积分系数
//     sptr->D = 0;         // 初始化微分系数
//     sptr->LastError = 0; // 初始化上一次误差
//     sptr->PrevError = 0; // 初始化上上次误差
// }

// // 速度环
// PID_Datatypedef sptr_line;
// float debug_p = 10.0, debug_i = 5.0, debug_d = 0; // PID调试参数
// float speed_r;                                      // 右电机速度
// float speed_l;                                      // 左电机速度
// float line_speed = 0.0;                             // 质心线速度
// float target_speed = 120.0;
// float debug_t_speed = 100.0;
// #define PWM_PID_P 30.0
// #define PWM_PID_I 2.0
// #define PWM_PID_D 20.0
// #define ENCODER_QUAD_1_PATH           ZF_ENCODER_QUAD_1
// #define ENCODER_QUAD_2_PATH           ZF_ENCODER_QUAD_2

// // 创建编码器对象，传入文件路径
// zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);
// zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);
// // 编码器读取函数
// void encoder_Read()
// {
//     encoder_count_l = encoder_quad_1.get_count(); // 读取左电机编码器值
//     encoder_quad_1.clear_count();              // 清除左电机编码器计数
//     encoder_count_r = -encoder_quad_2.get_count(); // 读取右电机编码器值（反向）
//     encoder_quad_2.clear_count();                  // 清除右电机编码器计数
//     // 计算轮速
//     speed_l = 100.0 * (float)encoder_count_l / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // 左轮速度
//     speed_r = 100.0 * (float)encoder_count_r / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // 右轮速度
//     line_speed = (speed_l + speed_r) / 2;                                                                              // 质心线速度
//     // 计算行驶距离
//     encoder_distance += (speed_l + speed_r) / 2 * (PIT_60_0_PERIOD * 0.001); // 平均速度
// }
// //陀螺仪读取函数
// void imu_Read()
// {
//     imu_acc_x = imu_dev.get_acc_x();
//     imu_acc_y = imu_dev.get_acc_y();
//     imu_acc_z = imu_dev.get_acc_z();

//     imu_gyro_x = imu_dev.get_gyro_x();
//     imu_gyro_y = imu_dev.get_gyro_y();
//     imu_gyro_z = imu_dev.get_gyro_z();
//     // float data = imu660ra_gyro_transition(imu660ra_gyro_z);
//     float data = imu_gyro_x;
//     if (zero_point_count < IMU_ZERO_COUNT)
//     {
//         zero_point += data;
//         zero_point_count++;
//     }
//     else if (zero_point_count == IMU_ZERO_COUNT)
//     {
//         zero_point /= IMU_ZERO_COUNT;
//         zero_point_count++;
//     }
//     else
//     {
//         data -= zero_point;
//     }
//     yaw_speed = data; // 角速度
//     angle_yaw += data * 0.001 * PIT_60_1_PERIOD;
// //    if (angle_yaw > 180)
// //        angle_yaw -= 360;
// //    else if (angle_yaw < -180)
// //        angle_yaw += 360;
//     // printf("angle_yaw:%f\r\n",angle_yaw);
// }
// //增量式PID
// float PID_Inc(PID_Datatypedef *sptr, float Now, float Expect)
// {
//     float Increase; // PID 输出增量
//     float iError;   // 当前误差
//     iError = Expect - Now;
//     Increase = sptr->P * (iError - sptr->LastError) + sptr->I * iError + sptr->D * (iError - 2.0f * sptr->LastError + sptr->PrevError);

//     // 更新误差历史
//     sptr->PrevError = sptr->LastError;
//     sptr->LastError = iError;

//     return Increase;
// }
// //速度环pid输出
// void PID_Inc_Speed_Output(float target_line_speed, float current_line_speed)
// {
//     // 速度环
//     int16 pwm_line_add = PID_Inc(&sptr_line, current_line_speed, target_line_speed);
//     // 更新PWM值
//     pwm_l += pwm_line_add;
//     pwm_r += pwm_line_add;

// }
// void pwm_out_put()
// {
//         if(pwm_l >= 0)                                                           // 正转
//         {
//             drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
//             drv8701e_pwm_2.set_duty(pwm_l * (MOTOR2_PWM_DUTY_MAX / 100));       // 计算占空比


//         }
//         else
//         {
//             drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
//             drv8701e_pwm_2.set_duty(-pwm_l * (MOTOR2_PWM_DUTY_MAX / 100));      // 计算占空比

//         }
//         if(pwm_r >= 0)
//         {
//             drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
//             drv8701e_pwm_1.set_duty(pwm_r * (MOTOR1_PWM_DUTY_MAX / 100));       // 计算占空比


//         }
//         else
//         {
//             drv8701e_dir_1.set_level(0);                                      // DIR输出低电平
//             drv8701e_pwm_1.set_duty(-pwm_r * (MOTOR1_PWM_DUTY_MAX / 100));      // 计算占空比

//         }
// } 
// // 电机控制函数
// void motor_control()
// {
//     // 在此处进行初始化，在all_init.c中尽量只进行硬件初始化
//     static bool initialized = false; // 只在第一次进入时为 false

//     sptr_line.P = PWM_PID_P;
//     sptr_line.I = PWM_PID_I;
//     sptr_line.D = PWM_PID_D;
//     int16 pwm_r_add = 0; // 右电机PWM增量
//     int16 pwm_l_add = 0; // 左电机PWM增量

//     // angular_speed_control(error_image, yaw_speed); // 角速度环
//     PID_Inc_Speed_Output(target_speed, line_speed);      // 线速度环

//     // running_state_update(); // 更新运行状态,限制从零开始加速时的饱和值，防止过冲

//     // 限制PWM值的范围
//     pwm_l=limit_int(pwm_l,-MAX_DUTY,MAX_DUTY);
//     pwm_r=limit_int(pwm_r,-MAX_DUTY,MAX_DUTY);

//     pwm_out_put();
// }
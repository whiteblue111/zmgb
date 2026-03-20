// #ifndef CODE_MOTOR_H_
// #define CODE_MOTOR_H_
// #define PIT_60_0_PERIOD 10 // 50ms 编码器
// #define PIT_60_1_PERIOD 5 // 10ms imu，pid计算
// typedef struct
// {
//         float P;
//         float I;
//         float D;

//         float LastError;
//         float PrevError;

// } PID_Datatypedef;
// /* ---------------------------------- 函数原型声明 --------------------------------- */  

// void PID_Init(PID_Datatypedef *sptr);  
  

// void encoder_Read(void);  

// float PID_Inc(PID_Datatypedef *sptr, float Now, float Expect);  
  

// void PID_Inc_Speed_Output(float target_line_speed, float current_line_speed);  
  

// void pwm_out_put(void);  
  

// void motor_control(void);  
// #endif
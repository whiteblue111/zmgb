#include "zf_common_headfile.hpp"
#include "imu0.hpp"
#include <cstdio>
#include <cmath>

/* ============ 设备对象 ============ */
static zf_device_imu imu_dev;

/* ============ 全局状态 ============ */
volatile float g_yaw_speed = 0.0f;
volatile float g_angle_yaw = 0.0f;
volatile bool g_imu_ready = false;

/* ============ 模块私有 ============ */
static float s_zero_sum = 0.0f;
static float s_zero_point = 0.0f;
static int s_zero_count = 0;
static float s_yaw_lpf = 0.0f; // 低通滤波状态

volatile uint32_t g_imu_call_cnt = 0; // 当前1秒内调用次数
volatile uint32_t g_imu_call_hz = 0;  // 上一秒统计值

/* ============ 初始化 ============ */
void imu_init(void)
{
    imu_dev.init();

    if (DEV_IMU660RA == imu_dev.imu_type)
        printf("[imu] 检测到 IMU660RA\n");
    else
    {
        printf("[imu] ❌ 未检测到 IMU！\n");
        return;
    }

    s_zero_sum = 0.0f;
    s_zero_point = 0.0f;
    s_zero_count = 0;
    s_yaw_lpf = 0.0f;
    g_yaw_speed = 0.0f;
    g_angle_yaw = 0.0f;
    g_imu_ready = false;

    printf("[imu] 开始零漂校准，请保持静止...\n");
}

/* ============  ============ */
void imu_read(void)
{
    g_imu_call_cnt++;
    /* 1. 读取原始值 */
    int16 raw_z = imu_dev.get_gyro_z();

    /* 2. 零漂校准状态机（与参考代码一致） */
    if (s_zero_count < IMU_ZERO_COUNT)
    {
        s_zero_sum += (float)raw_z;
        s_zero_count++;
        g_yaw_speed = 0.0f;
        return; // 校准期间不积分
    }
    else if (s_zero_count == IMU_ZERO_COUNT)
    {
        s_zero_point = s_zero_sum / (float)IMU_ZERO_COUNT;
        s_zero_count++;
        g_imu_ready = true;
        g_angle_yaw = 0.0f;
        s_yaw_lpf = 0.0f;
        printf("[imu] 零漂校准完成: %.2f\n", s_zero_point);
        return;
    }
    float dt = IMU_DT_MS * 0.001f;
    float yaw_lsb = (float)raw_z - s_zero_point;
    float yaw_dps = yaw_lsb;
    // s_yaw_lpf = (1.0f - IMU_LPF_ALPHA) * s_yaw_lpf + IMU_LPF_ALPHA * yaw_dps;
    s_yaw_lpf = yaw_dps;
    float calibrated_dps = s_yaw_lpf;
    // if (fabsf(calibrated_dps) < 0.8f) calibrated_dps = 0.0f; // dps死区

    g_yaw_speed = calibrated_dps;       // 单位：deg/s
    g_angle_yaw += calibrated_dps * dt; // 单位：deg
}

/* ============ 工具函数 ============ */
void imu_reset_angle(void)
{
    g_angle_yaw = 0.0f;
}

float imu_get_yaw_speed(void)
{
    return g_yaw_speed;
}

float imu_get_angle(void)
{
    return g_angle_yaw;
}

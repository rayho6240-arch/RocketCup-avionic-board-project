#ifndef CORE_INC_EKF_H_
#define CORE_INC_EKF_H_

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define EKF_BUFFER_SIZE 100

// Memory partition macro for CCM RAM allocation
#define CCMRAM __attribute__((section(".ccmram")))

// Structure for a single 1000 Hz synchronized IMU + Baro sample
typedef struct {
    float ax, ay, az;       // Linear acceleration in body frame (m/s^2)
    float gx, gy, gz;       // Angular velocity in body frame (rad/s)
    uint32_t timestamp_us;  // Sub-microsecond timestamp from DWT
    uint8_t has_baro;       // 1 if barometer sample is present, 0 otherwise
    float baro_alt;         // Barometer altitude in meters
} EKF_Sample_t;

// Double-buffered container representing a 100ms EKF cycle
typedef struct {
    EKF_Sample_t samples[EKF_BUFFER_SIZE];
    uint32_t start_time_us;
} EKF_Buffer_t;

// State structure returned by the EKF
typedef struct {
    float pos_x, pos_y, pos_z;  // 3D Position in ENU (meters)
    float vel_x, vel_y, vel_z;  // 3D Velocity in ENU (m/s)
    float q[4];                 // 3D Attitude quaternion [qw, qx, qy, qz]
    float accel_bias[3];        // Estimated accelerometer biases (m/s^2)
} EKF_State_t;

// EKF thread attributes and static control definitions
extern const osThreadAttr_t EKF_Task_attributes;
extern osMessageQueueId_t xEKFQueue;

// Public EKF Status Flags for FSM Synchronization
extern uint8_t EKF_calibrated;
extern uint8_t EKF_in_flight;
extern uint8_t g_mag_yaw_lock;

// Public functions
void DWT_Init(void);
uint32_t DWT_GetMicroseconds(void);

void EKF_Task(void *argument);
void EKF_Init(void);
void EKF_ResetOrientation(void);
void EKF_Predict(float ax, float ay, float az, float dt);
void EKF_UpdateBaro(float baro_alt);
void EKF_UpdateBaroDelayed(float baro_alt, float z_pred);
void EKF_AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);

/* GPS 水平位置注入（由 defaultTask 呼叫，~1Hz）。
 * 校準完成後第一次有效定位鎖定發射台原點 (lat0/lon0)，之後以等距投影換算
 * ENU 水平位移並排程一次 (E/N) 位置量測更新，於 EKF_Task 內套用。
 * 多次提交僅保留最新一筆；衛星數越少量測噪聲 R 越大（越不信任）。 */
void EKF_SubmitGPS(int32_t lat_1e6, int32_t lon_1e6, uint8_t satellites);

/* 地磁計航向 (yaw) 注入（由 defaultTask 呼叫，~10Hz）。
 * mx,my,mz 為 IMU body frame 下的磁場向量（任意單位，僅取方向）。
 * 僅於發射台階段（校準後、未升空）做傾斜補償後的 yaw 互補修正，
 * 用以鎖定絕對航向並抑制重力回授無法觀測的陀螺 yaw 漂移；
 * 升空後不使用（與既有設計一致：飛行中姿態為純陀螺遞推）。
 * 注意：mx,my,mz 軸向須與 IMU body frame 對齊；若 bench 測試發現航向往反向
 * 收斂，於呼叫端對相應軸取負號即可。 */
void EKF_SubmitMag(float mx, float my, float mz);

float EKF_GetCPUUsage(void);
EKF_State_t EKF_GetState(void);

/* P0-C：EKF 防護（ekf_guard.h）健康介面。
 * EKF_GetHealthBits()==0 且 (now − EKF_GetLastUpdateTick()) ≤ 300ms 才視為 healthy；
 * unhealthy 時 FSM 切換 raw-baro 降級開傘鏈（fsm.c），遙測置 TELEM_FLAG_EKF_UNHEALTHY。 */
#include "ekf_guard.h"
uint8_t  EKF_GetHealthBits(void);
uint32_t EKF_GetLastUpdateTick(void);

void EKF_SaveCalibrationToFlash(void);
void EKF_LoadCalibrationFromFlash(void);
void EKF_HotRestartRestore(float last_altitude, float est_vel_z, const float *last_q);
void EKF_ResetCalibration(void);
void EKF_SaveMagCalibration(float cx, float cy, float cz);

#endif /* CORE_INC_EKF_H_ */

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

// Public functions
void DWT_Init(void);
uint32_t DWT_GetMicroseconds(void);

void EKF_Init(void);
void EKF_Predict(float ax, float ay, float az, float dt);
void EKF_UpdateBaro(float baro_alt);
void EKF_UpdateBaroDelayed(float baro_alt, float z_pred);
void EKF_AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);

EKF_State_t EKF_GetState(void);

void EKF_Task(void *argument);

#endif /* CORE_INC_EKF_H_ */

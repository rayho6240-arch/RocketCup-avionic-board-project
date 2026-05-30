#include "ekf.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// -------------------------------------------------------------------------
// Global EKF variables residing in CCM RAM
// -------------------------------------------------------------------------

// EKF state vector
// State elements:
// x[0] = Position East (m)
// x[1] = Position North (m)
// x[2] = Position Up (m) / Altitude
// x[3] = Velocity East (m/s)
// x[4] = Velocity North (m/s)
// x[5] = Velocity Up (m/s)
static float EKF_x[6] CCMRAM;

// Covariance matrix (6x6)
static float EKF_P[6][6] CCMRAM;

// Attitude quaternion (qw, qx, qy, qz)
static float EKF_q[4] CCMRAM;

// Dynamic stationary calibration variables
uint8_t EKF_calibrated CCMRAM = 0;
static uint32_t EKF_calib_samples CCMRAM = 0;
static float EKF_accel_bias[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_accel_sum[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_gyro_bias[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_gyro_sum[3] CCMRAM = {0.0f, 0.0f, 0.0f};

// Barometer launchpad reference altitude
static float EKF_baro_launchpad CCMRAM = 0.0f;
static float EKF_baro_sum CCMRAM = 0.0f;
static uint32_t EKF_baro_samples CCMRAM = 0;

// Flight state transition flag, launch counter, and rest counter
uint8_t EKF_in_flight CCMRAM = 0;
static uint32_t EKF_launch_counter CCMRAM = 0;
static uint32_t EKF_rest_counter CCMRAM = 0;

// Process noise covariances (diagonals)
static const float Q_pos = 0.005f;  // Position process noise variance
static const float Q_vel = 0.1f;    // Velocity process noise variance

// Measurement noise covariance for Barometer altitude
static const float R_baro = 0.36f;  // ~0.6m altitude measurement stddev

// Gravity constant in ENU
static const float GRAVITY = 9.80665f;

// -------------------------------------------------------------------------
// FreeRTOS Task and Queue definitions mapped to CCM RAM
// -------------------------------------------------------------------------
osMessageQueueId_t xEKFQueue;

static StaticTask_t EKF_TaskControlBlock CCMRAM;
static StackType_t EKF_TaskStack[1024] CCMRAM;

const osThreadAttr_t EKF_Task_attributes = {
  .name = "ekfTask",
  .cb_mem = &EKF_TaskControlBlock,
  .cb_size = sizeof(EKF_TaskControlBlock),
  .stack_mem = EKF_TaskStack,
  .stack_size = sizeof(EKF_TaskStack),
  .priority = (osPriority_t) osPriorityHigh,
};

// -------------------------------------------------------------------------
// DWT High-Resolution Clock Implementation (Cortex-M4 Cycle Counter)
// -------------------------------------------------------------------------

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetMicroseconds(void) {
    // STM32F407 clock runs at 168 MHz
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

// -------------------------------------------------------------------------
// EKF State & Math Functions
// -------------------------------------------------------------------------

void EKF_Init(void) {
    // Reset state to origin with zero velocity
    memset(EKF_x, 0, sizeof(EKF_x));

    // Initialize state covariance P as diagonal matrix
    memset(EKF_P, 0, sizeof(EKF_P));
    for (int i = 0; i < 3; i++) {
        EKF_P[i][i] = 1.0f;       // Initial position uncertainty (1.0 m^2)
        EKF_P[i+3][i+3] = 2.0f;   // Initial velocity uncertainty (2.0 m^2/s^2)
    }

    // Initialize attitude quaternion as identity (no rotation)
    EKF_q[0] = 1.0f;
    EKF_q[1] = 0.0f;
    EKF_q[2] = 0.0f;
    EKF_q[3] = 0.0f;

    // Reset calibration and launchpad variables
    EKF_calibrated = 0;
    EKF_calib_samples = 0;
    memset(EKF_accel_bias, 0, sizeof(EKF_accel_bias));
    memset(EKF_accel_sum, 0, sizeof(EKF_accel_sum));
    memset(EKF_gyro_bias, 0, sizeof(EKF_gyro_bias));
    memset(EKF_gyro_sum, 0, sizeof(EKF_gyro_sum));
    
    EKF_baro_launchpad = 0.0f;
    EKF_baro_sum = 0.0f;
    EKF_baro_samples = 0;
    
    EKF_in_flight = 0;
    EKF_launch_counter = 0;
    EKF_rest_counter = 0;
}

void EKF_AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float qw = EKF_q[0];
    float qx = EKF_q[1];
    float qy = EKF_q[2];
    float qz = EKF_q[3];

    // If we are stationary on the launchpad (not in flight), we apply a Mahony
    // Complementary Filter gravity feedback loop. This uses the accelerometer's
    // gravity vector to estimate roll and pitch errors, and feeds them back
    // to correct gyroscope drift, forcing the attitude to pull back to absolute level.
    if (!EKF_in_flight) {
        float norm_a = sqrtf(ax*ax + ay*ay + az*az);
        if (norm_a > 0.01f) {
            float ax_n = ax / norm_a;
            float ay_n = ay / norm_a;
            float az_n = az / norm_a;

            // Predicted gravity direction in the body frame (third row of R(q))
            float vx = 2.0f * (qx*qz - qw*qy);
            float vy = 2.0f * (qy*qz + qw*qx);
            float vz = 1.0f - 2.0f * (qx*qx + qy*qy);

            // Error is the cross product between measured gravity and predicted gravity
            float ex = (ay_n * vz - az_n * vy);
            float ey = (az_n * vx - ax_n * vz);
            float ez = (ax_n * vy - ay_n * vx);

            // Apply feedback correction (Kp = 2.0f pulls the tilt back to level within 1s)
            float Kp = 2.0f;
            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;
        }
    }

    // Quaternion derivative integration
    float d_qw = 0.5f * dt * (-qx * gx - qy * gy - qz * gz);
    float d_qx = 0.5f * dt * ( qw * gx + qy * gz - qz * gy);
    float d_qy = 0.5f * dt * ( qw * gy - qx * gz + qz * gx);
    float d_qz = 0.5f * dt * ( qw * gz + qx * gy - qy * gx);

    qw += d_qw;
    qx += d_qx;
    qy += d_qy;
    qz += d_qz;

    // Re-normalize quaternion to prevent accumulation of numerical errors
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 1e-6f) {
        EKF_q[0] = qw / norm;
        EKF_q[1] = qx / norm;
        EKF_q[2] = qy / norm;
        EKF_q[3] = qz / norm;
    }
}

void EKF_Predict(float ax, float ay, float az, float dt) {
    // 1. Get rotation matrix R from attitude quaternion (body to local ENU navigation frame)
    float qw = EKF_q[0];
    float qx = EKF_q[1];
    float qy = EKF_q[2];
    float qz = EKF_q[3];

    float r11 = 1.0f - 2.0f * (qy*qy + qz*qz);
    float r12 = 2.0f * (qx*qy - qw*qz);
    float r13 = 2.0f * (qx*qz + qw*qy);

    float r21 = 2.0f * (qx*qy + qw*qz);
    float r22 = 1.0f - 2.0f * (qx*qx + qz*qz);
    float r23 = 2.0f * (qy*qz - qw*qx);

    float r31 = 2.0f * (qx*qz - qw*qy);
    float r32 = 2.0f * (qy*qz + qw*qx);
    float r33 = 1.0f - 2.0f * (qx*qx + qy*qy);

    // 2. Rotate body acceleration into navigation frame
    float a_nav_x = r11 * ax + r12 * ay + r13 * az;
    float a_nav_y = r21 * ax + r22 * ay + r23 * az;
    float a_nav_z = r31 * ax + r32 * ay + r33 * az - GRAVITY; // Subtract gravity along ENU z-axis

    // 3. Propagate state vector x
    // p = p + v * dt + 0.5 * a * dt^2
    // v = v + a * dt
    float dt2 = 0.5f * dt * dt;
    EKF_x[0] += EKF_x[3] * dt + a_nav_x * dt2;
    EKF_x[1] += EKF_x[4] * dt + a_nav_y * dt2;
    EKF_x[2] += EKF_x[5] * dt + a_nav_z * dt2;

    EKF_x[3] += a_nav_x * dt;
    EKF_x[4] += a_nav_y * dt;
    EKF_x[5] += a_nav_z * dt;

    // 4. Analytically propagate 6x6 covariance matrix P = F * P * F^T + Q
    // Partition P into four 3x3 matrices: P_pp (top-left), P_pv (top-right), P_vp (bottom-left), P_vv (bottom-right)
    float P_pp_new[3][3];
    float P_pv_new[3][3];
    float P_vp_new[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // P_pp = P_pp + dt * (P_vp + P_pv) + dt^2 * P_vv
            P_pp_new[i][j] = EKF_P[i][j] + dt * (EKF_P[i+3][j] + EKF_P[i][j+3]) + dt * dt * EKF_P[i+3][j+3];
            // P_pv = P_pv + dt * P_vv
            P_pv_new[i][j] = EKF_P[i][j+3] + dt * EKF_P[i+3][j+3];
            // P_vp = P_vp + dt * P_vv
            P_vp_new[i][j] = EKF_P[i+3][j] + dt * EKF_P[i+3][j+3];
        }
    }

    // Apply updates back to P, adding process noise Q along the diagonal
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EKF_P[i][j] = P_pp_new[i][j];
            EKF_P[i][j+3] = P_pv_new[i][j];
            EKF_P[i+3][j] = P_vp_new[i][j];
        }
        EKF_P[i][i] += Q_pos;       // Add process noise to position diagonal
        EKF_P[i+3][i+3] += Q_vel;   // Add process noise to velocity diagonal
    }
}

// Group-delay compensated measurement update using the Analytical Joseph Form
void EKF_UpdateBaroDelayed(float baro_alt, float z_pred) {
    // 1. Innovation using prediction at correct historical time
    float y = baro_alt - z_pred;

    // 2. Innovation Covariance S = H * P * H^T + R = P[2][2] + R_baro
    float S = EKF_P[2][2] + R_baro;
    if (S < 1e-6f) return; // Prevent division by zero

    float invS = 1.0f / S;

    // 3. Kalman Gain K = P * H^T * invS (H^T is the 3rd column of P, index 2)
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = EKF_P[i][2] * invS;
    }

    // 4. State correction x = x + K * y
    for (int i = 0; i < 6; i++) {
        EKF_x[i] += K[i] * y;
    }

    // 5. Covariance correction using robust Analytical Joseph Form:
    // P_new = (I - KH)*P*(I - KH)^T + K*R_baro*K^T
    // First, compute intermediate P' = (I - K*H)*P
    float P_prime[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_prime[i][j] = EKF_P[i][j] - K[i] * EKF_P[2][j];
        }
    }

    // Then compute P_new[i][j] = P'[i][j] - P'[i][2]*K[j] + K[i]*R_baro*K[j]
    for (int i = 0; i < 6; i++) {
        float P_prime_i2 = P_prime[i][2];
        for (int j = 0; j < 6; j++) {
            EKF_P[i][j] = P_prime[i][j] - P_prime_i2 * K[j] + K[i] * R_baro * K[j];
        }
    }
}

// Fallback interface to support standard direct barometer updates
void EKF_UpdateBaro(float baro_alt) {
    float baro_rel = baro_alt - EKF_baro_launchpad;
    EKF_UpdateBaroDelayed(baro_rel, EKF_x[2]);
}

EKF_State_t EKF_GetState(void) {
    EKF_State_t state;
    state.pos_x = EKF_x[0];
    state.pos_y = EKF_x[1];
    state.pos_z = EKF_x[2];
    state.vel_x = EKF_x[3];
    state.vel_y = EKF_x[4];
    state.vel_z = EKF_x[5];
    memcpy(state.q, EKF_q, sizeof(EKF_q));
    memcpy(state.accel_bias, EKF_accel_bias, sizeof(state.accel_bias));
    return state;
}

// Helper function to format floats safely using fast integer arithmetic
// Incorporates robust NaN, Infinity, and Overflow checks to prevent labs() HardFaults
static void EKF_PrintFloat(float val, char separator) {
    if (isnan(val)) {
        printf("NaN%c", separator);
        return;
    }
    if (isinf(val)) {
        if (val < 0.0f) {
            printf("-INF%c", separator);
        } else {
            printf("INF%c", separator);
        }
        return;
    }

    float scaled = val * 1000.0f;
    // Bound check against INT32 limits to prevent undefined casting/labs behavior
    if (scaled > 2147483647.0f) {
        printf("OVR%c", separator);
        return;
    }
    if (scaled < -2147483647.0f) {
        printf("-OVR%c", separator);
        return;
    }

    int32_t val_mm = (int32_t)scaled;
    if (val_mm < 0) {
        printf("-%ld.%03ld%c", labs(val_mm) / 1000, labs(val_mm) % 1000, separator);
    } else {
        printf("%ld.%03ld%c", val_mm / 1000, val_mm % 1000, separator);
    }
}

// -------------------------------------------------------------------------
// EKF Thread Implementation (Waiting for pointers via Queue)
// -------------------------------------------------------------------------

void EKF_Task(void *argument) {
    (void)argument;
    EKF_Buffer_t* p_buf = NULL;
    uint32_t last_timestamp_us = 0;

    // Local circular history buffer for 20ms barometer group-delay compensation
    // 20ms delay at 1000Hz = 20 samples. circular buffer size 25 is safe.
    float z_history[25] = {0.0f};
    uint8_t z_history_idx = 0;

    EKF_Init();

    // Enable CPU Cycle Counter for microsecond timestamps
    DWT_Init();

    printf("[EKF] EKF High-Performance Task successfully spawned in CCM RAM!\r\n");

    for (;;) {
        // Wait forever until a pointer to a filled buffer is placed on the queue
        if (osMessageQueueGet(xEKFQueue, &p_buf, NULL, osWaitForever) == osOK) {
            if (p_buf == NULL) continue;

            uint32_t start_cycles = DWT->CYCCNT;

            // Playback the EKF on the 100 accumulated samples sequentially
            for (int i = 0; i < EKF_BUFFER_SIZE; i++) {
                EKF_Sample_t* sample = &p_buf->samples[i];

                float dt;
                if (last_timestamp_us == 0) {
                    dt = 0.001f; // Standard 1000 Hz interval as baseline
                } else {
                    // Compute dt from microsecond clock difference
                    uint32_t diff = sample->timestamp_us - last_timestamp_us;
                    dt = (float)diff * 1e-6f;
                }
                last_timestamp_us = sample->timestamp_us;

                // Protect against outliers/system startup timing gaps
                if (dt <= 0.0f || dt > 0.05f) {
                    dt = 0.001f;
                }

                // --- 1. Dynamic Accelerometer & Barometer Stationary Calibration Phase ---
                if (!EKF_calibrated) {
                    EKF_accel_sum[0] += sample->ax;
                    EKF_accel_sum[1] += sample->ay;
                    EKF_accel_sum[2] += sample->az;
                    EKF_gyro_sum[0] += sample->gx;
                    EKF_gyro_sum[1] += sample->gy;
                    EKF_gyro_sum[2] += sample->gz;
                    EKF_calib_samples++;

                    if (sample->has_baro) {
                        EKF_baro_sum += sample->baro_alt;
                        EKF_baro_samples++;
                    }

                    if (EKF_calib_samples >= 3000) { // 3 seconds at 1000Hz
                        EKF_accel_bias[0] = EKF_accel_sum[0] / 3000.0f;
                        EKF_accel_bias[1] = EKF_accel_sum[1] / 3000.0f;
                        
                        // Universal bias calculation: forces az_corr to equal +g when stationary upright
                        float avg_az = EKF_accel_sum[2] / 3000.0f;
                        EKF_accel_bias[2] = avg_az - GRAVITY;

                        EKF_gyro_bias[0] = EKF_gyro_sum[0] / 3000.0f;
                        EKF_gyro_bias[1] = EKF_gyro_sum[1] / 3000.0f;
                        EKF_gyro_bias[2] = EKF_gyro_sum[2] / 3000.0f;
                        
                        if (EKF_baro_samples > 0) {
                            EKF_baro_launchpad = EKF_baro_sum / (float)EKF_baro_samples;
                        } else {
                            EKF_baro_launchpad = 0.0f;
                        }
                        
                        EKF_calibrated = 1;
                        
                        printf("[EKF] Stationary Calibration Done!\r\n");
                        printf("  -> Accel Bias X:");
                        EKF_PrintFloat(EKF_accel_bias[0], ' ');
                        printf("Y:");
                        EKF_PrintFloat(EKF_accel_bias[1], ' ');
                        printf("Z:");
                        EKF_PrintFloat(EKF_accel_bias[2], '\r\n');
                        printf("  -> Gyro Bias X:");
                        EKF_PrintFloat(EKF_gyro_bias[0], ' ');
                        printf("Y:");
                        EKF_PrintFloat(EKF_gyro_bias[1], ' ');
                        printf("Z:");
                        EKF_PrintFloat(EKF_gyro_bias[2], '\r\n');
                        printf("  -> Launchpad Baro Alt: ");
                        EKF_PrintFloat(EKF_baro_launchpad, ' ');
                        printf("m\r\n");
                    }
                    
                    // Maintain pre-launch state lock
                    memset(EKF_x, 0, sizeof(EKF_x));
                    EKF_q[0] = 1.0f; EKF_q[1] = 0.0f; EKF_q[2] = 0.0f; EKF_q[3] = 0.0f;
                    continue; // Skip propagation during active calibration
                }

                // --- 2. Correct Sensor Biases ---
                float ax_corr = sample->ax - EKF_accel_bias[0];
                float ay_corr = sample->ay - EKF_accel_bias[1];
                float az_corr = sample->az - EKF_accel_bias[2];

                float gx_corr = sample->gx - EKF_gyro_bias[0];
                float gy_corr = sample->gy - EKF_gyro_bias[1];
                float gz_corr = sample->gz - EKF_gyro_bias[2];

                // --- 3. Attitude Integration with Mahony Gravity Feedback ---
                EKF_AttitudeUpdate(gx_corr, gy_corr, gz_corr, ax_corr, ay_corr, az_corr, dt);

                // --- 4. Linear Propagation (Predict) ---
                EKF_Predict(ax_corr, ay_corr, az_corr, dt);

                // --- 5. Stationary Launchpad Lock (ZUPT) with Baro-Only Trigger ---
                if (!EKF_in_flight) {
                    float relative_baro_alt = 0.0f;
                    if (sample->has_baro) {
                        relative_baro_alt = sample->baro_alt - EKF_baro_launchpad;
                    }

                    // For desktop testing, we only trigger launch when altitude change exceeds 5 meters
                    if (sample->has_baro && relative_baro_alt > 5.0f) {
                        EKF_in_flight = 1;
                        printf("[EKF] Launch Detected! (Baro relative altitude: ");
                        EKF_PrintFloat(relative_baro_alt, ' ');
                        printf("m)\r\n");
                    } else {
                        // Force states to 0 to eliminate all pre-launch horizontal and vertical drift
                        memset(EKF_x, 0, sizeof(EKF_x));
                    }
                }

                // --- 6. Prior State Clipping (Physical Constraints) ---
                if (EKF_in_flight) {
                    if (EKF_x[2] < 0.0f) {
                        EKF_x[2] = 0.0f; // Altitude cannot go below ground level
                        if (EKF_x[5] < 0.0f) {
                            EKF_x[5] = 0.0f; // Velocity cannot go negative at the ground boundary
                        }
                    }
                }

                // --- 7. Save predicted Z-altitude to historical circular buffer ---
                z_history[z_history_idx] = EKF_x[2];
                z_history_idx = (z_history_idx + 1) % 25;

                // --- 8. Delayed Measurement Update from Barometer ---
                if (sample->has_baro) {
                    float z_pred;
                    if (EKF_calib_samples > 20) {
                        // Look back exactly 20 samples ago (20ms group delay at 1000Hz)
                        uint8_t hist_idx = (z_history_idx + 25 - 20) % 25;
                        z_pred = z_history[hist_idx];
                    } else {
                        z_pred = EKF_x[2];
                    }

                    float baro_rel = sample->baro_alt - EKF_baro_launchpad;
                    EKF_UpdateBaroDelayed(baro_rel, z_pred);
                }

                // --- 9. Dynamic Rest / Stationary Detector ---
                if (EKF_in_flight) {
                    float gyro_mag = sqrtf(gx_corr*gx_corr + gy_corr*gy_corr + gz_corr*gz_corr);
                    float accel_mag = sqrtf(ax_corr*ax_corr + ay_corr*ay_corr + az_corr*az_corr);

                    // Quiet gyro and gravity magnitude close to 1g
                    if (gyro_mag < 0.15f && fabsf(accel_mag - GRAVITY) < 0.8f) {
                        EKF_rest_counter++;
                    } else {
                        if (EKF_rest_counter > 0) {
                            EKF_rest_counter--;
                        }
                    }

                    // Reset to launchpad mode if resting still for 1.0 second (1000 samples)
                    if (EKF_rest_counter >= 1000) {
                        EKF_in_flight = 0;
                        EKF_launch_counter = 0;
                        EKF_rest_counter = 0;
                        memset(EKF_x, 0, sizeof(EKF_x)); // Reset position and velocity to 0 to stop drift
                        printf("[EKF] Rest/Stationary Detected! Resetting to Launchpad Mode.\r\n");
                    }
                } else {
                    EKF_rest_counter = 0;
                }
            }

            uint32_t end_cycles = DWT->CYCCNT;
            uint32_t elapsed_cycles = end_cycles - start_cycles;
            float elapsed_ms = (float)elapsed_cycles / (float)SystemCoreClock * 1000.0f;

            // Report estimated position, velocity, attitude and execution duration
            EKF_State_t current = EKF_GetState();

            // 1. Output diagnostic logs [EKF]
            printf("[EKF] playback done in ");
            EKF_PrintFloat(elapsed_ms, ' ');
            printf("ms. EST H:");
            EKF_PrintFloat(current.pos_z, ' ');
            printf("v:");
            EKF_PrintFloat(current.vel_z, ' ');
            printf("bias:[");
            EKF_PrintFloat(current.accel_bias[0], ',');
            EKF_PrintFloat(current.accel_bias[1], ',');
            EKF_PrintFloat(current.accel_bias[2], ']');
            printf("\r\n");

            // 2. Output parseable telemetry packets [TELE]
            printf("[TELE] pos:");
            EKF_PrintFloat(current.pos_x, ',');
            EKF_PrintFloat(current.pos_y, ',');
            EKF_PrintFloat(current.pos_z, ' ');
            printf("vel:");
            EKF_PrintFloat(current.vel_x, ',');
            EKF_PrintFloat(current.vel_y, ',');
            EKF_PrintFloat(current.vel_z, ' ');
            printf("q:");
            EKF_PrintFloat(current.q[0], ',');
            EKF_PrintFloat(current.q[1], ',');
            EKF_PrintFloat(current.q[2], ',');
            EKF_PrintFloat(current.q[3], '\r'); // carriage return + newline printed next
            printf("\n");
        }
    }
}

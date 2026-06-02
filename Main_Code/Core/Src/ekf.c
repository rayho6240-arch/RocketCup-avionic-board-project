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

// --- GPS horizontal-position fusion (East/North) ---
// Written by defaultTask via EKF_SubmitGPS(), consumed once per buffer in EKF_Task.
static volatile uint8_t EKF_gps_pending CCMRAM = 0;     // 1 = a measurement awaits application
static uint8_t  EKF_gps_origin_set CCMRAM = 0;          // 1 = launchpad lat/lon origin captured
static int32_t  EKF_gps_lat0_1e6 CCMRAM = 0;            // launchpad origin latitude  (deg x1e6)
static int32_t  EKF_gps_lon0_1e6 CCMRAM = 0;            // launchpad origin longitude (deg x1e6)
static float    EKF_gps_coslat0 CCMRAM = 1.0f;          // cos(lat0): East-scale for equirectangular proj
static float    EKF_gps_meas_E CCMRAM = 0.0f;           // pending measured East  (m, rel. launchpad)
static float    EKF_gps_meas_N CCMRAM = 0.0f;           // pending measured North (m, rel. launchpad)
static float    EKF_gps_R CCMRAM = 25.0f;               // pending measurement variance (m^2)

// --- Magnetometer heading (yaw) fusion ---
// Written by defaultTask via EKF_SubmitMag(), consumed once per buffer in EKF_Task.
static volatile uint8_t EKF_mag_pending CCMRAM = 0;     // 1 = a mag vector awaits application
static float    EKF_mag_x CCMRAM = 0.0f;                // body-frame magnetic field X
static float    EKF_mag_y CCMRAM = 0.0f;                // body-frame magnetic field Y
static float    EKF_mag_z CCMRAM = 0.0f;                // body-frame magnetic field Z

// Process noise covariances (diagonals)
static const float Q_pos = 0.005f;  // Position process noise variance
static const float Q_vel = 0.1f;    // Velocity process noise variance

// Measurement noise covariance for Barometer altitude
static const float R_baro = 0.36f;  // ~0.6m altitude measurement stddev

// Gravity constant in ENU
static const float GRAVITY = 9.80665f;

// GPS equirectangular-projection constants
static const float EARTH_RADIUS_M = 6378137.0f;            // WGS84 semi-major axis
static const float DEG2RAD = 0.017453292519943295f;        // pi / 180

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

    // GPS fusion: clear origin/pending so a fresh launchpad origin is captured
    EKF_gps_pending = 0;
    EKF_gps_origin_set = 0;
    EKF_gps_lat0_1e6 = 0;
    EKF_gps_lon0_1e6 = 0;
    EKF_gps_coslat0 = 1.0f;
    EKF_gps_meas_E = 0.0f;
    EKF_gps_meas_N = 0.0f;
    EKF_gps_R = 25.0f;

    // Magnetometer fusion: clear pending
    EKF_mag_pending = 0;
    EKF_mag_x = 0.0f;
    EKF_mag_y = 0.0f;
    EKF_mag_z = 0.0f;
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

// Magnetometer heading (yaw) correction, Mahony-style, in the SAME body->nav
// rotation convention as EKF_AttitudeUpdate/EKF_Predict (a_nav = R*a_body, with
// the gravity/up direction in body = third row of R). The horizontal-collapse of
// the reference field constrains heading only, leaving roll/pitch to gravity.
// Applied on the launchpad only, to pin absolute heading and arrest the gyro yaw
// drift that accelerometer (gravity-only) feedback cannot observe.
//   mx,my,mz : magnetometer vector in the IMU body frame (any unit; direction only)
//   dt       : integration interval for the complementary correction (s)
static void EKF_MagYawUpdate(float mx, float my, float mz, float dt) {
    float nm = sqrtf(mx*mx + my*my + mz*mz);
    if (nm < 1e-6f) return;                 // degenerate / no field
    float mxn = mx / nm, myn = my / nm, mzn = mz / nm;

    float qw = EKF_q[0], qx = EKF_q[1], qy = EKF_q[2], qz = EKF_q[3];

    // Rotation matrix R (body -> nav), identical to EKF_Predict
    float r11 = 1.0f - 2.0f*(qy*qy + qz*qz);
    float r12 = 2.0f*(qx*qy - qw*qz);
    float r13 = 2.0f*(qx*qz + qw*qy);
    float r21 = 2.0f*(qx*qy + qw*qz);
    float r22 = 1.0f - 2.0f*(qx*qx + qz*qz);
    float r23 = 2.0f*(qy*qz - qw*qx);
    float r31 = 2.0f*(qx*qz - qw*qy);
    float r32 = 2.0f*(qy*qz + qw*qx);
    float r33 = 1.0f - 2.0f*(qx*qx + qy*qy);

    // Rotate body mag into nav frame: h = R * m
    float hx = r11*mxn + r12*myn + r13*mzn;
    float hy = r21*mxn + r22*myn + r23*mzn;
    float hz = r31*mxn + r32*myn + r33*mzn;

    // Reference field b = [bx, 0, bz]: horizontal magnitude collapsed onto nav-East,
    // measured inclination kept on Up. Fixes the heading origin without touching tilt.
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    // Predicted mag direction back in body frame: w = R^T * b   (by = 0)
    float wx = r11*bx + r31*bz;
    float wy = r12*bx + r32*bz;
    float wz = r13*bx + r33*bz;

    // Error = measured x predicted (body frame); predominantly a yaw error vector
    float ex = (myn*wz - mzn*wy);
    float ey = (mzn*wx - mxn*wz);
    float ez = (mxn*wy - myn*wx);

    // Apply as a gyro-like correction integrated over dt (Kp_mag tunable on bench)
    const float Kp_mag = 1.0f;
    float gx = Kp_mag * ex;
    float gy = Kp_mag * ey;
    float gz = Kp_mag * ez;

    float d_qw = 0.5f * dt * (-qx*gx - qy*gy - qz*gz);
    float d_qx = 0.5f * dt * ( qw*gx + qy*gz - qz*gy);
    float d_qy = 0.5f * dt * ( qw*gy - qx*gz + qz*gx);
    float d_qz = 0.5f * dt * ( qw*gz + qx*gy - qy*gx);

    qw += d_qw; qx += d_qx; qy += d_qy; qz += d_qz;
    float qn = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (qn > 1e-6f) {
        EKF_q[0] = qw/qn; EKF_q[1] = qx/qn; EKF_q[2] = qy/qn; EKF_q[3] = qz/qn;
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

// Scalar position measurement update on a single state axis (idx: 0=E, 1=N, 2=U)
// using the same Analytical Joseph Form as the barometer update. H selects row idx.
static void EKF_UpdateScalarPos(int idx, float z, float R) {
    float y = z - EKF_x[idx];               // Innovation
    float S = EKF_P[idx][idx] + R;          // Innovation covariance
    if (S < 1e-6f) return;
    float invS = 1.0f / S;

    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = EKF_P[i][idx] * invS;        // Kalman gain (column idx of P)
    }
    for (int i = 0; i < 6; i++) {
        EKF_x[i] += K[i] * y;               // State correction
    }

    // Joseph form: P_new = (I-KH)P(I-KH)^T + K R K^T
    float P_prime[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_prime[i][j] = EKF_P[i][j] - K[i] * EKF_P[idx][j];
        }
    }
    for (int i = 0; i < 6; i++) {
        float P_prime_ii = P_prime[i][idx];
        for (int j = 0; j < 6; j++) {
            EKF_P[i][j] = P_prime[i][j] - P_prime_ii * K[j] + K[i] * R * K[j];
        }
    }
}

// Apply the pending GPS horizontal fix as two independent scalar updates on
// East (idx 0) and North (idx 1). Altitude (Up) is left to the barometer, which
// is far more accurate than GPS vertical. Called from EKF_Task only.
static void EKF_UpdateGPS(float meas_E, float meas_N, float R) {
    EKF_UpdateScalarPos(0, meas_E, R);
    EKF_UpdateScalarPos(1, meas_N, R);
}

// GPS injection from defaultTask (~1 Hz). Captures the launchpad origin on the
// first valid post-calibration fix, then publishes a pending ENU measurement.
void EKF_SubmitGPS(int32_t lat_1e6, int32_t lon_1e6, uint8_t satellites) {
    // Only fuse once stationary calibration is done, so the GPS origin coincides
    // with the same launchpad reference the rest of the EKF state is relative to.
    if (!EKF_calibrated) return;

    // First valid fix defines the launchpad origin (dE = dN = 0 by definition).
    if (!EKF_gps_origin_set) {
        EKF_gps_lat0_1e6 = lat_1e6;
        EKF_gps_lon0_1e6 = lon_1e6;
        EKF_gps_coslat0  = cosf((float)lat_1e6 * 1e-6f * DEG2RAD);
        EKF_gps_origin_set = 1;
        return;
    }

    // Equirectangular projection about the origin (accurate to <0.1% over the
    // few-km horizontal extent of a sounding-rocket flight).
    float dlat_deg = (float)(lat_1e6 - EKF_gps_lat0_1e6) * 1e-6f;
    float dlon_deg = (float)(lon_1e6 - EKF_gps_lon0_1e6) * 1e-6f;
    float meas_N = dlat_deg * DEG2RAD * EARTH_RADIUS_M;
    float meas_E = dlon_deg * DEG2RAD * EARTH_RADIUS_M * EKF_gps_coslat0;

    // Inflate measurement noise when the fix is weak (few satellites).
    float R;
    if      (satellites >= 8) R = 6.25f;    // ~2.5 m sigma
    else if (satellites >= 6) R = 25.0f;    // ~5 m
    else if (satellites >= 4) R = 100.0f;   // ~10 m
    else                      R = 2500.0f;  // barely trust a 3-sat fix

    // Publish atomically; EKF_Task (higher priority) may preempt mid-write.
    taskENTER_CRITICAL();
    EKF_gps_meas_E = meas_E;
    EKF_gps_meas_N = meas_N;
    EKF_gps_R = R;
    EKF_gps_pending = 1;
    taskEXIT_CRITICAL();
}

// Magnetometer injection from defaultTask (~10 Hz). Publishes the latest body-
// frame field vector; EKF_Task applies it as a pad-phase yaw correction.
void EKF_SubmitMag(float mx, float my, float mz) {
    taskENTER_CRITICAL();
    EKF_mag_x = mx;
    EKF_mag_y = my;
    EKF_mag_z = mz;
    EKF_mag_pending = 1;
    taskEXIT_CRITICAL();
}

EKF_State_t EKF_GetState(void) {
    EKF_State_t state;
    /* 改善項目 O：原子快照。EKF_Task（High prio）會連續寫入 EKF_x/EKF_q/EKF_accel_bias，
     * 而 defaultTask（Normal prio）於遙測 / FSM / SD 路徑呼叫本函式讀取；若不保護，
     * High-prio 任務可能在拷貝中途搶佔，造成「位置取自舊一輪、速度取自新一輪」的撕裂讀取。
     * 以臨界區封住拷貝期間（純記憶體搬移，數十週期），讓回傳的 state 為一致快照。
     * 註：EKF_Task 內部亦呼叫本函式（同任務），FreeRTOS 臨界區可巢狀，安全。 */
    taskENTER_CRITICAL();
    state.pos_x = EKF_x[0];
    state.pos_y = EKF_x[1];
    state.pos_z = EKF_x[2];
    state.vel_x = EKF_x[3];
    state.vel_y = EKF_x[4];
    state.vel_z = EKF_x[5];
    memcpy(state.q, EKF_q, sizeof(EKF_q));
    memcpy(state.accel_bias, EKF_accel_bias, sizeof(state.accel_bias));
    taskEXIT_CRITICAL();
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

            // --- Pending GPS horizontal-position update (applied once per buffer) ---
            // Consume atomically. Only fuse during flight: on the pad the ZUPT lock
            // zeroes horizontal state every sample, so a pad-time fix would be wiped
            // anyway — drop it and wait for fresh in-flight fixes.
            if (EKF_gps_pending) {
                float gps_E, gps_N, gps_R;
                taskENTER_CRITICAL();
                gps_E = EKF_gps_meas_E;
                gps_N = EKF_gps_meas_N;
                gps_R = EKF_gps_R;
                EKF_gps_pending = 0;
                taskEXIT_CRITICAL();

                if (EKF_in_flight) {
                    EKF_UpdateGPS(gps_E, gps_N, gps_R);
                }
            }

            // --- Pending magnetometer heading (yaw) correction (pad phase only) ---
            // Consume atomically. Only correct heading before launch (calibrated &&
            // !in_flight); in flight the attitude is gyro dead-reckoning by design,
            // and the rocket's spin/EMI make the magnetometer unreliable.
            if (EKF_mag_pending) {
                float m_x, m_y, m_z;
                taskENTER_CRITICAL();
                m_x = EKF_mag_x;
                m_y = EKF_mag_y;
                m_z = EKF_mag_z;
                EKF_mag_pending = 0;
                taskEXIT_CRITICAL();

                if (EKF_calibrated && !EKF_in_flight) {
                    EKF_MagYawUpdate(m_x, m_y, m_z, 0.1f);  // ~10Hz buffer cadence
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

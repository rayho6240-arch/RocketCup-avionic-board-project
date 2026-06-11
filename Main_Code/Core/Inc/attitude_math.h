#ifndef CORE_INC_ATTITUDE_MATH_H_
#define CORE_INC_ATTITUDE_MATH_H_

#include <math.h>

/*
 * attitude_math.h — 純姿態數學（TRIAD 初始化 + 四元數工具）
 * ============================================================
 * 無硬體相依，firmware 與 host 測試共用，避免兩邊各寫一份而漂移。
 *
 * 慣例：
 *   - 四元數 q = [qw,qx,qy,qz]，Hamilton 慣例，代表 body->nav 旋轉。
 *   - nav frame = ENU：X=East, Y=North, Z=Up。
 *   - body frame = (右, 前, 上)，見 sensor_axis.h。
 *   - 旋轉矩陣 R：v_nav = R * v_body（與 ekf.c EKF_Predict 一致）。
 */

/* 四元數 -> 旋轉矩陣 R (body->nav)。q 需已正規化或本函式內部正規化。 */
static inline void att_quat_to_R(const float q[4], float R[3][3])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) n = 1.0f;
    float qw = q[0]/n, qx = q[1]/n, qy = q[2]/n, qz = q[3]/n;

    R[0][0] = 1.0f - 2.0f*(qy*qy + qz*qz);
    R[0][1] = 2.0f*(qx*qy - qw*qz);
    R[0][2] = 2.0f*(qx*qz + qw*qy);
    R[1][0] = 2.0f*(qx*qy + qw*qz);
    R[1][1] = 1.0f - 2.0f*(qx*qx + qz*qz);
    R[1][2] = 2.0f*(qy*qz - qw*qx);
    R[2][0] = 2.0f*(qx*qz - qw*qy);
    R[2][1] = 2.0f*(qy*qz + qw*qx);
    R[2][2] = 1.0f - 2.0f*(qx*qx + qy*qy);
}

/* 3x3 旋轉矩陣 -> 四元數（Shepperd's method，數值穩定）。 */
static inline void att_R_to_quat(const float R[3][3], float q_out[4])
{
    float r11=R[0][0], r12=R[0][1], r13=R[0][2];
    float r21=R[1][0], r22=R[1][1], r23=R[1][2];
    float r31=R[2][0], r32=R[2][1], r33=R[2][2];
    float trace = r11 + r22 + r33;
    float qw, qx, qy, qz;

    if (trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        qw = 0.25f / s;
        qx = (r32 - r23) * s;
        qy = (r13 - r31) * s;
        qz = (r21 - r12) * s;
    } else if ((r11 > r22) && (r11 > r33)) {
        float s = 2.0f * sqrtf(1.0f + r11 - r22 - r33);
        qw = (r32 - r23) / s;
        qx = 0.25f * s;
        qy = (r12 + r21) / s;
        qz = (r13 + r31) / s;
    } else if (r22 > r33) {
        float s = 2.0f * sqrtf(1.0f + r22 - r11 - r33);
        qw = (r13 - r31) / s;
        qx = (r12 + r21) / s;
        qy = 0.25f * s;
        qz = (r23 + r32) / s;
    } else {
        float s = 2.0f * sqrtf(1.0f + r33 - r11 - r22);
        qw = (r21 - r12) / s;
        qx = (r13 + r31) / s;
        qy = (r23 + r32) / s;
        qz = 0.25f * s;
    }
    float qn = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (qn < 1e-9f) { q_out[0]=1; q_out[1]=q_out[2]=q_out[3]=0; return; }
    q_out[0]=qw/qn; q_out[1]=qx/qn; q_out[2]=qy/qn; q_out[3]=qz/qn;
}

/*
 * TRIAD：由 body-frame 重力向量 + 磁場向量求 body->nav(ENU) 初始四元數。
 *   grav_b : body 重力（加速度計靜態讀數；直立時應為 +Z_body，即朝上 = 反作用力）。
 *   mag_b  : body 磁場向量（任意單位，僅取方向）。
 *   q_out  : [qw,qx,qy,qz]，body->nav。
 * 回傳 1 = 成功；0 = 退化（重力或磁場太小、或兩者平行），此時 q_out 不變。
 *
 * 參考向量（nav ENU）：
 *   重力反作用力 -> +Up = (0,0,1)
 *   磁場水平分量 -> +North = (0,1,0)（僅用水平投影，忽略傾角 dip）
 */
static inline int att_triad_grav_mag_to_quat(const float grav_b[3],
                                             const float mag_b[3],
                                             float q_out[4])
{
    /* g_hat：body 內「朝上」單位向量 */
    float gn = sqrtf(grav_b[0]*grav_b[0] + grav_b[1]*grav_b[1] + grav_b[2]*grav_b[2]);
    if (gn < 1e-6f) return 0;
    float gx = grav_b[0]/gn, gy = grav_b[1]/gn, gz = grav_b[2]/gn;

    /* 磁場正規化 */
    float mn = sqrtf(mag_b[0]*mag_b[0] + mag_b[1]*mag_b[1] + mag_b[2]*mag_b[2]);
    if (mn < 1e-6f) return 0;
    float mx = mag_b[0]/mn, my = mag_b[1]/mn, mz = mag_b[2]/mn;

    /* 傾角補償：把磁場投影到水平面（與 g 垂直），得 body 內「朝北」方向 h_hat */
    float dot = mx*gx + my*gy + mz*gz;
    float hx = mx - dot*gx, hy = my - dot*gy, hz = mz - dot*gz;
    float hnn = sqrtf(hx*hx + hy*hy + hz*hz);
    if (hnn < 1e-6f) return 0;   /* 磁場與重力平行（指向天/地），無法定 heading */
    hx /= hnn; hy /= hnn; hz /= hnn;

    /* East = North × Up = h × g（ENU 右手定則；務必 h×g，不是 g×h） */
    float ex = hy*gz - hz*gy;
    float ey = hz*gx - hx*gz;
    float ez = hx*gy - hy*gx;

    /* R (body->nav) 以「列」組成：每一列 = 某 nav 軸在 body frame 的方向。
     *   row0 = East, row1 = North, row2 = Up
     * 驗證：R*g_hat=(0,0,1), R*h_hat=(0,1,0), R*e_hat=(1,0,0)。 */
    float R[3][3];
    R[0][0]=ex; R[0][1]=ey; R[0][2]=ez;   /* East  */
    R[1][0]=hx; R[1][1]=hy; R[1][2]=hz;   /* North */
    R[2][0]=gx; R[2][1]=gy; R[2][2]=gz;   /* Up    */

    att_R_to_quat(R, q_out);
    return 1;
}

#endif /* CORE_INC_ATTITUDE_MATH_H_ */

#ifndef LOAD_2DOF_COMMON_H
#define LOAD_2DOF_COMMON_H

#include <math.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <Eigen/Dense>

using namespace Eigen;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum ControlMode {
    MODE_FREEFALL = 0,
    MODE_GRAVITY_COMP = 1,
    MODE_PID_TRACKING = 2
};

typedef struct {
    double m1, m2;
    double L1, L2;
    double r1, r2;
    double g;
    double B[2];     // 점성 마찰
    double Kp[2];    // P Gain 
    double Kd[2];    // D Gain 
    double init_q_rad[2];
} TwoDOFParams;

static inline int streq(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}

static inline int parse_mode(const char *s) {
    if (streq(s, "freefall") || streq(s, "no_torque")) return MODE_FREEFALL;
    if (streq(s, "gravity") || streq(s, "gravity_comp")) return MODE_GRAVITY_COMP;
    if (streq(s, "pid") || streq(s, "tracking")) return MODE_PID_TRACKING;
    return MODE_PID_TRACKING;
}

static inline const char* mode_to_string(int mode) {
    switch (mode) {
        case MODE_FREEFALL: return "freefall";
        case MODE_GRAVITY_COMP: return "gravity_comp";
        case MODE_PID_TRACKING: return "pid";
        default: return "unknown";
    }
}

static inline double qd(double t, int idx) {
    if (idx == 0) return (M_PI / 4.0) * sin(t);
    else          return (M_PI / 8.0) * cos(t);
}
static inline double dqd(double t, int idx) {
    if (idx == 0) return (M_PI / 4.0) * cos(t);
    else          return -(M_PI / 8.0) * sin(t);
}
static inline double ddqd(double t, int idx) {
    if (idx == 0) return -(M_PI / 4.0) * sin(t);
    else          return -(M_PI / 8.0) * cos(t);
}

static inline Matrix4d get_Q_matrix() {
    Matrix4d Q = Matrix4d::Zero();
    Q(0, 1) = -1;
    Q(1, 0) = 1;
    return Q;
}

static inline Matrix4d get_U(Matrix4d T[2], Matrix4d Q, int j, int i) {
    if (j > i) return Matrix4d::Zero();
    Matrix4d T_prev = (j > 0) ? T[j - 1] : Matrix4d::Identity();
    return T_prev * Q * (T_prev.inverse() * T[i]);
}

static inline Matrix4d get_U_dot(Matrix4d T[2], Matrix4d Q, int j, int k, int i) {
    if (j > i || k > i) return Matrix4d::Zero();
    int m1 = (j <= k) ? j : k;
    int m2 = (j <= k) ? k : j;

    Matrix4d T_0_m1_prev = (m1 > 0) ? T[m1 - 1] : Matrix4d::Identity();
    Matrix4d T_m1_prev_m2_prev =
        T_0_m1_prev.inverse() * ((m2 > 0) ? T[m2 - 1] : Matrix4d::Identity());
    Matrix4d T_m2_prev_i =
        ((m2 > 0) ? T[m2 - 1] : Matrix4d::Identity()).inverse() * T[i];

    return T_0_m1_prev * Q * T_m1_prev_m2_prev * Q * T_m2_prev_i;
}

static inline void compute_dynamics(
    const TwoDOFParams *p, int mode, double t, 
    double q[2], double dq[2], 
    double D_out[2][2], double h_out[2], double C_out[2], 
    double tau_out[2], double ddq_out[2], 
    double q_ref[2], double dq_ref[2], double ddq_ref[2])
{

    for (int i = 0; i < 2; ++i) {
        q_ref[i]   = qd(t, i);
        dq_ref[i]  = dqd(t, i);
        ddq_ref[i] = ddqd(t, i);
    }

    Matrix4d Q = get_Q_matrix();
    
    Matrix4d Js[2];
    Js[0] = Matrix4d::Zero(); Js[0](0, 3) = p->m1 * p->r1; Js[0](3, 0) = p->m1 * p->r1; Js[0](3, 3) = p->m1;
    Js[1] = Matrix4d::Zero(); Js[1](0, 3) = p->m2 * p->r2; Js[1](3, 0) = p->m2 * p->r2; Js[1](3, 3) = p->m2;

    Matrix4d T[2];
    T[0] << cos(q[0]), -sin(q[0]), 0, p->L1*cos(q[0]),
            sin(q[0]),  cos(q[0]), 0, p->L1*sin(q[0]),
            0, 0, 1, 0,
            0, 0, 0, 1;
    Matrix4d T12;
    T12 << cos(q[1]), -sin(q[1]), 0, p->L2*cos(q[1]),
           sin(q[1]),  cos(q[1]), 0, p->L2*sin(q[1]),
           0, 0, 1, 0,
           0, 0, 0, 1;
    T[1] = T[0] * T12;

    Matrix2d D = Matrix2d::Zero();
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j <= i; j++) {
            for (int k = 0; k <= i; k++) {
                D(j, k) += (get_U(T, Q, k, i) * Js[i] * get_U(T, Q, j, i).transpose()).trace();
            }
        }
    }

    Vector2d h = Vector2d::Zero();
    for (int i = 0; i < 2; i++) {
        for (int k = 0; k <= i; k++) {
            for (int m = 0; m <= i; m++) {
                Matrix4d U_ikm = get_U_dot(T, Q, k, m, i);
                for (int j = 0; j <= i; j++) {
                    h(j) += (U_ikm * Js[i] * get_U(T, Q, j, i).transpose()).trace() * dq[k] * dq[m];
                }
            }
        }
    }

    Vector2d G = Vector2d::Zero();
    Vector4d g_vec(0, -p->g, 0, 0);
    for (int i = 0; i < 2; i++) {
        Vector4d r_i_vec( (i==0 ? p->r1 : p->r2), 0, 0, 1);
        double m_i = (i==0 ? p->m1 : p->m2);
        for (int j = 0; j <= i; j++) {
            Matrix<double, 1, 1> res = g_vec.transpose() * get_U(T, Q, j, i) * r_i_vec;
            G(j) -= m_i * res(0, 0);

        }
    }

    D_out[0][0] = D(0,0); D_out[0][1] = D(0,1);
    D_out[1][0] = D(1,0); D_out[1][1] = D(1,1);
    h_out[0] = h(0); h_out[1] = h(1);
    C_out[0] = G(0); C_out[1] = G(1);

    Vector2d tau = Vector2d::Zero();
    if (mode == MODE_FREEFALL) {
        tau << 0.0, 0.0;
    } else if (mode == MODE_GRAVITY_COMP) {
        tau = G;
    } else { 
        Vector2d q_ref_vec(q_ref[0], q_ref[1]);
        Vector2d q_vec(q[0], q[1]);
        Vector2d dq_ref_vec(dq_ref[0], dq_ref[1]);
        Vector2d dq_vec(dq[0], dq[1]);
        Vector2d ddq_ref_vec(ddq_ref[0], ddq_ref[1]);
        
        Vector2d u = ddq_ref_vec + Vector2d(p->Kp[0], p->Kp[1]).cwiseProduct(q_ref_vec - q_vec) 
                                 + Vector2d(p->Kd[0], p->Kd[1]).cwiseProduct(dq_ref_vec - dq_vec);
        tau = D * u + h + G;
    }

    tau_out[0] = tau(0); tau_out[1] = tau(1);

    Vector2d dq_v(dq[0], dq[1]);
    Vector2d B_v(p->B[0], p->B[1]);
    Vector2d ddq_v = D.inverse() * (tau - h - G - B_v.cwiseProduct(dq_v));
    
    ddq_out[0] = ddq_v(0); ddq_out[1] = ddq_v(1);
}

static inline void dynamics_step(
    const TwoDOFParams *p, int mode, double t, double dt, 
    double q[2], double dq[2], 
    double D[2][2], double h[2], double C[2], 
    double tau[2], double ddq[2], 
    double q_ref[2], double dq_ref[2], double ddq_ref[2])
{
    compute_dynamics(p, mode, t, q, dq, D, h, C, tau, ddq, q_ref, dq_ref, ddq_ref);
    for (int i = 0; i < 2; ++i) {
        dq[i] += ddq[i] * dt;
        q[i]  += dq[i] * dt;
    }
}

static inline void synthetic_workload(int inner_loops, volatile double *sink) {
    double x = 0.123456789;
    for (int i = 0; i < inner_loops; ++i) {
        x = sin(x) * cos(x) + sqrt(x * x + 1.0);
        x = x / (1.0000001 + fabs(sin(x)));
    }
    *sink = x;
}

static inline int64_t ts_to_ns(const struct timespec *ts) {
    return (int64_t)ts->tv_sec * 1000000000LL + ts->tv_nsec;
}

static inline void timespec_add_ns(struct timespec *r, const struct timespec *t, long long ns) {
    long long sec = ns / 1000000000LL;
    long long rem = ns % 1000000000LL;
    r->tv_sec  = t->tv_sec + sec;
    r->tv_nsec = t->tv_nsec + rem;
    if (r->tv_nsec >= 1000000000L) { r->tv_sec++; r->tv_nsec -= 1000000000L; }
}

#endif

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
    MODE_PID_TRACKING = 2,
    MODE_CARTESIAN_TRACKING = 3
};

typedef struct {
    double m1, m2;          
    double L1, L2;          
    double r1, r2;          
    double g;               
    double B[2];            
    double Kp[2];           
    double Kd[2];           
    double init_q_rad[2];   
} TwoDOFParams;

static inline int streq(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}


static inline int parse_mode(const char *s) {
    if (streq(s, "freefall") || streq(s, "no_torque")) return MODE_FREEFALL;
    if (streq(s, "gravity") || streq(s, "gravity_comp")) return MODE_GRAVITY_COMP;
    if (streq(s, "pid") || streq(s, "tracking")) return MODE_PID_TRACKING;
    if (streq(s, "cart") || streq(s, "cartesian")) return MODE_CARTESIAN_TRACKING;
    return MODE_PID_TRACKING;
}


static inline const char* mode_to_string(int mode) {
    switch (mode) {
        case MODE_FREEFALL: return "freefall";
        case MODE_GRAVITY_COMP: return "gravity_comp";
        case MODE_PID_TRACKING: return "pid";
        case MODE_CARTESIAN_TRACKING: return "cartesian";
        default: return "unknown";
    }
}


// Joint reference trajectory
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

// Cartesian reference trajectory
static inline void cartesian_ref(double t,
                                 double &xr, double &yr,
                                 double &dxr, double &dyr,
                                 double &ddxr, double &ddyr)
{
    const double xc = 1.2;   // Circle Center X
    const double yc = 0.3;   // Circle Center y
    const double r  = 0.25;  // radius
    const double w  = 0.8;   // angular velocity

    xr   = xc + r * cos(w * t);
    yr   = yc + r * sin(w * t);

    dxr  = -r * w * sin(w * t);
    dyr  =  r * w * cos(w * t);

    ddxr = -r * w * w * cos(w * t);
    ddyr = -r * w * w * sin(w * t);
}

// Forward Kinematics
static inline void forward_kinematics_2d(const TwoDOFParams *p,
                                         const double q[2],
                                         double x[2])
{
    const double q1 = q[0];
    const double q2 = q[1];

    x[0] = p->L1 * cos(q1) + p->L2 * cos(q1 + q2);
    x[1] = p->L1 * sin(q1) + p->L2 * sin(q1 + q2);
}

// Jacobian
static inline void jacobian_2d(const TwoDOFParams *p,
                               const double q[2],
                               double J[2][2])
{
    const double q1 = q[0];
    const double q2 = q[1];

    const double s1  = sin(q1);
    const double c1  = cos(q1);
    const double s12 = sin(q1 + q2);
    const double c12 = cos(q1 + q2);

    J[0][0] = -p->L1 * s1 - p->L2 * s12;
    J[0][1] = -p->L2 * s12;
    J[1][0] =  p->L1 * c1 + p->L2 * c12;
    J[1][1] =  p->L2 * c12;
}


// Jdot * qdot
static inline void jacobian_dot_times_dq(const TwoDOFParams *p,
                                         const double q[2],
                                         const double dq[2],
                                         double out[2])
{
    const double q1  = q[0];
    const double q2  = q[1];
    const double dq1 = dq[0];
    const double dq2 = dq[1];

    const double c1  = cos(q1);
    const double s1  = sin(q1);
    const double c12 = cos(q1 + q2);
    const double s12 = sin(q1 + q2);

    const double j11_dot = -p->L1 * c1 * dq1 - p->L2 * c12 * (dq1 + dq2);
    const double j12_dot = -p->L2 * c12 * (dq1 + dq2);
    const double j21_dot = -p->L1 * s1 * dq1 - p->L2 * s12 * (dq1 + dq2);
    const double j22_dot = -p->L2 * s12 * (dq1 + dq2);

    out[0] = j11_dot * dq1 + j12_dot * dq2;
    out[1] = j21_dot * dq1 + j22_dot * dq2;
}

static inline Matrix4d get_Q_matrix() {
    Matrix4d Q = Matrix4d::Zero();
    Q(0, 1) = -1;
    Q(1, 0) = 1;
    return Q;
}

static inline Matrix4d get_U(Matrix4d T[2], Matrix4d Q, int j, int i) {
    if (j > i) return Matrix4d::Zero();
    Matrix4d T_prev = (j > 0) ? T[j-1] : Matrix4d::Identity();
    return T_prev * Q * (T_prev.inverse() * T[i]);
}

static inline Matrix4d get_U_dot(Matrix4d T[2], Matrix4d Q, int j, int k, int i) {
    if (j > i || k > i) return Matrix4d::Zero();
    int m1 = (j <= k) ? j : k;
    int m2 = (j <= k) ? k : j;
    Matrix4d T_0_m1_prev = (m1 > 0) ? T[m1-1] : Matrix4d::Identity();
    Matrix4d T_m1_prev_m2_prev = T_0_m1_prev.inverse() * ((m2 > 0) ? T[m2-1] : Matrix4d::Identity());
    Matrix4d T_m2_prev_i = ((m2 > 0) ? T[m2-1] : Matrix4d::Identity()).inverse() * T[i];
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
            0,          0,         1, 0,
            0,          0,         0, 1;

    Matrix4d T12;
    T12 << cos(q[1]), -sin(q[1]), 0, p->L2*cos(q[1]),
           sin(q[1]),  cos(q[1]), 0, p->L2*sin(q[1]),
           0,          0,         1, 0,
           0,          0,         0, 1;

    T[1] = T[0] * T12;

    // D(q) 
    Matrix2d D = Matrix2d::Zero();
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j <= i; j++) {
            for (int k = 0; k <= i; k++) {
                D(j, k) += (get_U(T, Q, k, i) * Js[i] * get_U(T, Q, j, i).transpose()).trace();
            }
        }
    }

    //  h(q,dq) 
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

    //  G(q) 
    Vector2d G = Vector2d::Zero();
    Vector4d g_vec(0, -p->g, 0, 0);
    for (int i = 0; i < 2; i++) {
        Vector4d r_i_vec((i == 0 ? p->r1 : p->r2), 0, 0, 1);
        double m_i = (i == 0 ? p->m1 : p->m2);
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
    }
    else if (mode == MODE_GRAVITY_COMP) {
        tau = G;
    }
    else if (mode == MODE_PID_TRACKING) {

        Vector2d q_ref_vec(q_ref[0], q_ref[1]);
        Vector2d q_vec(q[0], q[1]);
        Vector2d dq_ref_vec(dq_ref[0], dq_ref[1]);
        Vector2d dq_vec(dq[0], dq[1]);
        Vector2d ddq_ref_vec(ddq_ref[0], ddq_ref[1]);

        // computed torque 형태의 joint PID
        Vector2d u = ddq_ref_vec
                   + Vector2d(p->Kp[0], p->Kp[1]).cwiseProduct(q_ref_vec - q_vec)
                   + Vector2d(p->Kd[0], p->Kd[1]).cwiseProduct(dq_ref_vec - dq_vec);

        tau = D * u + h + G;
    }
    else if (mode == MODE_CARTESIAN_TRACKING) {
        // Cartesian space tracking

        double x[2];
        double xr, yr, dxr, dyr, ddxr, ddyr;
        double J_raw[2][2];
        double jdotdq_raw[2];

        //  end-effector 
        forward_kinematics_2d(p, q, x);

        //  Jacobian
        jacobian_2d(p, q, J_raw);

        // Jdot * qdot
        jacobian_dot_times_dq(p, q, dq, jdotdq_raw);

        // Cartesian trajectory
        cartesian_ref(t, xr, yr, dxr, dyr, ddxr, ddyr);

        Matrix2d J;
        J << J_raw[0][0], J_raw[0][1],
             J_raw[1][0], J_raw[1][1];

        Vector2d dq_vec(dq[0], dq[1]);
        Vector2d x_vec(x[0], x[1]);
        Vector2d x_ref_vec(xr, yr);
        Vector2d dx_ref_vec(dxr, dyr);
        Vector2d ddx_ref_vec(ddxr, ddyr);

        // Current EE velocity = J * dq
        Vector2d dx_vec = J * dq_vec;

        // Jdot * dq
        Vector2d jdotdq_vec(jdotdq_raw[0], jdotdq_raw[1]);

        // Gain
        const double Kpx = 80.0;
        const double Kpy = 80.0;
        const double Kdx = 25.0;
        const double Kdy = 25.0;

        // Cartesian acceleration command
        // a_cmd = ddx_ref + Kp(x_ref - x) + Kd(dx_ref - dx)
        Vector2d a_cmd;

        Vector2d ddq_cmd = Vector2d::Zero();

        double detJ = J.determinant();

        a_cmd << ddx_ref_vec(0) + Kpx * (x_ref_vec(0) - x_vec(0)) + Kdx * (dx_ref_vec(0) - dx_vec(0)),
                 ddx_ref_vec(1) + Kpy * (x_ref_vec(1) - x_vec(1)) + Kdy * (dx_ref_vec(1) - dx_vec(1));

        if (fabs(detJ) > 1e-8) {
            ddq_cmd = J.inverse() * (a_cmd - jdotdq_vec);
        }

        tau = D * ddq_cmd + h + G;

        q_ref[0] = q[0];
        q_ref[1] = q[1];
        dq_ref[0] = 0.0;
        dq_ref[1] = 0.0;
        ddq_ref[0] = ddq_cmd(0);
        ddq_ref[1] = ddq_cmd(1);
    }

    tau_out[0] = tau(0);
    tau_out[1] = tau(1);


    Vector2d dq_v(dq[0], dq[1]);
    Vector2d B_v(p->B[0], p->B[1]);

    Vector2d ddq_v = D.inverse() * (tau - h - G - B_v.cwiseProduct(dq_v));

    ddq_out[0] = ddq_v(0);
    ddq_out[1] = ddq_v(1);
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
    if (r->tv_nsec >= 1000000000L) {
        r->tv_sec++;
        r->tv_nsec -= 1000000000L;
    }
}

#endif

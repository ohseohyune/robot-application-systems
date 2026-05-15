#ifndef LOAD_2DOF_COMMON_H
#define LOAD_2DOF_COMMON_H

#include <math.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

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

static inline void compute_dynamics(
    const TwoDOFParams *p, int mode, double t, 
    double q[2], double dq[2], 
    double D[2][2], double h[2], double C[2], 
    double tau[2], double ddq[2], 
    double q_ref[2], double dq_ref[2], double ddq_ref[2])
{
    for (int i = 0; i < 2; ++i) {
        q_ref[i]   = qd(t, i);
        dq_ref[i]  = dqd(t, i);
        ddq_ref[i] = ddqd(t, i);
    }

    D[0][0] = p->m1*p->r1*p->r1 + p->m2*(p->L1*p->L1 + p->r2*p->r2 + 2*p->L1*p->r2*cos(q[1]));
    D[0][1] = p->m2*(p->r2*p->r2 + p->L1*p->r2*cos(q[1]));
    D[1][0] = D[0][1];
    D[1][1] = p->m2*p->r2*p->r2;

    double h_term = p->m2 * p->L1 * p->r2 * sin(q[1]);
    h[0] = -h_term * (2.0*dq[0]*dq[1] + dq[1]*dq[1]);
    h[1] =  h_term * (dq[0]*dq[0]);

    C[0] = (p->m1*p->r1 + p->m2*p->L1) * p->g * cos(q[0]) + p->m2*p->r2*p->g * cos(q[0]+q[1]);
    C[1] = p->m2*p->r2*p->g * cos(q[0]+q[1]);

    for (int i = 0; i < 2; ++i) {
        if (mode == MODE_FREEFALL) {
            tau[i] = 0.0;
        } else if (mode == MODE_GRAVITY_COMP) {
            tau[i] = C[i];
        } else { 
            double u = ddq_ref[i] + p->Kp[i]*(q_ref[i] - q[i]) + p->Kd[i]*(dq_ref[i] - dq[i]);
            tau[i] = D[i][0]*u + D[i][1]*u + h[i] + C[i]; 
        }
    }

    double det = D[0][0]*D[1][1] - D[0][1]*D[1][0];
    double invD[2][2] = {
        { D[1][1]/det, -D[0][1]/det },
        {-D[1][0]/det,  D[0][0]/det }
    };
    
    double net_force[2];
    net_force[0] = tau[0] - h[0] - C[0] - p->B[0]*dq[0];
    net_force[1] = tau[1] - h[1] - C[1] - p->B[1]*dq[1];

    ddq[0] = invD[0][0]*net_force[0] + invD[0][1]*net_force[1];
    ddq[1] = invD[1][0]*net_force[0] + invD[1][1]*net_force[1];
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
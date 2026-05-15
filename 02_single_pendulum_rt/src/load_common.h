#ifndef LOAD_COMMON_H
#define LOAD_COMMON_H

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
    double m;
    double L;
    double lc;
    double I;
    double g;
    double B;
    double Kp;
    double Kd;
    double init_q_rad;
} PendulumParams;

static inline int streq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

static inline int parse_mode(const char *s)
{
    if (streq(s, "freefall") || streq(s, "pendulum") || streq(s, "no_torque"))
        return MODE_FREEFALL;
    if (streq(s, "gravity") || streq(s, "gravity_hold") || streq(s, "gravity_comp"))
        return MODE_GRAVITY_COMP;
    if (streq(s, "pid") || streq(s, "trajectory") || streq(s, "tracking"))
        return MODE_PID_TRACKING;
    return MODE_PID_TRACKING;
}

static inline const char* mode_to_string(int mode)
{
    switch (mode) {
    case MODE_FREEFALL: return "freefall";
    case MODE_GRAVITY_COMP: return "gravity";
    case MODE_PID_TRACKING: return "pid";
    default: return "unknown";
    }
}

static inline const char* mode_to_pretty_string(int mode)
{
    switch (mode) {
    case MODE_FREEFALL: return "Free Fall";
    case MODE_GRAVITY_COMP: return "Gravity Compensation";
    case MODE_PID_TRACKING: return "PID Tracking";
    default: return "Unknown";
    }
}

static inline double D_term(const PendulumParams *p, double q)
{
    (void)q;
    return p->I + p->m * p->L * p->L;
}

static inline double h_term(const PendulumParams *p, double q, double dq)
{
    (void)p;
    (void)q;
    (void)dq;
    return 0.0;
}

static inline double C_term(const PendulumParams *p, double q)
{
    return p->m * p->g * p->L * cos(q);
}

static inline double qd(double t)
{
    return (M_PI / 4.0) * sin(t);
}

static inline double dqd(double t)
{
    return (M_PI / 4.0) * cos(t);
}

static inline double ddqd(double t)
{
    return -(M_PI / 4.0) * sin(t);
}

static inline void compute_dynamics(
    const PendulumParams *p,
    int mode,
    double t,
    double q,
    double dq,
    double *D,
    double *h,
    double *C,
    double *tau,
    double *ddq,
    double *q_ref,
    double *dq_ref,
    double *ddq_ref)
{
    *D = D_term(p, q);
    *h = h_term(p, q, dq);
    *C = C_term(p, q);

    *q_ref = qd(t);
    *dq_ref = dqd(t);
    *ddq_ref = ddqd(t);

    switch (mode) {
    case MODE_FREEFALL:
        *tau = 0.0;
        break;
    case MODE_GRAVITY_COMP:
        *tau = *h + *C;
        break;
    case MODE_PID_TRACKING: {
        const double err_q = *q_ref - q;
        const double err_dq = *dq_ref - dq;
        *tau = (*D * *ddq_ref) + *h + *C + (p->B * dq)
             + (p->Kp * err_q) + (p->Kd * err_dq);
        break;
    }
    default:
        *tau = 0.0;
        break;
    }

    *ddq = (*tau - *h - *C - (p->B * dq)) / *D;
}

static inline void dynamics_step(
    const PendulumParams *p,
    int mode,
    double t,
    double dt,
    double *q,
    double *dq,
    double *D,
    double *h,
    double *C,
    double *tau,
    double *ddq,
    double *q_ref,
    double *dq_ref,
    double *ddq_ref)
{
    compute_dynamics(p, mode, t, *q, *dq, D, h, C, tau, ddq,
                     q_ref, dq_ref, ddq_ref);

    *dq += (*ddq) * dt;
    *q  += (*dq) * dt;
}

static inline void synthetic_workload(int inner_loops, volatile double *sink)
{
    double x = 0.123456789;
    for (int i = 0; i < inner_loops; ++i) {
        x = sin(x) * cos(x) + sqrt(x * x + 1.0);
        x = x / (1.0000001 + fabs(sin(x)));
    }
    *sink = x;
}

static inline int64_t ts_to_ns(const struct timespec *ts)
{
    return (int64_t)ts->tv_sec * 1000000000LL + ts->tv_nsec;
}

static inline void ns_to_ts(int64_t ns, struct timespec *ts)
{
    ts->tv_sec = ns / 1000000000LL;
    ts->tv_nsec = ns % 1000000000LL;
    if (ts->tv_nsec < 0) {
        ts->tv_sec--;
        ts->tv_nsec += 1000000000LL;
    }
}

static inline void timespec_add_ns(struct timespec *r,
                                   const struct timespec *t,
                                   long long ns)
{
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

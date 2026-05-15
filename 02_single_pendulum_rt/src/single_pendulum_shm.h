#ifndef SINGLE_PENDULUM_SHM_H
#define SINGLE_PENDULUM_SHM_H

#include <stdint.h>

#define RT_SINGLE_SHM  "/evl_rt_single_pendulum"
#define NRT_SINGLE_SHM "/linux_nrt_single_pendulum"

typedef struct {
    int initialized;
    int finished;

    int mode;
    int period_us;
    int workload_loops;

    double t;
    double q;
    double dq;
    double ddq;

    double D;
    double h;
    double C;
    double tau;

    double q_ref;
    double dq_ref;
    double ddq_ref;

    double err_q;
    double err_dq;
    double abs_err_q;
    double abs_err_dq;
    double avg_abs_err_q;
    double avg_abs_err_dq;
    double rms_err_q;
    double rms_err_dq;
    double max_abs_err_q;
    double max_abs_err_dq;

    double jitter_us;
    double min_jitter_us;
    double max_jitter_us;
    double avg_jitter_us;
    double avg_abs_jitter_us;

    double exec_time_us;
    double min_exec_time_us;
    double max_exec_time_us;
    double avg_exec_time_us;

    long long loop_count;
    long long miss_count;
    long long overrun_count;
} SharedState;

#endif

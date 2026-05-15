#ifndef TWO_DOF_SHM_H
#define TWO_DOF_SHM_H

#include <stdint.h>

#define RT_2DOF_SHM  "/evl_rt_2dof_arm"
#define NRT_2DOF_SHM "/linux_nrt_2dof_arm"

typedef struct {
    int initialized;
    int finished;

    int mode;
    int period_us;
    int workload_loops;

    double t;
    
    double q[2];
    double dq[2];
    double ddq[2];

    double D[2][2];
    double h[2];
    double C[2];
    double tau[2];

    double q_ref[2];
    double dq_ref[2];
    double ddq_ref[2];

    double err_q[2];
    double err_dq[2];
    double abs_err_q[2];
    double abs_err_dq[2];
    double avg_abs_err_q[2];
    double avg_abs_err_dq[2];
    double rms_err_q[2];
    double rms_err_dq[2];
    double max_abs_err_q[2];
    double max_abs_err_dq[2];

    // Cartesian state
    double x[2];        
    double dx[2];      
    double ddx[2];      

    double x_ref[2];  
    double dx_ref[2];    
    double ddx_ref[2];   

    double err_x[2];
    double err_dx[2];
    double rms_err_x[2];
    double max_abs_err_x[2];

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
} SharedState2DOF;

#endif
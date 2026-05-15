#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sched.h>
#include <time.h>
#include <float.h>
#include <math.h>

#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <evl/proxy.h>
#include <evl/sched.h>
#include "load_2dof_common2.h"
#include "two_dof_shm.h"

static int pin_cpu(int cpu) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(0, sizeof(set), &set);
}

int main(int argc, char *argv[]) {
    umask(0);

    const char *mode_str = "pid";
    int period_us = 1000;
    int duration_s = 10;
    int workload_loops = 5000;
    int cpu = 0;

    if (argc > 1) mode_str = argv[1];
    if (argc > 2) period_us = atoi(argv[2]);
    if (argc > 3) duration_s = atoi(argv[3]);
    if (argc > 4) workload_loops = atoi(argv[4]);
    if (argc > 5) cpu = atoi(argv[5]);

    int mode = parse_mode(mode_str);

    if (mlockall(MCL_CURRENT | MCL_FUTURE)) perror("mlockall");
    if (pin_cpu(cpu)) perror("sched_setaffinity");

    shm_unlink(RT_2DOF_SHM);
    int shm_fd = shm_open(RT_2DOF_SHM, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) { perror("shm_open"); return 1; }
    if (ftruncate(shm_fd, sizeof(SharedState2DOF)) < 0) { perror("ftruncate"); return 1; }

    SharedState2DOF *shm = (SharedState2DOF*)mmap(NULL, sizeof(SharedState2DOF),
                                          PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) { perror("mmap"); return 1; }

    memset(shm, 0, sizeof(SharedState2DOF));
    shm->initialized = 1;
    shm->mode = mode;
    shm->period_us = period_us;
    shm->workload_loops = workload_loops;

    int efd = evl_attach_self("rt-2dof:%d", getpid());
    if (efd < 0) { perror("evl_attach_self"); return 1; }
    
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;

    if (evl_set_schedattr(efd, &attrs)) { perror("evl_set_schedattr"); return 1; }

    int tfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
    if (tfd < 0) { perror("evl_new_timer"); return 1; }

    const int64_t period_ns = (int64_t)period_us * 1000LL;
    const int64_t total_loops = ((int64_t)duration_s * 1000000LL) / period_us;

    struct timespec now, first;
    struct itimerspec its;
    memset(&its, 0, sizeof(its));

    clock_gettime(CLOCK_MONOTONIC, &now);
    timespec_add_ns(&first, &now, period_ns);

    its.it_value = first;
    its.it_interval.tv_sec = period_ns / 1000000000LL;
    its.it_interval.tv_nsec = period_ns % 1000000000LL;

    if (evl_set_timer(tfd, &its, NULL) < 0) { perror("evl_set_timer"); return 1; }

    TwoDOFParams p = {
        1.0, 1.0,           // m1, m2
        1.0, 1.0,           // L1, L2
        0.5, 0.5,           // r1, r2
        9.81,               // g
        {0.1, 0.1},         // B1, B2
        {100.0, 50.0},      // Kp1, Kp2
        {20.0, 10.0},       // Kd1, Kd2
        {M_PI/4.0, 0.0}     // init q
    };

    double q[2] = {p.init_q_rad[0], p.init_q_rad[1]};
    double dq[2] = {0.0, 0.0};
    double D[2][2], h[2], C[2], tau[2], ddq[2], q_ref[2], dq_ref[2], ddq_ref[2];
    
    double sim_t = 0.0;
    double dt = period_ns / 1e9;
    volatile double sink = 0.0;
    int64_t expected_ns = ts_to_ns(&first);

    // 성능 지표 스칼라
    double min_jitter_us = DBL_MAX, max_jitter_us = -DBL_MAX, sum_jitter_us = 0.0, sum_abs_jitter_us = 0.0;
    double min_exec_us = DBL_MAX, max_exec_us = -DBL_MAX, sum_exec_us = 0.0;

    // 성능 지표 배열
    double sum_abs_err_q[2] = {0}, sum_abs_err_dq[2] = {0};
    double sum_sq_err_q[2] = {0}, sum_sq_err_dq[2] = {0};
    double max_abs_err_q[2] = {0}, max_abs_err_dq[2] = {0};

    for (int64_t i = 0; i < total_loops; ++i) {
        uint64_t ticks = 0;
        if (oob_read(tfd, &ticks, sizeof(ticks)) < 0) { perror("oob_read"); break; }

        struct timespec actual_start, actual_end;
        clock_gettime(CLOCK_MONOTONIC, &actual_start);
        int64_t actual_start_ns = ts_to_ns(&actual_start);

        int64_t jitter_ns = actual_start_ns - expected_ns;
        double jitter_us = (double)jitter_ns / 1000.0;

        if (jitter_us < min_jitter_us) min_jitter_us = jitter_us;
        if (jitter_us > max_jitter_us) max_jitter_us = jitter_us;
        sum_jitter_us += jitter_us;
        sum_abs_jitter_us += fabs(jitter_us);

        if (jitter_ns > period_ns) shm->miss_count++;
        if (ticks > 1) shm->overrun_count += (long long)(ticks - 1);

        dynamics_step(&p, mode, sim_t, dt, q, dq, D, h, C, tau, ddq, q_ref, dq_ref, ddq_ref);
        
        synthetic_workload(workload_loops, &sink);
        sim_t += dt;

        for (int j = 0; j < 2; ++j) {
            double err_q = q_ref[j] - q[j];
            double err_dq = dq_ref[j] - dq[j];
            double abs_err_q = fabs(err_q);
            double abs_err_dq = fabs(err_dq);

            sum_abs_err_q[j] += abs_err_q;
            sum_abs_err_dq[j] += abs_err_dq;
            sum_sq_err_q[j] += err_q * err_q;
            sum_sq_err_dq[j] += err_dq * err_dq;
            if (abs_err_q > max_abs_err_q[j]) max_abs_err_q[j] = abs_err_q;
            if (abs_err_dq > max_abs_err_dq[j]) max_abs_err_dq[j] = abs_err_dq;

            shm->err_q[j] = err_q; shm->err_dq[j] = err_dq;
            shm->abs_err_q[j] = abs_err_q; shm->abs_err_dq[j] = abs_err_dq;
            shm->avg_abs_err_q[j] = sum_abs_err_q[j] / (double)(i + 1);
            shm->rms_err_q[j] = sqrt(sum_sq_err_q[j] / (double)(i + 1));
            shm->max_abs_err_q[j] = max_abs_err_q[j];
            
            shm->q[j] = q[j]; shm->dq[j] = dq[j]; shm->ddq[j] = ddq[j];
            shm->tau[j] = tau[j]; shm->h[j] = h[j]; shm->C[j] = C[j];
        }

        shm->D[0][0] = D[0][0]; shm->D[0][1] = D[0][1];
        shm->D[1][0] = D[1][0]; shm->D[1][1] = D[1][1];

        clock_gettime(CLOCK_MONOTONIC, &actual_end);
        int64_t exec_ns = ts_to_ns(&actual_end) - actual_start_ns;
        double exec_us = (double)exec_ns / 1000.0;

        if (exec_us < min_exec_us) min_exec_us = exec_us;
        if (exec_us > max_exec_us) max_exec_us = exec_us;
        sum_exec_us += exec_us;

        shm->t = sim_t;
        shm->jitter_us = jitter_us;
        shm->min_jitter_us = min_jitter_us;
        shm->max_jitter_us = max_jitter_us;
        shm->avg_jitter_us = sum_jitter_us / (double)(i + 1);
        
        shm->exec_time_us = exec_us;
        shm->min_exec_time_us = min_exec_us;
        shm->max_exec_time_us = max_exec_us;
        shm->avg_exec_time_us = sum_exec_us / (double)(i + 1);
        
        shm->loop_count = i + 1;
        expected_ns += (int64_t)ticks * period_ns;
    }

    shm->finished = 1;

    printf("\n=== RT 2-DOF Performance Summary ===\n");
    printf("mode              : %s\n", mode_to_string(mode));
    printf("period            : %d us\n", period_us);
    printf("loops             : %lld\n", shm->loop_count);
    printf("avg jitter        : %.3f us\n", shm->avg_jitter_us);
    printf("max exec time     : %.3f us\n", shm->max_exec_time_us);
    printf("deadline miss     : %lld\n", shm->miss_count);
    printf("--- Joint 1 ---\n");
    printf("final q1          : %.4f rad\n", q[0]);
    printf("rms err q1        : %.4f rad\n", shm->rms_err_q[0]);
    printf("--- Joint 2 ---\n");
    printf("final q2          : %.4f rad\n", q[1]);
    printf("rms err q2        : %.4f rad\n", shm->rms_err_q[1]);

    close(tfd);
    close(efd);
    munmap(shm, sizeof(SharedState2DOF));
    close(shm_fd);
    return 0;
}
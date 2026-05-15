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
#include "load_common.h"
#include "single_pendulum_shm.h"

static int pin_cpu(int cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(0, sizeof(set), &set);
}

int main(int argc, char *argv[])
{
    umask(0);

    const char *mode_str = "pid";
    int period_us = 1000;
    int duration_s = 10;
    int workload_loops = 5000;
    int cpu = 0;
    double init_deg = 60.0;
    double kp = 20.0;
    double kd = 5.0;

    if (argc > 1) mode_str = argv[1];
    if (argc > 2) period_us = atoi(argv[2]);
    if (argc > 3) duration_s = atoi(argv[3]);
    if (argc > 4) workload_loops = atoi(argv[4]);
    if (argc > 5) cpu = atoi(argv[5]);
    if (argc > 6) init_deg = atof(argv[6]);
    if (argc > 7) kp = atof(argv[7]);
    if (argc > 8) kd = atof(argv[8]);

    int mode = parse_mode(mode_str);

    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        perror("mlockall");

    if (pin_cpu(cpu))
        perror("sched_setaffinity");

    shm_unlink(RT_SINGLE_SHM);

    int shm_fd = shm_open(RT_SINGLE_SHM, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("shm_open");
        return 1;
    }

    if (ftruncate(shm_fd, sizeof(SharedState)) < 0) {
        perror("ftruncate");
        return 1;
    }

    SharedState *shm = (SharedState*)mmap(NULL, sizeof(SharedState),
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    memset(shm, 0, sizeof(SharedState));
    shm->initialized = 1;
    shm->mode = mode;
    shm->period_us = period_us;
    shm->workload_loops = workload_loops;

    int efd = evl_attach_self("rt-single:%d", getpid());
    if (efd < 0) {
        errno = -efd;
        perror("evl_attach_self");
        return 1;
    }
    
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;   // 1~99, 보통 70~90 많이 사용

    int ret = evl_set_schedattr(efd, &attrs);
    if (ret) {
        errno = -ret;
        perror("evl_set_schedattr");
        return 1;
    }

    int tfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
    if (tfd < 0) {
        errno = -tfd;
        perror("evl_new_timer");
        return 1;
    }

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

    if (evl_set_timer(tfd, &its, NULL) < 0) {
        perror("evl_set_timer");
        return 1;
    }

    PendulumParams p = {1.0, 1.0, 0.5, 1.0 / 12.0, 9.81, 0.5, kp, kd, init_deg * M_PI / 180.0};

    double q = p.init_q_rad;
    double dq = 0.0;
    double sim_t = 0.0;
    double dt = period_ns / 1e9;
    volatile double sink = 0.0;
    int64_t expected_ns = ts_to_ns(&first);

    double min_jitter_us = DBL_MAX;
    double max_jitter_us = -DBL_MAX;
    double sum_jitter_us = 0.0;
    double sum_abs_jitter_us = 0.0;

    double min_exec_us = DBL_MAX;
    double max_exec_us = -DBL_MAX;
    double sum_exec_us = 0.0;

    double sum_abs_err_q = 0.0;
    double sum_abs_err_dq = 0.0;
    double sum_sq_err_q = 0.0;
    double sum_sq_err_dq = 0.0;
    double max_abs_err_q = 0.0;
    double max_abs_err_dq = 0.0;

    for (int64_t i = 0; i < total_loops; ++i) {
        uint64_t ticks = 0;
        ssize_t n = oob_read(tfd, &ticks, sizeof(ticks));
        if (n < 0) {
            perror("oob_read");
            break;
        }

        struct timespec actual_start, actual_end;
        clock_gettime(CLOCK_MONOTONIC, &actual_start);
        int64_t actual_start_ns = ts_to_ns(&actual_start);

        int64_t jitter_ns = actual_start_ns - expected_ns;
        double jitter_us = (double)jitter_ns / 1000.0;

        if (jitter_us < min_jitter_us) min_jitter_us = jitter_us;
        if (jitter_us > max_jitter_us) max_jitter_us = jitter_us;
        sum_jitter_us += jitter_us;
        sum_abs_jitter_us += fabs(jitter_us);

        if (jitter_ns > period_ns)
            shm->miss_count++;

        if (ticks > 1)
            shm->overrun_count += (long long)(ticks - 1);

        double D, h, C, tau, ddq, q_ref, dq_ref, ddq_ref;
        dynamics_step(&p, mode, sim_t, dt, &q, &dq,
                      &D, &h, &C, &tau, &ddq,
                      &q_ref, &dq_ref, &ddq_ref);

        synthetic_workload(workload_loops, &sink);

        sim_t += dt;
        q_ref = qd(sim_t);
        dq_ref = dqd(sim_t);
        ddq_ref = ddqd(sim_t);

        double err_q = q_ref - q;
        double err_dq = dq_ref - dq;
        double abs_err_q = fabs(err_q);
        double abs_err_dq = fabs(err_dq);

        sum_abs_err_q += abs_err_q;
        sum_abs_err_dq += abs_err_dq;
        sum_sq_err_q += err_q * err_q;
        sum_sq_err_dq += err_dq * err_dq;
        if (abs_err_q > max_abs_err_q) max_abs_err_q = abs_err_q;
        if (abs_err_dq > max_abs_err_dq) max_abs_err_dq = abs_err_dq;

        clock_gettime(CLOCK_MONOTONIC, &actual_end);
        int64_t exec_ns = ts_to_ns(&actual_end) - actual_start_ns;
        double exec_us = (double)exec_ns / 1000.0;

        if (exec_us < min_exec_us) min_exec_us = exec_us;
        if (exec_us > max_exec_us) max_exec_us = exec_us;
        sum_exec_us += exec_us;

        shm->t = sim_t;
        shm->q = q;
        shm->dq = dq;
        shm->ddq = ddq;
        shm->D = D;
        shm->h = h;
        shm->C = C;
        shm->tau = tau;
        shm->q_ref = q_ref;
        shm->dq_ref = dq_ref;
        shm->ddq_ref = ddq_ref;

        shm->err_q = err_q;
        shm->err_dq = err_dq;
        shm->abs_err_q = abs_err_q;
        shm->abs_err_dq = abs_err_dq;
        shm->avg_abs_err_q = sum_abs_err_q / (double)(i + 1);
        shm->avg_abs_err_dq = sum_abs_err_dq / (double)(i + 1);
        shm->rms_err_q = sqrt(sum_sq_err_q / (double)(i + 1));
        shm->rms_err_dq = sqrt(sum_sq_err_dq / (double)(i + 1));
        shm->max_abs_err_q = max_abs_err_q;
        shm->max_abs_err_dq = max_abs_err_dq;

        shm->jitter_us = jitter_us;
        shm->min_jitter_us = min_jitter_us;
        shm->max_jitter_us = max_jitter_us;
        shm->avg_jitter_us = sum_jitter_us / (double)(i + 1);
        shm->avg_abs_jitter_us = sum_abs_jitter_us / (double)(i + 1);

        shm->exec_time_us = exec_us;
        shm->min_exec_time_us = min_exec_us;
        shm->max_exec_time_us = max_exec_us;
        shm->avg_exec_time_us = sum_exec_us / (double)(i + 1);

        shm->loop_count = i + 1;
        expected_ns += (int64_t)ticks * period_ns;
    }

    shm->finished = 1;

    printf("\n=== RT Performance Summary ===\n");
    printf("mode              : %s\n", mode_to_string(mode));
    printf("period            : %d us\n", period_us);
    printf("loops             : %lld\n", shm->loop_count);
    printf("current jitter    : %.3f us\n", shm->jitter_us);
    printf("min jitter        : %.3f us\n", shm->min_jitter_us);
    printf("max jitter        : %.3f us\n", shm->max_jitter_us);
    printf("avg jitter        : %.3f us\n", shm->avg_jitter_us);
    printf("avg abs jitter    : %.3f us\n", shm->avg_abs_jitter_us);
    printf("current exec time : %.3f us\n", shm->exec_time_us);
    printf("min exec time     : %.3f us\n", shm->min_exec_time_us);
    printf("max exec time     : %.3f us\n", shm->max_exec_time_us);
    printf("avg exec time     : %.3f us\n", shm->avg_exec_time_us);
    printf("deadline miss     : %lld\n", shm->miss_count);
    printf("overrun count     : %lld\n", shm->overrun_count);
    printf("final q           : %.4f rad\n", q);
    printf("final dq          : %.4f rad/s\n", dq);
    printf("avg abs err q     : %.4f rad\n", shm->avg_abs_err_q);
    printf("rms err q         : %.4f rad\n", shm->rms_err_q);
    printf("max abs err q     : %.4f rad\n", shm->max_abs_err_q);
    printf("avg abs err dq    : %.4f rad/s\n", shm->avg_abs_err_dq);
    printf("rms err dq        : %.4f rad/s\n", shm->rms_err_dq);
    printf("max abs err dq    : %.4f rad/s\n", shm->max_abs_err_dq);

    close(tfd);
    close(efd);
    munmap(shm, sizeof(SharedState));
    close(shm_fd);
    return 0;
}

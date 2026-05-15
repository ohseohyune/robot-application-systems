#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <pthread.h>
#include <stdint.h>

#define SHM_NAME "/evl_sine_cosine_shm"

#define MAX_SAMPLES       200000
#define TICK_PERIOD_NS    1000000LL   // 1 ms
#define SIGNAL_FREQ_HZ    1.0
#define PLOT_REFRESH_US   50000       // 50 ms
#define LIVE_WINDOW_SEC   2.0

typedef struct {
    pthread_mutex_t mutex;

    int initialized;
    int running;
    double sim_time_sec;

    long long count;
    long long capacity;

    // statistics
    long long current_jitter_ns;
    long long min_jitter_ns;
    long long max_jitter_ns;
    long long total_jitter_ns;
    double avg_jitter_ns;
    long long samples;

    long long current_overrun;
    long long total_overruns;
    long long max_overrun;
    long long overrun_events;

    // data arrays
    double t[MAX_SAMPLES];
    double s[MAX_SAMPLES];
    double c[MAX_SAMPLES];
    double j[MAX_SAMPLES];
} shared_data_t;

#endif
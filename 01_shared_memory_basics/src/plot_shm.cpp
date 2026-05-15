#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "shared_data.h"

typedef struct {
    long long count;
    long long samples;

    long long current_jitter_ns;
    long long min_jitter_ns;
    long long max_jitter_ns;
    double avg_jitter_ns;

    long long current_overrun;
    long long total_overruns;
    long long max_overrun;
    long long overrun_events;

    int running;
    double sim_time_sec;

    double t[MAX_SAMPLES];
    double s[MAX_SAMPLES];
    double c[MAX_SAMPLES];
    double j[MAX_SAMPLES];
} snapshot_t;

static void make_snapshot(shared_data_t *shm, snapshot_t *snap)
{
    pthread_mutex_lock(&shm->mutex);

    memset(snap, 0, sizeof(*snap));
    snap->count = shm->count;
    snap->samples = shm->samples;
    snap->current_jitter_ns = shm->current_jitter_ns;
    snap->min_jitter_ns = (shm->samples > 0) ? shm->min_jitter_ns : 0;
    snap->max_jitter_ns = shm->max_jitter_ns;
    snap->avg_jitter_ns = shm->avg_jitter_ns;
    snap->current_overrun = shm->current_overrun;
    snap->total_overruns = shm->total_overruns;
    snap->max_overrun = shm->max_overrun;
    snap->overrun_events = shm->overrun_events;
    snap->running = shm->running;
    snap->sim_time_sec = shm->sim_time_sec;

    for (long long i = 0; i < shm->count; i++) {
        snap->t[i] = shm->t[i];
        snap->s[i] = shm->s[i];
        snap->c[i] = shm->c[i];
        snap->j[i] = shm->j[i];
    }

    pthread_mutex_unlock(&shm->mutex);
}

static void plot_frame(FILE *gp, const snapshot_t *snap, int final_view)
{
    double now_t = 0.0;
    if (snap->count > 0)
        now_t = snap->t[snap->count - 1];

    double x_min = 0.0;
    double x_max = final_view ? snap->sim_time_sec : LIVE_WINDOW_SEC;

    if (!final_view && snap->count > 0) {
        x_max = (now_t > LIVE_WINDOW_SEC) ? now_t : LIVE_WINDOW_SEC;
        x_min = (now_t > LIVE_WINDOW_SEC) ? (now_t - LIVE_WINDOW_SEC) : 0.0;
    }

    double jitter_plot_max = 1000.0;
    if (snap->max_jitter_ns > 0) {
        jitter_plot_max = snap->max_jitter_ns * 1.2;
        if (jitter_plot_max < 1000.0)
            jitter_plot_max = 1000.0;
    }

    fprintf(gp, "clear\n");
    fprintf(gp, "set multiplot\n");

    fprintf(gp, "set size 0.26, 0.90\n");
    fprintf(gp, "set origin 0.02, 0.05\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "unset xtics\n");
    fprintf(gp, "unset ytics\n");
    fprintf(gp, "unset xlabel\n");
    fprintf(gp, "unset ylabel\n");
    fprintf(gp, "unset grid\n");
    fprintf(gp, "set border 1\n");
    fprintf(gp, "set xrange [0:1]\n");
    fprintf(gp, "set yrange [0:1]\n");

    fprintf(gp,
        "set object 1 rect from 0,0 to 1,1 "
        "fc rgb 'white' fillstyle solid 1.0 border rgb 'black'\n");

    fprintf(gp,
        "set label 1 at 0.04,0.96 left front "
        "\"[INFO]\\n"
        "Signal freq    : %.3f Hz\\n"
        "Sampling       : %.3f ms\\n"
        "Plot refresh   : %.3f ms\\n"
        "Now time       : %.3f s\\n"
        "Samples        : %lld\\n"
        "\\n"
        "Current jitter : %lld ns\\n"
        "Min jitter     : %lld ns\\n"
        "Max jitter     : %lld ns\\n"
        "Avg jitter     : %.2f ns\\n"
        "\\n"
        "Current overrun: %lld\\n"
        "Overrun events : %lld\\n"
        "Total overruns : %lld\\n"
        "Max overrun    : %lld\\n"
        "\\n"
        "Mode           : %s\" front\n",
        SIGNAL_FREQ_HZ,
        TICK_PERIOD_NS / 1e6,
        PLOT_REFRESH_US / 1000.0,
        now_t,
        snap->samples,
        snap->current_jitter_ns,
        snap->min_jitter_ns,
        snap->max_jitter_ns,
        snap->avg_jitter_ns,
        snap->current_overrun,
        snap->overrun_events,
        snap->total_overruns,
        snap->max_overrun,
        final_view ? "FINAL" : "LIVE");

    fprintf(gp, "plot NaN notitle\n");
    fprintf(gp, "unset label 1\n");
    fprintf(gp, "unset object 1\n");

    fprintf(gp, "set size 0.68, 0.42\n");
    fprintf(gp, "set origin 0.30, 0.53\n");
    fprintf(gp, "set border\n");
    fprintf(gp, "set xtics\n");
    fprintf(gp, "set ytics\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set xlabel 'Time (s)'\n");
    fprintf(gp, "set ylabel 'Amplitude'\n");
    fprintf(gp, "set title 'EVL Sine / Cosine Plot'\n");
    fprintf(gp, "set xrange [%f:%f]\n", x_min, x_max);
    fprintf(gp, "set yrange [-1.2:1.2]\n");
    fprintf(gp, "set key right top\n");

    fprintf(gp, "plot '-' w l lw 2 title 'sine', '-' w l lw 2 title 'cosine'\n");

    for (long long i = 0; i < snap->count; i++) {
        if (snap->t[i] >= x_min && snap->t[i] <= x_max)
            fprintf(gp, "%f %f\n", snap->t[i], snap->s[i]);
    }
    fprintf(gp, "e\n");

    for (long long i = 0; i < snap->count; i++) {
        if (snap->t[i] >= x_min && snap->t[i] <= x_max)
            fprintf(gp, "%f %f\n", snap->t[i], snap->c[i]);
    }
    fprintf(gp, "e\n");

    fprintf(gp, "set size 0.68, 0.38\n");
    fprintf(gp, "set origin 0.30, 0.08\n");
    fprintf(gp, "set border\n");
    fprintf(gp, "set xtics\n");
    fprintf(gp, "set ytics\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set xlabel 'Time (s)'\n");
    fprintf(gp, "set ylabel 'Jitter (ns)'\n");
    fprintf(gp, "set title 'Jitter History'\n");
    fprintf(gp, "set xrange [%f:%f]\n", x_min, x_max);
    fprintf(gp, "set yrange [0:%f]\n", jitter_plot_max);
    fprintf(gp, "set key right top\n");

    fprintf(gp, "plot '-' w l lw 2 title 'jitter(ns)'\n");
    for (long long i = 0; i < snap->count; i++) {
        if (snap->t[i] >= x_min && snap->t[i] <= x_max)
            fprintf(gp, "%f %f\n", snap->t[i], snap->j[i]);
    }
    fprintf(gp, "e\n");

    fprintf(gp, "unset multiplot\n");
    fflush(gp);
}

int main(void)
{
    int shm_fd;
    shared_data_t *shm = NULL;

    while (1) {
        shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
        if (shm_fd >= 0)
            break;

        printf("shared memory 대기중...\n");
        usleep(200000);
    }

    shm = (shared_data_t *)mmap(NULL, sizeof(shared_data_t),
                                PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("mmap 실패");
        close(shm_fd);
        return 1;
    }

    FILE *gp = popen("gnuplot -persist", "w");
    if (!gp) {
        perror("gnuplot 실행 실패");
        munmap(shm, sizeof(shared_data_t));
        close(shm_fd);
        return 1;
    }

    fprintf(gp, "set term qt 0 size 1600,950 title 'EVL Shared Memory Plot'\n");
    fflush(gp);

    snapshot_t snap;
    while (1) {
        make_snapshot(shm, &snap);
        plot_frame(gp, &snap, 0);

        if (!snap.running)
            break;

        usleep(PLOT_REFRESH_US);
    }

    make_snapshot(shm, &snap);
    plot_frame(gp, &snap, 1);

    pclose(gp);
    munmap(shm, sizeof(shared_data_t));
    close(shm_fd);

    return 0;
}
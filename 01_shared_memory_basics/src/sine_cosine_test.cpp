#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <evl/thread.h>
#include <evl/clock.h>
#include <evl/timer.h>

#include "shared_data.h"

static volatile int keep_running = 1;
static shared_data_t *g_shm = NULL;

static long long ts_to_ns(const struct timespec *t)
{
    return (long long)t->tv_sec * 1000000000LL + (long long)t->tv_nsec;
}

static void add_ns(struct timespec *r, const struct timespec *t, long long ns)
{
    r->tv_sec  = t->tv_sec + ns / 1000000000LL;
    r->tv_nsec = t->tv_nsec + ns % 1000000000LL;

    if (r->tv_nsec >= 1000000000L) {
        r->tv_sec++;
        r->tv_nsec -= 1000000000L;
    } else if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += 1000000000L;
    }
}

static void stop_handler(int sig)
{
    (void)sig;
    keep_running = 0;
    if (g_shm) {
        pthread_mutex_lock(&g_shm->mutex);
        g_shm->running = 0;
        pthread_mutex_unlock(&g_shm->mutex);
    }
}

static int init_shared_memory(double sim_time, shared_data_t **out_shm, int *out_fd)
{
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("shm_open 실패");
        return -1;
    }

    if (ftruncate(shm_fd, sizeof(shared_data_t)) < 0) {
        perror("ftruncate 실패");
        close(shm_fd);
        return -1;
    }

    void *addr = mmap(NULL, sizeof(shared_data_t),
                      PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (addr == MAP_FAILED) {
        perror("mmap 실패");
        close(shm_fd);
        return -1;
    }

    shared_data_t *shm = (shared_data_t *)addr;
    memset(shm, 0, sizeof(shared_data_t));

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    pthread_mutex_lock(&shm->mutex);
    shm->initialized = 1;
    shm->running = 1;
    shm->sim_time_sec = sim_time;
    shm->capacity = MAX_SAMPLES;
    shm->min_jitter_ns = (1LL << 60);
    pthread_mutex_unlock(&shm->mutex);

    *out_shm = shm;
    *out_fd = shm_fd;
    return 0;
}

static void push_sample(shared_data_t *shm,
                        double t, double s, double c,
                        long long jitter_ns, long long overruns)
{
    pthread_mutex_lock(&shm->mutex);

    if (shm->count < shm->capacity) {
        long long i = shm->count;
        shm->t[i] = t;
        shm->s[i] = s;
        shm->c[i] = c;
        shm->j[i] = (double)jitter_ns;
        shm->count++;
    }

    shm->current_jitter_ns = jitter_ns;

    if (jitter_ns < shm->min_jitter_ns)
        shm->min_jitter_ns = jitter_ns;
    if (jitter_ns > shm->max_jitter_ns)
        shm->max_jitter_ns = jitter_ns;

    shm->total_jitter_ns += jitter_ns;
    shm->samples++;
    shm->avg_jitter_ns =
        (double)shm->total_jitter_ns / (double)shm->samples;

    shm->current_overrun = overruns;
    shm->total_overruns += overruns;

    if (overruns > 0) {
        shm->overrun_events++;
        if (overruns > shm->max_overrun)
            shm->max_overrun = overruns;
    }

    pthread_mutex_unlock(&shm->mutex);
}

int main(void)
{
    signal(SIGINT, stop_handler);

    double sim_time = 10.0;
    printf("시뮬레이션 시간(초): ");
    fflush(stdout);
    if (scanf("%lf", &sim_time) != 1 || sim_time <= 0.0)
        sim_time = 10.0;

    int shm_fd;
    if (init_shared_memory(sim_time, &g_shm, &shm_fd) != 0)
        return 1;

    int tfd = evl_attach_self("sine-test-%d", getpid());
    if (tfd < 0) {
        fprintf(stderr, "EVL attach 실패: %s\n", strerror(-tfd));
        return 1;
    }

    int tmfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
    if (tmfd < 0) {
        fprintf(stderr, "timer 생성 실패: %s\n", strerror(-tmfd));
        evl_detach_self();
        return 1;
    }

    struct timespec now, expected;
    struct itimerspec its, oldits;
    int ret = evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
    if (ret < 0) {
        fprintf(stderr, "clock 읽기 실패\n");
        close(tmfd);
        evl_detach_self();
        return 1;
    }

    add_ns(&its.it_value, &now, TICK_PERIOD_NS);
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = TICK_PERIOD_NS;

    ret = evl_set_timer(tmfd, &its, &oldits);
    if (ret < 0) {
        fprintf(stderr, "timer 설정 실패\n");
        close(tmfd);
        evl_detach_self();
        return 1;
    }

    expected = its.it_value;
    uint64_t ticks;
    double elapsed = 0.0;
    const double w = 2.0 * M_PI * SIGNAL_FREQ_HZ;

    while (keep_running && elapsed < sim_time) {
        // 타이머가 만료될 때까지 실시간 모드로 대기
        ret = oob_read(tmfd, &ticks, sizeof(ticks));
        if (ret != (int)sizeof(ticks))
            break;

        // 현재 시각을 읽어와서 예상 시각과의 차이를 계산
        ret = evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
        if (ret < 0)
            break;

        // ticks는 타이머가 만료된 횟수이므로, 1회 이상 만료된 경우 오버런이 발생한 것으로 간주
        long long overruns = (ticks > 0) ? ((long long)ticks - 1) : 0;

        struct timespec expected_last = expected;
        if (ticks > 1)
            add_ns(&expected_last, &expected,
                ((long long)ticks - 1) * TICK_PERIOD_NS);

        // Jitter 계산
        long long actual_ns   = ts_to_ns(&now);
        long long expected_ns = ts_to_ns(&expected_last);
        long long jitter_ns   = llabs(actual_ns - expected_ns);

        // 작업지연 발생시 시간 맞추기
        elapsed += (double)((long long)ticks * TICK_PERIOD_NS) / 1e9;
        if (elapsed > sim_time)
            elapsed = sim_time;

        // 데이터 생성 및 공유 메모리 전송
        push_sample(g_shm, elapsed,
                    sin(w * elapsed), cos(w * elapsed),
                    jitter_ns, overruns);

        // 다음 Deadline 설정
        add_ns(&expected, &expected, (long long)ticks * TICK_PERIOD_NS);
    }

    pthread_mutex_lock(&g_shm->mutex);
    g_shm->running = 0;
    pthread_mutex_unlock(&g_shm->mutex);

    printf("\n--- 결과 ---\n");
    printf("샘플링 주기      : %.3f ms\n", TICK_PERIOD_NS / 1e6);

    pthread_mutex_lock(&g_shm->mutex);
    printf("총 샘플 수       : %lld\n", g_shm->samples);
    if (g_shm->samples > 0) {
        printf("최소 지터        : %lld ns\n", g_shm->min_jitter_ns);
        printf("최대 지터        : %lld ns\n", g_shm->max_jitter_ns);
        printf("평균 지터        : %.2f ns\n", g_shm->avg_jitter_ns);
    }
    printf("오버런 발생 횟수 : %lld\n", g_shm->overrun_events);
    printf("누적 오버런 수   : %lld\n", g_shm->total_overruns);
    printf("최대 오버런      : %lld\n", g_shm->max_overrun);
    pthread_mutex_unlock(&g_shm->mutex);

    close(tmfd);
    evl_detach_self();

    munmap(g_shm, sizeof(shared_data_t));
    close(shm_fd);

    return 0;
}
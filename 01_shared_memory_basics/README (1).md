# 01 — Shared Memory Basics & SHM Visualization

> **Week 05** | Inter-Process Communication via POSIX Shared Memory

---

## 목적 및 이론적 배경

### 핵심 개념
실시간 로봇 제어 시스템에서 **프로세스 간 통신(IPC)** 은 필수 인프라다.
이 모듈은 POSIX Shared Memory를 이용해 두 프로세스가 데이터를 주고받는 기본 패턴을 구현한다.

```
[Writer Process]  →  /dev/shm/shared_data  →  [Reader/Plotter Process]
  sine_cosine_test                               plot_shm
```

### 왜 Shared Memory인가
| IPC 방식 | 속도 | 복잡도 | 실시간 적합성 |
|---|---|---|---|
| Socket | 느림 | 중간 | 낮음 |
| Pipe | 중간 | 낮음 | 중간 |
| **Shared Memory** | **매우 빠름** | 낮음 | **높음** |
| Message Queue | 중간 | 중간 | 중간 |

실시간 제어에서 센서→제어기→시각화 파이프라인은 수 ms 이내에 데이터가 전달되어야 한다.
Shared Memory는 커널 복사 없이 동일 메모리 공간을 참조하므로 지연이 최소화된다.

### SHM (Simple Harmonic Motion)
시험 신호로 사용된 정현파:

```
x(t) = A·sin(ωt + φ)
v(t) = A·ω·cos(ωt + φ)
```

`withoutStress.png` / `withStress.png` 는 부하 유무에 따른 신호 품질 비교 결과다.

---

## 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Linux (Ubuntu 22.04) |
| Compiler | g++ -std=c++17 |
| IPC | POSIX shm_open / mmap |
| Visualization | (plot_shm 바이너리 직접 실행) |
| 의존성 | `-lrt` (POSIX real-time library) |

---

## 코드 구조

```
01_shared_memory_basics/
├── src/
│   ├── shared_data.h          # 공유 메모리 데이터 구조체 정의
│   ├── sine_cosine_test.cpp   # Writer: sin/cos 신호 생성 → shm write
│   └── plot_shm.cpp           # Reader: shm read → 터미널 출력/플롯
└── results/
    ├── withoutStress.png       # 부하 없을 때 신호
    └── withStress.png          # 부하 있을 때 신호 (지터 확인)
```

**데이터 흐름:**
```
sine_cosine_test
  └─ shm_open("shared_data", O_CREAT)
  └─ 매 루프: x = sin(t), y = cos(t) → shm write
                        ↓
                  /dev/shm/shared_data
                        ↓
plot_shm
  └─ shm_open("shared_data", O_RDONLY)
  └─ 매 루프: shm read → stdout
```

**`shared_data.h` 구조체 핵심:**
```cpp
struct SharedData {
    double time;
    double sine_val;
    double cosine_val;
    // mutex or flag for sync (있는 경우)
};
```

---

## 빌드 및 실행

```bash
cd 01_shared_memory_basics/src

# 빌드
g++ -std=c++17 -O2 sine_cosine_test.cpp -o sine_cosine_test -lrt
g++ -std=c++17 -O2 plot_shm.cpp         -o plot_shm         -lrt

# 터미널 1: Writer 실행
./sine_cosine_test

# 터미널 2: Reader/Plotter 실행
./plot_shm
```

---

## 실제 적용 가능한 유스케이스

| 유스케이스 | 설명 |
|---|---|
| **센서 드라이버 ↔ 제어기** | 센서 드라이버 프로세스가 shm에 데이터를 쓰고, 제어기 프로세스가 읽음 |
| **제어기 ↔ 시각화** | 제어 루프와 시각화 루프를 분리해 실시간성 보장 |
| **ROS2 없는 경량 IPC** | 임베디드 환경에서 ROS2 오버헤드 없이 프로세스 간 통신 |
| **HIL 테스트** | 시뮬레이션 플랜트와 실제 제어기 코드를 shm으로 연결 |

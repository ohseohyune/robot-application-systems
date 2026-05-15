# 02 — Single Pendulum Real-Time Control (EVL)

> **Week 06** | 1-DOF Real-Time Control using EVL (Embedded Value Layer) RT Framework

---

## 목적 및 이론적 배경

### 시스템 모델 — 단진자 (Single Pendulum)

```
       pivot
         |
         | L
         |
        [m]  ← 질량

운동방정식 (비선형):
  mL²θ̈ + bθ̇ + mgL·sin(θ) = τ

선형화 (θ ≈ 0):
  mL²θ̈ + bθ̇ + mgLθ = τ

상태공간:
  x = [θ, θ̇]ᵀ
  ẋ = [θ̇, (τ - bθ̇ - mgLθ) / mL²]ᵀ
```

### EVL Real-Time Framework
EVL(Embedded Value Layer)은 리눅스 커널 위에서 하드 실시간(Hard Real-Time) 제어 루프를 실행하기 위한 프레임워크다.

```
┌─────────────────────────────────┐
│  EVL Real-Time Core             │  ← 결정론적 실행 (jitter < 수십 μs)
│  evl_rt_single (C++)            │
│  └─ 제어 루프 @ 고정 주기        │
│     └─ shm write → 상태 공유    │
├─────────────────────────────────┤
│  Linux User Space               │
│  rt_viewer.py / rt_dashboard.py │  ← 비실시간 시각화
│  └─ shm read → 플롯             │
└─────────────────────────────────┘
```

**왜 EVL인가:**
일반 Linux 스케줄러는 수 ms의 jitter가 발생해 고속 제어 루프(>1kHz)에 부적합하다.
EVL은 리눅스 커널을 건드리지 않고 실시간 코어를 추가해 μs 수준의 결정론적 실행을 보장한다.

---

## 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Linux with EVL-patched kernel |
| Framework | EVL (libevl) |
| Compiler | g++ -std=c++17 |
| Visualization | Python 3.x (PyQt / matplotlib) |
| IPC | POSIX Shared Memory |
| 의존성 | `-levl`, `-lrt`, `-lpthread` |

---

## 코드 구조

```
02_single_pendulum_rt/
├── src/
│   ├── single_pendulum_shm.h   # shm 구조체: θ, θ̇, τ, timestamp
│   ├── load_common.h           # 시스템 파라미터 (m, L, b, g)
│   └── evl_rt_single.cpp       # 메인 RT 제어 루프
└── viz/
    ├── rt_viewer.py            # 실시간 상태 플롯 (θ, θ̇)
    └── rt_dashboard.py         # 종합 대시보드 (상태 + 제어 입력)
```

**제어 루프 흐름 (`evl_rt_single.cpp`):**
```
EVL Thread 초기화
└─ evl_attach_self()
└─ shm 생성 및 초기화
└─ 주기 타이머 설정 (Ts = ___ ms)

RT Loop (매 Ts마다):
  1. 현재 상태 읽기: θ[k], θ̇[k]
  2. 제어 입력 계산: τ[k] = Controller(θ[k], θ̇[k], r[k])
  3. 시뮬레이션 업데이트: Euler / RK4
  4. 상태 → shm write
  5. evl_sleep_until() → 다음 주기 대기
```

**`single_pendulum_shm.h` 핵심 구조:**
```cpp
struct PendulumShm {
    double theta;       // [rad] 각도
    double theta_dot;   // [rad/s] 각속도
    double tau;         // [Nm] 제어 토크
    double t;           // [s] 타임스탬프
};
```

---

## 빌드 및 실행

```bash
cd 02_single_pendulum_rt/src

# 빌드 (EVL 설치 필요)
g++ -std=c++17 -O2 evl_rt_single.cpp -o evl_rt_single -levl -lrt -lpthread

# 터미널 1: RT 제어 루프 실행
sudo ./evl_rt_single

# 터미널 2: 시각화
cd ../viz
python3 rt_viewer.py
# 또는
python3 rt_dashboard.py
```

> ⚠️ EVL RT 스레드는 `sudo` 또는 CAP_SYS_NICE 권한이 필요하다.

---

## 실제 적용 가능한 유스케이스

| 유스케이스 | 설명 |
|---|---|
| **역진자 균형 제어** | LQR/PID로 불안정 평형점 안정화 |
| **로봇 관절 1-DOF 제어** | 단일 관절 각도 추적 제어기 프로토타입 |
| **실시간 제어기 검증** | EVL 루프에서 jitter 측정 및 타이밍 분석 |
| **교육용 제어 실험** | 게인 조정 → 즉각적 시각화로 직관적 학습 |

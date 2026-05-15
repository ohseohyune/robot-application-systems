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

---

## 1. 경로 비교 (Path Analysis)

### [일반 Linux 경로 — In-Band]
일반적인 리눅스 환경에서는 모든 작업이 커널의 표준 실행 경로를 따릅니다.
- **흐름:** 앱 → `read()` → Linux 시스템콜 → 일반 스케줄러 → 프로세스 깨움
- **병목 현상:** 리눅스 커널의 다른 인터럽트 처리, 디스크 I/O, 네트워크 작업 등에 의해 실행이 지연될 수 있습니다.
- **성능:** 지터(Jitter)가 보통 수 μs에서 수십 μs까지 발생합니다.

### [EVL 경로 — Out-Of-Band]
EVL은 리눅스 커널 옆에 병렬적으로 존재하는 전용 하이퍼바이저/코어 층을 통해 통신합니다.
- **흐름:** 앱 → `oob_read()` → **EVL 코어** (별도 관리) → 즉시 깨움
- **장점:** 리눅스 커널의 활동(Heavy Load)과 완전히 무관하게 동작합니다. 하드웨어 인터럽트를 EVL이 먼저 가로채어 처리합니다.
- **성능:** 지터(Jitter)가 대부분 **1μs 미만**으로 억제됩니다.

---

## 2. 직관적인 비유

| 구분 | 일반 Linux (In-Band) | EVL (Out-Of-Band) |
| :--- | :--- | :--- |
| **비유** | "공용 관리자가 1ms마다 깨워줌" | "VIP 전용 비서가 옆에서 대기함" |
| **상황** | 관리자가 다른 민원 업무를 처리하느라 늦을 수 있음 | 리눅스 관리자가 무엇을 하든, 비서는 정해진 시간에 정확히 깨움 |
| **스케줄링** | 일반 대기열에서 차례를 기다림 | 전용 급행 라인을 이용함 |

---

## 3. 핵심 함수: `evl_attach_self()`

이 함수는 해당 스레드의 관리 주체를 변경하는 결정적인 역할을 합니다.

- **의미:** "이 스레드를 일반 리눅스 스케줄러가 아닌, **EVL 전용 비서(OOB 코어)의 관리 명단**에 올린다."
- **효과:** 호출 직후부터 해당 스레드는 Out-Of-Band 경로를 사용할 수 있는 자격을 얻으며, 리눅스 커널의 간섭으로부터 자유로워집니다.

---

## 4. 요약

EVL의 강력함은 리눅스를 단순히 "빨리" 만드는 것이 아니라, 리눅스 커널의 복잡성으로부터 실시간 작업만을 분리(Isolation)하여 별도의 고속 경로를 제공하는 데에서 나옵니다.
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

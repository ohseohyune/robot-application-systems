# 01 — Shared Memory Basics & SHM Visualization

> **Week 05** | Inter-Process Communication via POSIX Shared Memory

---

## 개념 정리 (Conceptual Primer)

> 이 코드를 이해하려면 먼저 **프로세스/스레드/CPU** 개념이 머리에 있어야 한다.
> 비유로 한 번에 잡아보자.

### 프로그램 vs 프로세스

| 구분 | 정의 | 상태 |
|---|---|---|
| **프로그램 (Program)** | 디스크에 있는 파일 (`sine_cosine_test`) | 정적 — 가만히 있는 코드 덩어리 |
| **프로세스 (Process)** | 그 파일이 메모리에 올라가서 실행 중인 것 | 동적 — 살아 움직이는 실행체 |

프로세스는 OS(운영체제)로부터 다음 자원들을 지원받는다:

1. **PID (Process ID)** — 고유 번호 (예: 12345)
2. **메모리 공간** — 다른 프로세스는 접근하지 못하고 자기만 쓰는 주소 공간 (회사 사무실)
3. **파일 핸들** — 열어놓은 파일들
4. **스레드 최소 1개** — 실제로 일하는 주체 (직원)

---

### 핵심 비유

| 실세계 | 컴퓨터 세계 |
|---|---|
| 거대한 빌딩 | 컴퓨터 |
| 빌딩 관리자 | 운영체제 (OS) |
| 실제로 일하는 손 (보통 4~16개) | CPU (코어) |
| 하나의 회사 | 프로세스 |
| 회사의 사무실 | 메모리 |
| 회사의 직원 | 스레드 |

---

### 1. 프로세스 = 하나의 회사

크롬을 켜면 크롬이라는 회사가 생기고, 한글을 켜면 한글이라는 회사가 또 생긴다.

```
빌딩(컴퓨터) 안:
├─ 크롬 회사 (프로세스)
├─ 한글 회사 (프로세스)
├─ VSCode 회사 (프로세스)
└─ sine_cosine_test 회사 (프로세스)  ← ./sine_cosine_test 실행
```

> 회사마다 **자기 사무실(메모리)을 따로** 가진다. 다른 회사의 사무실에 접근할 수 없다.
>
> <img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/c9563f75-d8ea-49db-9d09-6f52f7122cea" />


---

### 2. 스레드 = 직원

각 회사에는 직원이 최소 1명 있다. 회사가 일이 많으면 직원을 여러 명 둘 수 있다.

```
크롬 회사 (프로세스):
├─ 직원 1 (스레드): 화면 그리는 일
├─ 직원 2 (스레드): 인터넷 통신
├─ 직원 3 (스레드): 사용자 클릭 감지
└─ ...

sine_cosine_test 회사 (프로세스):
└─ 직원 1 (스레드): 모든 일 혼자 함  ← 현재 코드
```

> 한 회사 안의 직원들은 **같은 사무실(메모리)을 공유**한다.
>
> <img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/c1b821f1-3b11-4568-8af0-6d7069abc62b" />


---

### 3. CPU = 실제 일하는 손

직원들은 손이 모두 없다. 건물의 관리자(OS)가 손을 끼워줘야 일할 수 있다.

빌딩에 손이 4개밖에 없다고 치자 (CPU 4코어). 그런데 직원은 수백 명이다.
→ 관리자가 **매우매우×999 빠르게**, 직원들에게 손을 바꿔 끼워준다.

```
0.001초: 손1 → 크롬 직원1 일 시킴
0.002초: 손1 → 한글 직원2 일 시킴
0.003초: 손1 → sine_cosine_test 직원1 일 시킴
0.004초: 손1 → 다시 크롬 직원1 일 시킴
...
```

> 손 4개가 동시에 이렇게 작동한다. **(병렬)**
> 같은 순간에 진짜로 4가지 일이 동시에 진행되며, 거기에 각 손이 또 빠르게 직원을 바꿔가며 일하니까 전체적으로 수백 개의 일이 "동시처럼" 보인다.

<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/dd46c4d4-26e6-410c-b369-72d08cdbcf9a" />

---

### 4. "스레드가 잔다" = 직원이 의자에 앉아 대기

`oob_read`라는 함수를 만나면, 직원이 관리자에게 말한다:

> "저 1ms 후에 깨워주세요. 그동안 다른 직원한테 손 주세유."

손(CPU)은 한 번도 쉬지 않는다. **그 직원에게 안 갈 뿐**이다.

<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/1b687f05-5570-4fff-ab0d-0b92c758ada0" />

---

### 5. EVL RT 스레드 vs 일반 스레드

**일반 직원 (보통 스레드)** — 주말 아침 느낌. 일찍 일어나면 좋지만 좀 더 자도 괜찮은…
```
직원: "1ms 후에 깨워주세요"
관리자: "어, 알겠음. 근데 그때 바쁘면 좀 늦을 수 있음 ㅎㅎ"
→ 실제로는 1.5ms, 2ms 후에 깨어날 수도 있음
```

**EVL RT 직원 (실시간 스레드)** — 아침 일찍 중요한 시험이 있는 날.
```
직원: "1ms 후에 깨워주세요. 무조건요."
관리자: "넵 VIP시군요. 무조건 정시에 깨워드림"
→ 거의 정확히 1ms 후 깨어남 (오차 μs 수준)
```
<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/40fcad12-7c85-4027-88aa-bc2ffaa2b9e7" />

**왜 VIP가 필요한가:**
로봇 제어에서는 1ms 늦으면 로봇이 넘어진다. 0.5ms 빨라도 위험하다. 정확한 시간에 정확하게 일해야 한다.
- 일반 직원: "어 좀 늦어도 괜찮잖아? 영화 1초 늦게 나오는 게 뭐가 문제임?"
- RT 직원: "안 됨. 1ms 안에 무조건 계산 끝내야 함."

---

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

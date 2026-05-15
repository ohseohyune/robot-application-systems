# 03 — 2-DOF System Real-Time Control (EVL)

> **Week 07~08** | 2-DOF Real-Time Control with Progressive Refinement

---

## 목적 및 이론적 배경

### 시스템 모델 — 2-DOF 시스템

```
       pivot₁
         |
         | L₁, m₁
         |
       pivot₂ (θ₁)
         |
         | L₂, m₂
         |
        [m₂] (θ₂)

일반화 좌표: q = [θ₁, θ₂]ᵀ
```

**Euler-Lagrange 운동방정식:**
```
M(q)q̈ + C(q,q̇)q̇ + g(q) = τ

M(q) : (2×2) 관성 행렬 (상태 의존)
C(q,q̇): (2×2) 코리올리/원심력 행렬
g(q) : (2×1) 중력 벡터
τ    : (2×1) 제어 토크 입력
```

**선형화 (q ≈ 0):**
```
상태: x = [θ₁, θ₂, θ̇₁, θ̇₂]ᵀ ∈ ℝ⁴
입력: u = [τ₁, τ₂]ᵀ ∈ ℝ²

ẋ = Ax + Bu
y  = Cx
```

### Week 07 → Week 08 변경 사항

| 항목 | Week 07 | Week 08 |
|---|---|---|
| 공통 헤더 | `load_2dof_common.h` + `load_2dof_common2.h` | `load_2dof_common3.h` (통합) |
| 목적 | 2-DOF RT 기본 구현 | 제어기 개선 / 파라미터 튜닝 |
| 구조 변화 | 헤더 분리 | 헤더 통합 → 코드 정리 |

> 두 주차를 같은 폴더에 보존하는 이유:
> 동일 시스템의 **리팩토링 과정**이 코드 진화를 보여주는 포트폴리오적 가치가 있다.

---

## 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Linux with EVL-patched kernel |
| Framework | EVL (libevl) |
| Compiler | g++ -std=c++17 |
| Visualization | Python 3.x |
| IPC | POSIX Shared Memory |
| 의존성 | `-levl`, `-lrt`, `-lpthread` |

---

## 코드 구조

```
03_2dof_system_rt/
├── README.md
├── week07/
│   ├── evl_rt_2dof.cpp          # RT 제어 루프 (2-DOF, v1)
│   ├── load_2dof_common.h       # 파라미터 및 행렬 (Part 1)
│   ├── load_2dof_common2.h      # 파라미터 및 행렬 (Part 2)
│   └── two_dof_shm.h            # shm 구조체
├── week08/
│   ├── evl_rt_2dof.cpp          # RT 제어 루프 (2-DOF, v2)
│   ├── load_2dof_common3.h      # 통합 파라미터 헤더
│   └── two_dof_shm.h            # shm 구조체
└── viz/
    └── 2dof_viewer.py           # 2-DOF 상태 시각화 (공용)
```

**제어 루프 흐름 (`evl_rt_2dof.cpp`):**
```
EVL RT Thread
└─ 초기화: shm, 타이머, 제어기 파라미터

RT Loop (매 Ts마다):
  1. 상태 읽기: q[k] = [θ₁, θ₂]ᵀ, q̇[k] = [θ̇₁, θ̇₂]ᵀ
  2. M(q), C(q,q̇), g(q) 계산
  3. 제어 입력 계산: τ[k]
  4. 동역학 적분: RK4 or Euler
     q[k+1]  = q[k]  + Ts·q̇[k]
     q̇[k+1] = q̇[k] + Ts·M⁻¹(τ - Cq̇ - g)
  5. 상태 → shm write
  6. evl_sleep_until()
```

**`two_dof_shm.h` 핵심 구조:**
```cpp
struct TwoDofShm {
    double theta[2];      // [rad] 관절 각도
    double theta_dot[2];  // [rad/s] 관절 각속도
    double tau[2];        // [Nm] 제어 토크
    double t;             // [s] 타임스탬프
};
```

---

## 빌드 및 실행

```bash
# Week 07
cd 03_2dof_system_rt/week07
g++ -std=c++17 -O2 evl_rt_2dof.cpp -o evl_rt_2dof -levl -lrt -lpthread
sudo ./evl_rt_2dof

# Week 08
cd ../week08
g++ -std=c++17 -O2 evl_rt_2dof.cpp -o evl_rt_2dof -levl -lrt -lpthread
sudo ./evl_rt_2dof

# 시각화 (공용)
cd ../viz
python3 2dof_viewer.py
```

---

## 실제 적용 가능한 유스케이스

| 유스케이스 | 설명 |
|---|---|
| **2-링크 매니퓰레이터 제어** | 평면 2축 로봇팔의 관절 토크 제어 직접 적용 |
| **역진자 계열 확장** | 2-DOF 언더액추에이티드 시스템 (Acrobot, Pendubot) |
| **Computed Torque Control** | M, C, g 계산 구조를 피드포워드 보상에 활용 |
| **실시간 동역학 시뮬레이터** | EVL 루프를 HIL 플랜트로 사용 |

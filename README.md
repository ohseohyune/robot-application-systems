# Robot Application Systems — Lab Archive

> Practical lab implementations from the *Robot Application Systems* course.
> Covers shared memory IPC, real-time control with EVL, and MuJoCo simulation.

---

## Repository Structure

```
robot-application-systems/
│
├── 01_shared_memory_basics/       # Week 05 — IPC & SHM visualization
├── 02_single_pendulum_rt/         # Week 06 — 1-DOF real-time control (EVL)
├── 03_2dof_system_rt/             # Week 07~08 — 2-DOF real-time control (EVL)
│   ├── week07/
│   └── week08/
└── 04_mujoco_simulation/          # Week 11 — 2-DOF MuJoCo simulation
```

**왜 주제별로 묶었는가:**
- 7~8주차는 동일한 2-DOF 시스템의 점진적 확장 (`common` → `common3`)
- 11주차 MuJoCo 모델이 7~8주차 물리 모델을 시뮬레이션으로 재현
- 주차 번호보다 "무엇을 다루는가"가 포트폴리오에서 더 명확하게 읽힘

---

## Topics Covered

| 모듈 | 핵심 개념 | 사용 기술 |
|---|---|---|
| 01 | Shared Memory IPC, SHM | C++, POSIX shm |
| 02 | 1-DOF RT control, EVL framework | C++, Python, EVL RT |
| 03 | 2-DOF RT control, 제어기 설계 | C++, Python, EVL RT |
| 04 | MuJoCo 2-DOF simulation | Python, MuJoCo XML |

---

## Environment

| 항목 | 버전 |
|---|---|
| OS | Linux (RT kernel with EVL) |
| Compiler | g++ (C++17) |
| Python | 3.x |
| MuJoCo | (버전 기입) |
| ROS2 | (사용 시 기입) |

---

## Quick Start

```bash
git clone https://github.com/<username>/robot-application-systems.git
cd robot-application-systems

# 각 모듈의 README.md를 먼저 읽는다
cat 01_shared_memory_basics/README.md
```

# 04 — 2-DOF MuJoCo Simulation

> **Week 11** | Physics-Based Simulation of 2-DOF System using MuJoCo

---

## 목적 및 이론적 배경

### MuJoCo vs EVL RT 제어 루프 비교

```
[03 EVL RT]                        [04 MuJoCo]
─────────────────                  ─────────────────
직접 동역학 수치 적분               물리 엔진이 동역학 처리
M(q)q̈ = τ - Cq̇ - g              XML 모델 정의 → 자동 계산
수동 RK4/Euler 구현                contact, constraint 자동 처리
실시간 하드웨어 연동 목적           시뮬레이션 검증 목적
```

**MuJoCo를 쓰는 이유:**
- 마찰, 충돌, 관절 제한 등 실제 하드웨어 물리를 정확하게 모델링
- 03 모듈에서 설계한 제어기를 하드웨어 없이 검증
- 강화학습, MPC 등 고급 제어 알고리즘 테스트베드로 확장 가능

### `2dof.xml` — MuJoCo 모델 구조

```xml
<!-- MuJoCo XML 계층 구조 -->
<mujoco>
  <worldbody>
    <body name="link1">         <!-- 첫 번째 링크 -->
      <joint name="joint1"/>    <!-- θ₁ -->
      <body name="link2">       <!-- 두 번째 링크 -->
        <joint name="joint2"/>  <!-- θ₂ -->
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint1"/>     <!-- τ₁ -->
    <motor joint="joint2"/>     <!-- τ₂ -->
  </actuator>
</mujoco>
```

---

## 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Linux (Ubuntu 22.04) |
| Physics Engine | MuJoCo (버전: `MUJOCO_LOG.TXT` 참조) |
| Python | 3.x |
| 의존성 | `pip install mujoco` |
| 모델 포맷 | MJCF (MuJoCo XML Format) |

---

## 코드 구조

```
04_mujoco_simulation/
├── model/
│   └── 2dof.xml         # 2-DOF 시스템 물리 모델 (MJCF)
├── src/
│   └── move.py          # 제어 루프 및 시뮬레이션 실행
└── MUJOCO_LOG.TXT       # MuJoCo 실행 로그 (버전, 경고 등)
```

**`move.py` 제어 루프 흐름:**
```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("model/2dof.xml")
data  = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 1. 현재 상태 읽기
        q    = data.qpos   # [θ₁, θ₂]
        qdot = data.qvel   # [θ̇₁, θ̇₂]

        # 2. 제어 입력 계산
        data.ctrl[:] = controller(q, qdot)

        # 3. 물리 시뮬레이션 스텝
        mujoco.mj_step(model, data)

        # 4. 뷰어 업데이트
        viewer.sync()
```

---

## 빌드 및 실행

```bash
cd 04_mujoco_simulation

# MuJoCo 설치 (미설치 시)
pip install mujoco

# 시뮬레이션 실행
python3 src/move.py
```

> MuJoCo 뷰어 창이 열리며 2-DOF 시스템이 시각화된다.
> `MUJOCO_LOG.TXT`에서 실행 로그 및 버전 정보를 확인할 수 있다.

---

## 03 → 04 연결: 제어기 이식 흐름

```
[03 EVL RT] 에서 설계한 제어기
         ↓
파라미터 (Kp, Kd, K_lqr 등) 그대로 가져옴
         ↓
[04 MuJoCo] move.py의 controller() 함수에 적용
         ↓
실제 하드웨어 없이 동작 검증
         ↓
검증 완료 → 실제 하드웨어 탑재
```

---

## 실제 적용 가능한 유스케이스

| 유스케이스 | 설명 |
|---|---|
| **제어기 사전 검증** | 하드웨어 없이 제어기 안정성/성능 확인 |
| **강화학습 환경** | MuJoCo 환경을 Gym wrapper로 감싸 RL 학습 |
| **MPC 테스트베드** | 예측 모델로 MuJoCo 사용, CasADi와 연동 |
| **파라미터 최적화** | 시뮬레이션에서 게인 스윕 자동화 |
| **데이터 생성** | 실험 데이터 대신 시뮬레이션 데이터로 학습/분석 |

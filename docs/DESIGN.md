# 설계 요약

세부 결정의 근거는 `DECISIONS.md`에 있다.

## 구성
| 파일 | 역할 |
|---|---|
| `src/irobot_sim/app.py` | CLI, 실행 루프(뷰어 실시간 / 헤드리스), 종료 처리(창 닫기, Q 키, SIGINT), 요약 |
| `src/irobot_sim/state_machine.py` | 상태 머신. 매 스텝 `pre_step`(제어 입력) → [step_hook] → `mj_step` → `post_step`(감시·판정·전이) |
| `src/irobot_sim/planning.py` | 별도 `plan_data`에서 목표 샘플링, DLS IK, 유효성 검사 |
| `src/irobot_sim/checks.py` | 도달/복귀 판정, 안전 감시 |
| `src/irobot_sim/trajectory.py` | 관절공간 quintic 궤적(위치와 해석적 속도) |
| `src/irobot_sim/model.py` | 모델 로드, 인덱스, 초기 keyframe(유일한 상태 대입 지점), 제어 입력 |
| `config/sim.toml` | 모든 튜닝값 |

## 상태와 전이
`INIT → CHECK_HOME → SAMPLE_TARGET → REACH → VERIFY_REACH → RETURN_HOME → VERIFY_HOME → (대기 0.5 s) → SAMPLE_TARGET …`

- 실패하면 어느 상태에서든 `FAILED`로 간다. 물리가 멈추고 다음 목표는 만들지 않는다.
- 종료 요청이 오면 어느 상태에서든 `SHUTDOWN`으로 간다.
- 다음 목표는 `home_verified`(첫 사이클은 `check_home_ok`) 이후에만 생성한다.

## 제어
- 목표 관절값: IK로 미리 구한다.
- 기준 궤적: 관절공간 quintic이다. 최고속도가 0.4·vmax 이하가 되도록 1.5~6초로 정한다.
- 제어 입력: position 액추에이터에 `ctrl = q_ref + (kv/kp)·q̇_ref`를 준다(D-009). kp/kv는 j1–4가 4500/450, j5–7이 2000/200이다.
- 중력 보상: `gravcomp` + `actuatorgravcomp`이며, 힘 한계 87/12 Nm 안에서 처리된다.
- 복귀: 같은 제어기로 현재 실측 관절값에서 home까지 이동한다.

## 판정 (모두 연속 0.25초 유지해야 성공)
- 도달: 위치 오차 < 3 mm, 방향 오차 < 2°, 모든 |q̇| < 0.02 rad/s. 타임아웃 10초(실패 판정 전용).
- 복귀: 모든 |q − q_home| < 0.005 rad, |q̇| < 0.02 rad/s. 타임아웃 10초.

## 매 스텝 안전 감시
- `NUMERIC_DIVERGENCE`: NaN/Inf 또는 MuJoCo 경고. 자동 리셋은 꺼 둔다(D-007).
- `STATE_DISCONTINUITY`: 한 스텝의 |Δq| > 3·vmax·dt.
- `JOINT_LIMIT`: 한계 ± 0.01 rad를 벗어남.
- `UNEXPECTED_CONTACT`: 로봇이 관련된 모든 접촉.
- `TRACKING_ERROR`: REACH와 RETURN_HOME에서 |q − q_ref| > 0.022 rad.

## 목표 유효성 (확정 전에 검사)
- 샘플링 영역: 원통 r 0.30–0.70 m, φ ±90°, z 0.15–0.65 m. 어깨에서 0.85 m를 넘으면 기각한다.
- 자세: TCP가 아래를 향하고 yaw는 균일 샘플한다.
- IK 수렴 기준: 1 mm, 0.5°. home에서 시작하고, 실패하면 무작위 시작점으로 4회 재시도한다.
- 통과 조건: 관절 한계 여유 0.05 rad, 목표 자세 접촉 0, home→목표 직선 경로 25점 접촉 0, 6×7 야코비안 σ_min ≥ 0.05.
- 사이클당 최대 100회 시도한다. 사이클 k의 시드는 `[master, k]`이다.

# 결정 기록

각 항목: 결정, 근거, 날짜. 계획과 달라진 결정은 사용자 확인 후에만 추가한다.

## D-001 Python 3.12 작업공간 venv (2026-10-07)
- 결정: `/usr/bin/python3.12 -m venv ~/irobot_ws/.venv`. 패키지는 `requirements.lock`에 고정한다(mujoco 3.15.0, numpy 2.5.3, trimesh 5.1.1, pycollada 0.9.3, lxml 6.1.3, glfw 2.10.2).
- 근거: 시스템에 `python3.12-venv`가 이미 있다. 셸에서 활성화된 `drake-env`와 분리하기 위함이다.
- 모든 실행은 `scripts/run.sh`를 거친다. 이 래퍼는 PYTHONPATH, PYTHONHOME, AMENT_PREFIX_PATH, VIRTUAL_ENV, PYTHONSTARTUP을 제거하고 `PYTHONNOUSERSITE=1`로 실행한다.

## D-002 LD_LIBRARY_PATH 유지 (2026-10-07)
- 판정: **영향 없음.**
- 방법: `scripts/check_env.py`가 같은 프로브를 LD_LIBRARY_PATH(ROS Humble 경로 4개)가 있는 경우와 없는 경우로 각각 실행했다.
  - 프로브 동작: mujoco와 glfw를 import한 뒤 숨김 창, GL 컨텍스트, MjrContext를 만든다.
  - 비교 대상: `/proc/self/maps`에 기록된 libmujoco, libglfw, libGL*, libEGL*, libstdc++, libX11, libwayland-*, libdrm*, libgbm의 실제 경로.
- 결과: 두 경우의 라이브러리 경로 19개가 완전히 일치했다.
  - libglfw는 venv의 `glfw/wayland/libglfw.so`, libmujoco는 venv의 `libmujoco.so.3.15.0`를 썼다.
  - 나머지는 모두 `/usr/lib/x86_64-linux-gnu`에서 로드됐다.
  - GL: Mesa Intel UHD (CML GT2), OpenGL 4.6 Compatibility, Mesa 23.2.1.
- 결정: `run.sh`는 LD_LIBRARY_PATH를 그대로 둔다(`IROBOT_UNSET_LD=0`이 기본값). 문제가 생기면 `IROBOT_UNSET_LD=1`로 제거한 채 실행할 수 있다.

## D-003 GLFW 플랫폼 (2026-10-07)
- glfw 2.10.2 wheel은 Wayland 세션에서 네이티브 Wayland 백엔드(`PLATFORM_WAYLAND`)를 선택했다.
- 빈 장면 passive viewer는 5초 스모크 테스트에서 오류 없이 실행됐다.
- 창이 보이지 않거나 제목 표시줄이 없어 닫기 어려우면 `PYGLFW_LIBRARY_VARIANT=x11`로 XWayland를 쓴다.

## D-004 모델 출처와 Xacro 전개 (2026-10-07)
- 모델: franka_description tag 2.9.0, commit `7aeeddc449edf8d62b594f9e36a81da53e7796f9`, Apache-2.0. 출처는 `models/franka/SOURCE.md`에 기록했다.
- 도구: ROS2 Humble의 xacro 2.1.1로 2.9.0을 오류 없이 전개했다. 계획 §9의 대체안(이전 tag 사용, PyPI xacro 설치)은 쓰지 않았다.
- `$(find franka_description)` 해석: 작업공간 안의 ament index shim(`converted/ament_prefix`)으로 처리한다. `expand_xacro.sh`가 전개 전에 `get_package_share_directory`가 shim 경로를 돌려주는지 확인한다.
- 미해석 표현 검사는 XML 주석을 제외한다. 원본 매크로의 설명 주석에 `$(find ...)` 예시가 그대로 남기 때문이다.
- 가속도계 프레임 링크와 조인트 24개(링크 12개 + fixed 조인트 12개)는 URDF 후처리에서 제거했다. 질량과 형상이 없는 센서 표시용 프레임이라 시뮬레이션에 영향이 없다.

## D-005 contact exclude: fr3_link7 – fr3_hand (2026-10-07)
- 두 body는 질량과 형상이 없는 `fr3_link8`(fixed)을 사이에 두고 강체로 연결된다.
- MuJoCo의 부모-자식 필터는 바로 위아래 body끼리만 적용되므로, 이 쌍은 자동으로 제외되지 않는다.
- home 자세에서 두 충돌 메시가 21.3mm 겹친다(`mj_geomDistance` 기준). 강체 연결이라 어떤 자세에서도 이 겹침은 같다.
- 그래서 exclude로 지정했다. 그 밖의 제외 쌍은 없다. home 자세의 접촉 수는 0이다.

## D-006 메시 변환 세부 (2026-10-07)
- 시각 메시: DAE는 미터 단위다. 씬 그래프 변환을 적용하고 색상별로 합쳐 OBJ로 저장한다(링크당 1~6조각, 총 26조각). 색은 DAE 재질의 diffuse 값을 geom rgba로 옮긴다.
- OBJ에는 법선을 넣지 않는다. MuJoCo가 법선을 직접 계산하고, 법선 계산에 필요한 scipy를 따로 설치하지 않기 위함이다.
- 시각/충돌 bbox 중심 차이: 최대 2.5mm(link3). 기준값 10mm 이하로 PASS.
- 손가락 mimic: MuJoCo URDF 임포터가 equality(joint)를 직접 만들었다. 별도로 추가하지 않았다.
- 관절 힘 한계: 임포터가 URDF effort를 `actuatorfrcrange`로 옮겼다(87/12 Nm). 중력 보상(actuatorgravcomp)을 포함한 전체 액추에이터 힘이 이 한계 안에서 제한된다.
- `mj_saveLastXML`은 숫자를 유효숫자 6자리로 저장한다. 이 때문에 질량에 최대 4.65e-6 kg의 반올림 오차가 생겼다.
- 대응: 허용 오차는 그대로 두고, URDF를 직접 컴파일한 모델의 배정밀도 값으로 `<inertial>`을 다시 썼다. 그 결과 질량, 질량중심, 관성 오차가 모두 1e-14 수준이 됐다.

## D-007 MuJoCo 자동 리셋 비활성화 (2026-10-07)
- MuJoCo는 수치 발산(NaN 등)을 감지하면 기본 설정에서 `mj_resetData`로 자동 리셋한다. 이 동작은 리셋 금지 원칙을 어기고 `NUMERIC_DIVERGENCE` 감지도 가린다.
- 대응: `build_model.py`가 `<option><flag autoreset="disable"/></option>`을 넣는다. `Robot` 초기화 때 이 설정이 꺼져 있는지 확인하고, 켜져 있으면 `MODEL_LOAD` 실패로 처리한다.

## D-008 뷰어 종료 시 segfault 방지 (2026-10-07)
- 현상: `launch_passive`의 `with` 블록을 나오자마자 프로세스가 끝나면 exit 139(segfault)가 난다. Wayland와 x11 백엔드 모두 재현됐다.
- 원인: 뷰어 스레드가 정리를 끝내기 전에 인터프리터가 종료된다.
- 대응: 루프가 끝나면 `v.close()`를 호출하고, `is_running()`이 False가 될 때까지(최대 3초) 기다린 뒤 0.3초를 더 기다렸다가 반환한다. 이후 exit 0을 확인했다.

## D-009 속도 피드포워드와 추종 오차 임계값 (2026-10-07, 사용자 승인: 단계 1 확인 지점에서 A안)
- 문제: MuJoCo position 액추에이터의 힘은 `kp*(ctrl - q) - kv*qd`이다. 감쇠 항이 속도 오차가 아니라 속도 자체에 걸린다.
  - 그래서 `ctrl = q_ref` 방식에서는 이동 중에 (kv/kp)·q̇ = 0.1 s × 속도만큼 지연이 생긴다.
  - 1b에서는 최대 0.074 rad였고, 랜덤 목표 20개에서는 최대 0.204 rad(j7)였다.
  - 그 결과 임시 임계값 0.1 rad에서 정상 동작이 TRACKING_ERROR로 오판됐다(1c의 도달 불가·바닥 아래 케이스).
- 결정: 계획 §6의 제어식을 `ctrl = q_ref(t) + (kv/kp)·q̇_ref(t)`로 바꾼다.
  - 이렇게 하면 힘이 `kp*(q_ref - q) + kv*(q̇_ref - q̇)`가 되어 PD 추종이 된다.
  - q̇_ref는 quintic의 해석적 미분이다. kv/kp는 모델의 액추에이터 값에서 읽는다(모든 팔 관절 0.1).
  - 추종 오차 감시는 바꾸지 않았다(|q − q_ref|).
- 결과:
  - 1b 추종 오차: 최대 0.00167 rad. 도달 위치 오차 0.51 mm, 복귀 관절 오차 0.00007 rad.
  - 랜덤 목표 20개(실험): 최대 0.0041 rad.
- 임계값: 계획 공식 max(2×측정, 측정+0.02)를 적용해 `tracking_error_max = 0.022 rad`로 정했다. 랜덤 목표 실측 최대 대비 약 5배 여유다.
- 1c 재실행: 9/9가 기대 유형과 일치했다(도달 불가 → REACH_TIMEOUT, 바닥 아래 → UNEXPECTED_CONTACT 포함).

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

#!/usr/bin/env bash
# 작업공간 venv의 Python으로 실행하는 래퍼.
# 셸에 활성화된 다른 venv(drake-env)와 ROS2 Humble 환경(PYTHONPATH 등)이
# 작업공간 실행에 섞이지 않도록 관련 변수를 제거한 뒤 실행한다.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$WS/.venv/bin/python"

if [[ ! -x "$PY" ]]; then
    echo "[run.sh] venv가 없습니다: $PY" >&2
    exit 2
fi

# LD_LIBRARY_PATH 처리: docs/DECISIONS.md의 D-002 참조.
UNSET_LD="${IROBOT_UNSET_LD:-0}"   # 판정: 영향 없음 → 기본 유지(0)
unset_args=(-u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u VIRTUAL_ENV -u PYTHONSTARTUP)
if [[ "$UNSET_LD" == "1" ]]; then
    unset_args+=(-u LD_LIBRARY_PATH)
fi

exec env "${unset_args[@]}" \
    PYTHONNOUSERSITE=1 \
    PYTHONPATH="$WS/src" \
    IROBOT_WS="$WS" \
    "$PY" "$@"

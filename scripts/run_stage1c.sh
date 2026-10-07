#!/usr/bin/env bash
# 단계 1c: 실패 경로 주입 테스트. 각 케이스의 실제 실패 유형, 종료 코드, 반복 중단 여부를 표로 출력한다.
# 1차 통과 기준: 반복 중단(성공 사이클 이후 다음 목표 없음) + 실패 보고(failure 이벤트) + exit 1.
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
R="scripts/run.sh"
FT="--fixed-target 0.5,0.0,0.35"
COMMON="--headless --quiet --seed 1"

cases=(
  "unreachable|REACH_TIMEOUT|-m irobot_sim $COMMON --fixed-target 1.5,0,0.5 --skip-validation"
  "short_reach_timeout|REACH_TIMEOUT|-m irobot_sim $COMMON $FT --reach-timeout 0.3"
  "short_return_timeout|RETURN_TIMEOUT|-m irobot_sim $COMMON $FT --return-timeout 0.3"
  "below_floor|UNEXPECTED_CONTACT 또는 REACH_TIMEOUT|-m irobot_sim $COMMON --fixed-target 0.5,0,-0.05 --skip-validation"
  "no_target_attempts|TARGET_GENERATION|-m irobot_sim $COMMON --max-target-attempts 0"
  "missing_model|MODEL_LOAD|-m irobot_sim $COMMON --model models/franka/converted/mjcf/does_not_exist.xml"
  "inject_nan|NUMERIC_DIVERGENCE|scripts/fault_inject.py $COMMON $FT --inject nan --joint 2 --at-state REACH --delay 0.5"
  "inject_jump|STATE_DISCONTINUITY|scripts/fault_inject.py $COMMON $FT --inject jump --joint 2 --at-state REACH --delay 0.5"
  "inject_torque|TRACKING_ERROR|scripts/fault_inject.py $COMMON $FT --inject torque --joint 6 --tau 40 --at-state REACH --delay 0.5"
)

printf "%-22s %-36s %-22s %-5s %-10s %s\n" "케이스" "기대" "실제" "exit" "중단" "로그"
for c in "${cases[@]}"; do
  IFS="|" read -r name expect cmd <<< "$c"
  before=$(ls -1t logs/run_*.jsonl 2>/dev/null | head -1)
  $R $cmd > /dev/null 2>&1
  code=$?
  log=$(ls -1t logs/run_*.jsonl | head -1)
  [[ "$log" == "$before" ]] && log="(없음)"
  actual=$(grep -o '"kind": "failure", "t": [0-9.]*, "wall": [0-9.]*, "type": "[A-Z_]*"' "$log" 2>/dev/null | sed 's/.*"type": "//;s/"//' | head -1)
  # 반복 중단: 실패 이후 target 이벤트가 없어야 한다
  stopped=$($R - "$log" <<'PY'
import json, sys
ev = [json.loads(l) for l in open(sys.argv[1])]
fi = next((i for i, e in enumerate(ev) if e["kind"] == "failure"), None)
after = [e for e in ev[fi + 1:] if e["kind"] in ("target", "trace", "reach_verified", "home_verified")] if fi is not None else []
print("예" if fi is not None and not after else "아니오")
PY
)
  printf "%-22s %-36s %-22s %-5s %-10s %s\n" "$name" "$expect" "${actual:-?}" "$code" "$stopped" "$(basename "$log")"
done

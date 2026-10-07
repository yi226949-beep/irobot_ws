#!/usr/bin/env bash
# franka_description Xacro를 URDF로 전개한다(ROS2 Humble의 xacro를 서브셸에서 사용).
# $(find franka_description)는 작업공간 안의 ament index shim으로 해석한다.
#
# 사용법: scripts/expand_xacro.sh <출력 URDF 경로>
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:?출력 URDF 경로가 필요합니다}"
SRC="$WS/models/franka/source/franka_description"
SHIM="$WS/models/franka/converted/ament_prefix"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "[xacro] /opt/ros/humble 없음" >&2; exit 1; }
[[ -d "$SRC" ]] || { echo "[xacro] 원본 없음: $SRC (scripts/fetch_model.sh 먼저 실행)" >&2; exit 1; }

# ament index shim: 패키지 마커 + share 심볼릭 링크
mkdir -p "$SHIM/share/ament_index/resource_index/packages"
: > "$SHIM/share/ament_index/resource_index/packages/franka_description"
ln -sfn "../../../source/franka_description" "$SHIM/share/franka_description"

# 깨끗한 환경에서 ROS를 source 한다(작업공간 venv·drake-env와 섞지 않음).
env -i HOME="$HOME" PATH="/usr/bin:/bin" bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    export AMENT_PREFIX_PATH="'"$SHIM"':$AMENT_PREFIX_PATH"
    found=$(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g(\"franka_description\"))")
    echo "[xacro] get_package_share_directory(franka_description) = $found"
    if [[ "$found" != "'"$SHIM"'/share/franka_description" ]]; then
        echo "[xacro] shim 해석 실패" >&2; exit 1
    fi
    xacro "'"$SRC"'/robots/fr3/fr3.urdf.xacro" hand:=true ee_id:=franka_hand with_sc:=false -o "'"$OUT"'"
'

# 미해석 표현 검사: XML 주석(원본 매크로 설명문)은 제외하고 속성·텍스트만 본다.
/usr/bin/python3 - "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
bad = []
for el in ET.parse(sys.argv[1]).iter():
    for k, v in el.attrib.items():
        if "$(" in v or "${" in v:
            bad.append(f"<{el.tag} {k}='{v}'>")
    if el.text and ("$(" in el.text or "${" in el.text):
        bad.append(f"<{el.tag}> text")
if bad:
    print("[xacro] 미해석 표현이 남아 있습니다:", *bad[:10], sep="\n  ", file=sys.stderr)
    sys.exit(1)
print("[xacro] 미해석 $(...)/${...} 없음 (주석 제외)")
PY
echo "[xacro] 전개 완료: $OUT"

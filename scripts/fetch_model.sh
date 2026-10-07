#!/usr/bin/env bash
# 공식 franka_description 원본을 고정 tag로 가져와 models/franka/source/ 에 둔다.
# 원본은 수정하지 않는다. 이미 있으면 커밋 SHA만 검증한다.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_URL="https://github.com/frankarobotics/franka_description.git"
TAG="2.9.0"
EXPECTED_SHA="7aeeddc449edf8d62b594f9e36a81da53e7796f9"
DEST="$WS/models/franka/source/franka_description"
SOURCE_MD="$WS/models/franka/SOURCE.md"

if [[ -d "$DEST/.git" ]]; then
    echo "[fetch] 이미 존재: $DEST"
else
    echo "[fetch] clone $REPO_URL @ $TAG"
    git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$TAG" "$REPO_URL" "$DEST"
fi

SHA="$(git -C "$DEST" rev-parse HEAD)"
if [[ "$SHA" != "$EXPECTED_SHA" ]]; then
    echo "[fetch] 커밋 SHA 불일치: $SHA (기대값 $EXPECTED_SHA)" >&2
    exit 1
fi
if [[ -n "$(git -C "$DEST" status --porcelain)" ]]; then
    echo "[fetch] 원본에 수정 사항이 있습니다. 원본은 수정 금지입니다." >&2
    git -C "$DEST" status --short >&2
    exit 1
fi
echo "[fetch] SHA 검증 OK: $SHA"

if [[ ! -f "$SOURCE_MD" ]]; then
    cat > "$SOURCE_MD" <<EOF
# Franka 모델 출처

| 항목 | 값 |
|---|---|
| 저장소 | $REPO_URL |
| tag | $TAG |
| 커밋 SHA | $SHA |
| 라이선스 | Apache-2.0 (\`source/franka_description/LICENSE\`) |
| 가져온 날짜 | $(date +%Y-%m-%d) |
| 가져온 방법 | \`scripts/fetch_model.sh\` (git clone --depth 1 --branch $TAG) |
| 사용 모델 | FR3 + Franka Hand (\`robots/fr3/fr3.urdf.xacro\`, \`hand:=true ee_id:=franka_hand with_sc:=false\`) |

원본(\`source/\`)은 수정하지 않는다. 모든 변환은 \`scripts/build_model.py\`로 재현하며, 결과는 \`converted/\`에 둔다.

## 변환 재현 명령
\`\`\`bash
cd ~/irobot_ws
scripts/fetch_model.sh
scripts/run.sh scripts/build_model.py
scripts/run.sh scripts/inspect_model.py
\`\`\`
EOF
    echo "[fetch] SOURCE.md 작성"
fi
du -sh "$DEST"

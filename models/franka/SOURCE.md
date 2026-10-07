# Franka 모델 출처

| 항목 | 값 |
|---|---|
| 저장소 | https://github.com/frankarobotics/franka_description.git |
| tag | 2.9.0 |
| 커밋 SHA | 7aeeddc449edf8d62b594f9e36a81da53e7796f9 |
| 라이선스 | Apache-2.0 (`source/franka_description/LICENSE`) |
| 가져온 날짜 | 2026-10-07 |
| 가져온 방법 | `scripts/fetch_model.sh` (git clone --depth 1 --branch 2.9.0) |
| 사용 모델 | FR3 + Franka Hand (`robots/fr3/fr3.urdf.xacro`, `hand:=true ee_id:=franka_hand with_sc:=false`) |

원본(`source/`)은 수정하지 않는다. 모든 변환은 `scripts/build_model.py`로 재현하며, 결과는 `converted/`에 둔다.

## 변환 재현 명령
```bash
cd ~/irobot_ws
scripts/fetch_model.sh
scripts/run.sh scripts/build_model.py
scripts/run.sh scripts/inspect_model.py
```

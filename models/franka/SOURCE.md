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

## 라이선스와 변경 사항 고지
- 원본 라이선스: `models/franka/LICENSE`, `models/franka/NOTICE` (원본 저장소의 파일을 그대로 복사)
- `converted/`는 원본을 변환한 결과물이며, 다음과 같이 **변경되었다**.
  - `urdf/fr3_hand.urdf`: Xacro 전개 결과(내용 변경 없음)
  - `urdf/fr3_hand.mj.urdf`: 가속도계 프레임 링크 제거, 메시 경로를 OBJ/STL로 치환, MuJoCo 컴파일러 태그 추가
  - `meshes/visual/*.obj`: 원본 DAE의 씬 그래프 변환을 적용하고 색상별로 분할해 OBJ로 변환
  - `meshes/collision/*.stl`: 원본 STL 복사(이름만 변경)
  - `mjcf/*.xml`: MuJoCo 형식으로 변환. 관절 동역학, 중력 보상, 액추에이터, TCP site, contact exclude, home keyframe, 바닥, 목표 마커 추가
- 변환 근거는 `docs/DECISIONS.md`의 D-004~D-007에 있다.

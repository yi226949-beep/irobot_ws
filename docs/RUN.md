# 실행 방법

모든 명령은 `~/irobot_ws`에서 실행한다. Python은 항상 `scripts/run.sh`를 거쳐 실행한다. 이 래퍼가 작업공간 venv를 쓰고 ROS와 drake-env 환경 변수를 제거한다.

## 최종 실행 (랜덤 목표 무한 반복)
```bash
cd ~/irobot_ws
scripts/run.sh -m irobot_sim
```
- 종료 방법: 뷰어 창 닫기, 뷰어에서 `Q` 키, 터미널에서 `Ctrl+C`. 어느 방법이든 요약을 남기고 정상 종료한다(exit 0).
- 실패하면 반복이 멈추고 터미널과 로그에 실패 보고가 남는다. 물리도 멈추며 마커가 빨갛게 바뀐다. 창은 마지막 상태로 남아 있고, 위 세 방법 중 하나로 종료한다(exit 1).
- 마커 색: 노랑 = 목표로 이동 중, 초록 = 도달 확인, 파랑 = home 복귀 중, 빨강 = 실패.
- 주의: 뷰어의 Backspace(리셋)나 마우스 드래그(외력)는 실험 조건을 바꾼다. 리셋하면 `STATE_DISCONTINUITY` 실패로 멈춘다.
- 로그: `logs/run_<시각>.log`(사람이 읽는 용), `logs/run_<시각>.jsonl`(분석용). 뷰어 실행에서는 1시간에 약 40MB가 쌓인다.

## 재현과 검증용 옵션 (기본값은 모두 꺼져 있음)
| 옵션 | 용도 |
|---|---|
| `--seed S` | 마스터 시드를 고정한다(생략하면 무작위로 정하고 로그에 기록) |
| `--seed S --replay-cycle K --max-cycles 1` | 시드 S의 K번째 사이클 목표를 재현한다 |
| `--max-cycles N` | N 사이클 성공 후 종료한다(기본은 제한 없음) |
| `--headless` | 뷰어 없이 최대 속도로 실행한다 |
| `--fixed-target x,y,z` | 고정 목표 1점(단계 1 검증) |

## 모델 재생성
```bash
scripts/fetch_model.sh                 # 공식 원본 확보(tag 2.9.0, SHA 검증)
scripts/run.sh scripts/build_model.py  # 변환 파이프라인
scripts/run.sh scripts/inspect_model.py [--view]
```

## 검증 도구
```bash
scripts/run.sh scripts/check_env.py [--viewer]   # 환경 점검 / 빈 뷰어
scripts/run.sh scripts/analyze_log.py [로그]     # 순서·판정값·복귀 연속성·추종 오차
scripts/run.sh scripts/check_forbidden.py        # qpos/qvel 대입·리셋 호출·주입 분리 정적 점검
scripts/run_stage1c.sh                           # 실패 주입 9종
scripts/run.sh scripts/fault_inject.py --inject {nan,jump,torque,lowkp} ...   # 검증 전용
```

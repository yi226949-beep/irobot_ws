"""실행 로그(.jsonl) 검증.

검사 항목
  A. 순서: 다음 목표는 직전 사이클의 home 복귀 확인(첫 사이클은 초기 home 확인) 뒤에만 생성된다.
           각 사이클은 target → REACH → VERIFY_REACH → reach_verified → RETURN_HOME → VERIFY_HOME → home_verified 순서.
  B. 판정값: 도달/복귀 확인 시점의 오차가 설정 임계값 안.
  C. 복귀가 실제 제어로 이루어졌는지: RETURN_HOME 동안 관절값이 연속적으로 변하고(샘플 간 |Δq| ≤ vmax·Δt),
     충분한 수의 샘플이 있으며, 목표 자세 근처에서 home 근처로 이동했다.
  D. 추종 오차 통계: 관절별 최대 |q - q_ref| (REACH, RETURN_HOME).
  E. 요약/실패 이벤트.

사용법: scripts/run.sh scripts/analyze_log.py [logs/run_*.jsonl]   (생략 시 가장 최근 로그)
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from irobot_sim.config import load_config, resolve  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def main():
    cfg = load_config()
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        path = max(resolve(cfg, cfg.run.log_dir).glob("run_*.jsonl"), key=lambda p: p.stat().st_mtime)
    ev = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    print(f"== {path}  (이벤트 {len(ev)}개)")
    vmax = np.array(cfg.model.vmax)
    home = np.array(cfg.model.home)

    # A. 순서
    order_ok, problems = True, []
    last_ok_for_next = False   # 다음 목표 생성이 허용되는 상태인지
    cycle_seq = {}
    for e in ev:
        k = e["kind"]
        if k == "check_home_ok":
            last_ok_for_next = True
        elif k == "target":
            if not last_ok_for_next:
                order_ok = False
                problems.append(f"cycle {e['cycle']}: home 확인 전에 목표 생성")
            last_ok_for_next = False
            cycle_seq[e["cycle"]] = ["target"]
        elif k == "state" and e.get("cycle") in cycle_seq and e["to"] in ("REACH", "VERIFY_REACH", "RETURN_HOME", "VERIFY_HOME"):
            cycle_seq[e["cycle"]].append(e["to"])
        elif k in ("reach_verified", "home_verified"):
            cycle_seq.setdefault(e["cycle"], []).append(k)
            if k == "home_verified":
                last_ok_for_next = True
    expected = ["target", "REACH", "VERIFY_REACH", "reach_verified", "RETURN_HOME", "VERIFY_HOME", "home_verified"]
    completed = [c for c, s in cycle_seq.items() if s == expected]
    for c, s in cycle_seq.items():
        if s != expected and s != expected[:len(s)]:
            order_ok = False
            problems.append(f"cycle {c}: 순서 {s}")
    print("== A. 순서")
    check("home 확인 전에는 다음 목표를 생성하지 않음 + 사이클 내부 순서", order_ok, "; ".join(problems[:5]))
    print(f"  완료된 사이클 {len(completed)}개 / 목표가 생성된 사이클 {len(cycle_seq)}개")

    # B. 판정값
    print("== B. 판정값")
    rv = [e for e in ev if e["kind"] == "reach_verified"]
    hv = [e for e in ev if e["kind"] == "home_verified"]
    if rv:
        pe = max(e["pos_err"] for e in rv)
        oe = max(e["ori_err_deg"] for e in rv)
        check("도달 확인 시 위치오차 < pos_tol", pe < cfg.reach.pos_tol, f"최대 {pe*1000:.3f} mm (기준 {cfg.reach.pos_tol*1000:.1f} mm)")
        check("도달 확인 시 방향오차 < ori_tol", oe < cfg.reach.ori_tol_deg, f"최대 {oe:.3f}° (기준 {cfg.reach.ori_tol_deg}°)")
        rt = [e["reach_time"] for e in rv]
        print(f"  도달 소요 시간: 평균 {np.mean(rt):.2f} s, 최대 {np.max(rt):.2f} s (타임아웃 {cfg.reach.timeout} s)")
    if hv:
        je = max(e["joint_err_max"] for e in hv)
        check("복귀 확인 시 관절오차 < joint_tol", je < cfg.home.joint_tol, f"최대 {je:.5f} rad (기준 {cfg.home.joint_tol})")
        rt = [e["return_time"] for e in hv]
        print(f"  복귀 소요 시간: 평균 {np.mean(rt):.2f} s, 최대 {np.max(rt):.2f} s (타임아웃 {cfg.home.timeout} s)")

    # C. 복귀 연속성(실제 제어)
    print("== C. 복귀가 실제 제어로 이루어졌는지")
    tr = [e for e in ev if e["kind"] == "trace"]
    cont_ok, n_checked, worst_ratio = True, 0, 0.0
    min_samples_ok = True
    moved_ok = True
    for c in completed:
        ret = [e for e in tr if e["cycle"] == c and e["state"] == "RETURN_HOME"]
        dur = next(e for e in ev if e["kind"] == "state" and e.get("cycle") == c and e["to"] == "RETURN_HOME").get("duration")
        need = int(0.9 * dur * cfg.run.trace_hz) if dur else 1
        if len(ret) < need:
            min_samples_ok = False
        for a, b in zip(ret, ret[1:]):
            dt = b["t"] - a["t"]
            dq = np.abs(np.array(b["q"]) - np.array(a["q"]))
            ratio = float(np.max(dq / (vmax * dt + 1e-9)))
            worst_ratio = max(worst_ratio, ratio)
            n_checked += 1
            if ratio > 1.05:
                cont_ok = False
        if ret:
            d0 = np.max(np.abs(np.array(ret[0]["q"]) - home))
            d1 = np.max(np.abs(np.array(ret[-1]["q"]) - home))
            if not d1 < d0:
                moved_ok = False
    check("복귀 중 샘플 수 충분(≥ 0.9 × 궤적시간 × trace_hz)", min_samples_ok and bool(completed))
    check("복귀 중 관절값 연속(샘플 간 |Δq| ≤ vmax·Δt)", cont_ok and n_checked > 0,
          f"검사 {n_checked}구간, 최대 |Δq|/(vmax·Δt) = {worst_ratio:.3f}")
    check("복귀 중 목표 자세 → home으로 이동", moved_ok and bool(completed))

    # D. 추종 오차
    print("== D. 추종 오차 (REACH + RETURN_HOME)")
    tm = [np.array(e["track_max"]) for e in ev if e["kind"] in ("reach_verified", "home_verified")]
    if tm:
        mx = np.max(np.vstack(tm), axis=0)
        print(f"  관절별 최대 |q - q_ref| [rad]: {np.round(mx, 5).tolist()}")
        print(f"  전체 최대: {mx.max():.5f} rad (현재 임계값 {cfg.safety.tracking_error_max})")

    # E. 요약 / 실패
    print("== E. 요약 / 실패")
    fails = [e for e in ev if e["kind"] == "failure"]
    summ = [e for e in ev if e["kind"] == "summary"]
    for f in fails:
        print(f"  실패: {f['type']} (cycle {f.get('cycle')}, state {f.get('state')}) detail={f.get('detail')}")
    if summ:
        s = summ[-1]
        print(f"  요약: 사이클 {s['cycles_started']}, 성공 {s['cycles_succeeded']}, 실패 {s['failure']}, "
              f"종료 사유 {s['shutdown_reason']}, 시드 {s['master_seed']}")
    check("요약 이벤트 기록됨", bool(summ))

    print("== 결과:", "PASS" if all(results) else "FAIL")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

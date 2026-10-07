"""검증 전용 실패 주입 진입점 (계획 §4.1). src/irobot_sim은 이 파일을 import하지 않는다.

irobot_sim의 기본 CLI 옵션을 모두 받고, 여기에만 있는 --inject 옵션으로 step_hook을 만들어
irobot_sim.app.run(args, step_hook=hook)을 호출한다.

주입 종류
  nan    : data.qvel[관절] = NaN (1회)                     → 기대 NUMERIC_DIVERGENCE
  jump   : data.qpos[관절] += --jump (1회)                  → 기대 STATE_DISCONTINUITY
  torque : data.qfrc_applied[관절] = --tau (주입 구간 동안)  → 기대 TRACKING_ERROR  (qpos 대입 없음)
  lowkp  : 시작 시 액추에이터 kp/kv를 --kp-scale 배로 축소   → 기대 TRACKING_ERROR

사용 예:
  scripts/run.sh scripts/fault_inject.py --inject torque --joint 6 --tau 40 --at-state REACH \
      --fixed-target 0.5,0,0.35 --headless
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from irobot_sim.app import build_arg_parser, run  # noqa: E402


def make_hook(a):
    j = a.joint - 1
    state = {"fired": False, "lowkp_done": False}

    def hook(model, data, ctx):
        if a.inject == "lowkp":
            if not state["lowkp_done"]:
                acts = [i for i in range(model.nu) if model.actuator_trnid[i, 0] < 7]
                for i in acts:
                    model.actuator_gainprm[i, 0] *= a.kp_scale
                    model.actuator_biasprm[i, 1] *= a.kp_scale
                    model.actuator_biasprm[i, 2] *= a.kp_scale
                state["lowkp_done"] = True
                print(f"[inject] lowkp: 팔 액추에이터 kp/kv × {a.kp_scale}", flush=True)
            return
        active = ctx.state == a.at_state and ctx.time_in_state >= a.delay
        dof, qadr = ctx.arm_dofadr[j], ctx.arm_qadr[j]
        if a.inject == "torque":
            if active and (a.duration is None or ctx.time_in_state < a.delay + a.duration):
                if not state["fired"]:
                    print(f"[inject] torque: joint{a.joint}에 {a.tau} Nm (qfrc_applied) 시작, "
                          f"{ctx.state} +{ctx.time_in_state:.3f}s", flush=True)
                    state["fired"] = True
                data.qfrc_applied[dof] = a.tau
            else:
                data.qfrc_applied[dof] = 0.0
            return
        if active and not state["fired"]:
            state["fired"] = True
            if a.inject == "nan":
                data.qvel[dof] = float("nan")
            elif a.inject == "jump":
                data.qpos[qadr] += a.jump
            print(f"[inject] {a.inject}: joint{a.joint}, {ctx.state} +{ctx.time_in_state:.3f}s", flush=True)

    hook.__name__ = f"inject_{a.inject}"
    return hook


def main():
    ap = build_arg_parser()
    ap.prog = "fault_inject"
    g = ap.add_argument_group("실패 주입(검증 전용)")
    g.add_argument("--inject", required=True, choices=["nan", "jump", "torque", "lowkp"])
    g.add_argument("--joint", type=int, default=6, help="대상 관절 번호(1~7)")
    g.add_argument("--at-state", default="REACH", help="주입할 상태")
    g.add_argument("--delay", type=float, default=0.5, help="상태 진입 후 주입까지 시간 [s]")
    g.add_argument("--duration", type=float, default=None, help="torque 주입 지속 시간 [s] (기본: 상태 동안 계속)")
    g.add_argument("--tau", type=float, default=40.0, help="torque 크기 [Nm]")
    g.add_argument("--jump", type=float, default=0.3, help="jump 크기 [rad]")
    g.add_argument("--kp-scale", type=float, default=0.02, help="lowkp 축소 배율")
    a = ap.parse_args()
    code, _, _ = run(a, step_hook=make_hook(a))
    return code


if __name__ == "__main__":
    sys.exit(main())

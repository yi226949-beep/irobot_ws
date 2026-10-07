"""변환된 FR3 모델 검증 (단계 1a).

공식 YAML(joint_limits, inertials, kinematics)과 MuJoCo 모델을 비교하고,
home 자세의 FK·접촉을 출력한다.

사용법:
    scripts/run.sh scripts/inspect_model.py           # 검증 출력
    scripts/run.sh scripts/inspect_model.py --view    # + 뷰어(home 자세를 액추에이터로 유지)
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_model import load_yaml_simple  # noqa: E402

WS = Path(os.environ.get("IROBOT_WS", Path(__file__).resolve().parents[1]))
SRC = WS / "models/franka/source/franka_description"
SCENE = WS / "models/franka/converted/mjcf/scene.xml"
ARM = [f"fr3_joint{i}" for i in range(1, 8)]

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, float))
    return m.reshape(3, 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    name = lambda t, i: mujoco.mj_id2name(m, t, i)  # noqa: E731
    J, B, A, G = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_GEOM

    print(f"== 모델: {SCENE.relative_to(WS)}  (MuJoCo {mujoco.__version__})")
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody} ngeom={m.ngeom} nmesh={m.nmesh} neq={m.neq} "
          f"timestep={m.opt.timestep} integrator={mujoco.mjtIntegrator(m.opt.integrator).name}")

    print("== 관절")
    for j in range(m.njnt):
        print(f"  {j}: {name(J, j):18s} type={mujoco.mjtJoint(m.jnt_type[j]).name:10s} "
              f"range=[{m.jnt_range[j, 0]:+.4f}, {m.jnt_range[j, 1]:+.4f}] armature={m.dof_armature[m.jnt_dofadr[j]]:.4f} "
              f"frictionloss={m.dof_frictionloss[m.jnt_dofadr[j]]:.3f}")
    print("== 액추에이터")
    for a in range(m.nu):
        print(f"  {a}: {name(A, a):12s} → {name(J, m.actuator_trnid[a, 0]):18s} kp={m.actuator_gainprm[a, 0]:g} "
              f"kv={-m.actuator_biasprm[a, 2]:g} ctrl=[{m.actuator_ctrlrange[a, 0]:+.4f}, {m.actuator_ctrlrange[a, 1]:+.4f}] "
              f"force=[{m.actuator_forcerange[a, 0]:+g}, {m.actuator_forcerange[a, 1]:+g}]")

    print("== 검증: 개수")
    check("관절 9개(팔 7 + 손가락 2)", m.njnt == 9, f"njnt={m.njnt}")
    check("액추에이터 8개(팔 7 + 손가락 1)", m.nu == 8, f"nu={m.nu}")

    if SRC.is_dir():
        print("== 검증: 관절 한계 vs joint_limits.yaml")
        lim = load_yaml_simple(SRC / "robots/fr3/joint_limits.yaml")
        worst = 0.0
        for jn in ARM:
            j = m.joint(jn).id
            y = lim[jn.replace("fr3_", "")]["limit"]
            worst = max(worst, abs(m.jnt_range[j, 0] - y["lower"]), abs(m.jnt_range[j, 1] - y["upper"]))
        check("관절 한계 일치(|차이| ≤ 1e-4)", worst <= 1e-4, f"최대 차이 {worst:.2e}")

        print("== 검증: 질량·관성 vs inertials.yaml")
        iner = load_yaml_simple(SRC / "robots/fr3/inertials.yaml")
        hand_iner = load_yaml_simple(SRC / "end_effectors/franka_hand/inertials.yaml")
        mass_err = com_err = inertia_rel = 0.0
        for k in ("hand", "leftfinger", "rightfinger"):
            diff = abs(m.body_mass[m.body(f"fr3_{k}").id] - hand_iner[k]["mass"])
            mass_err = max(mass_err, diff)
        for k, v in iner.items():
            b = m.body(f"fr3_{k}").id
            mass_err = max(mass_err, abs(m.body_mass[b] - v["mass"]))
            com = np.array([float(x) for x in v["origin"]["xyz"].split()])
            com_err = max(com_err, np.linalg.norm(m.body_ipos[b] - com))
            I = v["inertia"]
            Iy = np.array([[I["xx"], I["xy"], I["xz"]], [I["xy"], I["yy"], I["yz"]], [I["xz"], I["yz"], I["zz"]]])
            R = quat_to_mat(m.body_iquat[b])
            Im = R @ np.diag(m.body_inertia[b]) @ R.T
            rel = np.linalg.norm(Im - Iy) / np.linalg.norm(Iy)
            inertia_rel = max(inertia_rel, rel)
            ev = np.sort(np.linalg.eigvalsh(Iy))
            if rel > 1e-3:
                print(f"    {k}: 관성 상대오차 {rel:.2e}, yaml 고유값 {ev}, mujoco diag {np.sort(m.body_inertia[b])}")
        check("링크 질량 일치(팔 8 + 핸드·손가락 3, ≤1e-6 kg)", mass_err <= 1e-6, f"최대 차이 {mass_err:.2e} kg")
        check("링크 질량중심 일치(≤1e-6 m)", com_err <= 1e-6, f"최대 차이 {com_err:.2e} m")
        check("링크 관성 텐서 일치(상대오차 ≤ 1e-3)", inertia_rel <= 1e-3, f"최대 상대오차 {inertia_rel:.2e}")
        robot_bodies = [b for b in range(m.nbody) if (name(B, b) or "").startswith("fr3_")]
        total = sum(m.body_mass[b] for b in robot_bodies)
        yaml_total = sum(v["mass"] for v in iner.values())
        print(f"  로봇 총질량 {total:.4f} kg (팔 yaml 합 {yaml_total:.4f} kg + 핸드·손가락)")

        print("== 검증: 관절 원점 vs kinematics.yaml")
        kin = load_yaml_simple(SRC / "robots/fr3/kinematics.yaml")
        pos_err = rot_err = 0.0
        for i in range(1, 8):
            k = kin[f"joint{i}"]["kinematic"]
            b = m.body(f"fr3_link{i}").id
            pos_err = max(pos_err, np.linalg.norm(m.body_pos[b] - [k["x"], k["y"], k["z"]]))
            Rm, Ry = quat_to_mat(m.body_quat[b]), rpy_to_mat(k["roll"], k["pitch"], k["yaw"])
            rot_err = max(rot_err, np.linalg.norm(Rm - Ry))
        k8 = kin["joint8"]["kinematic"]
        pos_err = max(pos_err, np.linalg.norm(m.body_pos[m.body("fr3_link8").id] - [k8["x"], k8["y"], k8["z"]]))
        check("관절 원점 위치 일치(≤1e-6 m)", pos_err <= 1e-6, f"최대 차이 {pos_err:.2e} m")
        check("관절 원점 자세 일치(‖ΔR‖ ≤ 1e-4)", rot_err <= 1e-4, f"최대 차이 {rot_err:.2e}")
    else:
        print("== 검증: 공식 YAML 비교 — [SKIP] 원본이 없습니다(models/franka/source/).")
        print("  원본과 비교하려면 먼저 scripts/fetch_model.sh 를 실행하세요(git만 필요, ROS 불필요).")
    check("TCP 오프셋 = 0.1034 m", abs(m.body_pos[m.body("fr3_hand_tcp").id][2] - 0.1034) < 1e-9)

    print("== 충돌 geom (body별)")
    per_body = {}
    for g in range(m.ngeom):
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            per_body.setdefault(name(B, m.geom_bodyid[g]), []).append(mujoco.mjtGeom(m.geom_type[g]).name.replace("mjGEOM_", ""))
    for bn, gs in per_body.items():
        print(f"  {bn:16s} {len(gs)}개: {', '.join(gs)}")
    check("충돌 geom: 링크 0–7·핸드 메시 1개씩, 손가락 박스 4개씩, 바닥 1개",
          all(len(per_body.get(f"fr3_link{i}", [])) == 1 for i in range(8))
          and len(per_body.get("fr3_hand", [])) == 1
          and len(per_body.get("fr3_leftfinger", [])) == 4 and len(per_body.get("fr3_rightfinger", [])) == 4)

    print("== home 자세 FK / 접촉")
    key = m.key("home").id
    d.qpos[:] = m.key_qpos[key]
    d.ctrl[:] = m.key_ctrl[key]
    mujoco.mj_forward(m, d)
    tcp = d.site("tcp")
    zaxis = tcp.xmat.reshape(3, 3)[:, 2]
    print(f"  home qpos = {np.round(d.qpos[:7], 4).tolist()}")
    print(f"  TCP 위치 = {np.round(tcp.xpos, 4).tolist()} m")
    print(f"  TCP z축  = {np.round(zaxis, 4).tolist()} (아래 방향이면 [0,0,-1])")
    print(f"  플랜지(link8) 위치 = {np.round(d.body('fr3_link8').xpos, 4).tolist()} m")
    for c in d.contact[:d.ncon]:
        print(f"    접촉: {name(B, m.geom_bodyid[c.geom1])} – {name(B, m.geom_bodyid[c.geom2])} dist={c.dist:+.4f}")
    check("home 자세 접촉 0", d.ncon == 0, f"ncon={d.ncon}")
    in_lim = all(m.jnt_range[j, 0] <= d.qpos[m.jnt_qposadr[j]] <= m.jnt_range[j, 1] for j in range(m.njnt))
    check("home 자세가 관절 한계 안", in_lim)

    print("== 요약")
    print(f"  {sum(results)}/{len(results)} PASS" + ("" if all(results) else "  ← FAIL 있음"))

    if args.view:
        print("뷰어: home 자세를 position 액추에이터로 유지합니다(창을 닫으면 종료).")
        with mujoco.viewer.launch_passive(m, d) as v:
            while v.is_running():
                t0 = time.time()
                for _ in range(8):
                    mujoco.mj_step(m, d)
                v.sync()
                time.sleep(max(0.0, 8 * m.opt.timestep - (time.time() - t0)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

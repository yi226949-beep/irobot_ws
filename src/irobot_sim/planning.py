"""목표 샘플링, IK 사전 계산, 유효성 검사.

모든 계산은 시뮬레이션 데이터와 분리된 plan_data(별도 MjData)에서 한다.
이 파일에서 qpos 대입은 plan_data에 대해서만 허용된다(check_forbidden.py R1-b).
"""

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np


def target_rotation(yaw):
    """TCP z축이 아래(-z)를 향하고 z축 회전이 yaw인 자세: Rz(yaw) @ Rx(pi)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def rotation_error(R_target, R):
    """월드 좌표계 회전 오차 벡터(축 × 각도)와 각도."""
    R_err = R_target @ R.T
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R_err.flatten())
    vel = np.zeros(3)
    mujoco.mju_quat2Vel(vel, quat, 1.0)
    return vel, float(np.linalg.norm(vel))


@dataclass
class Target:
    cycle: int
    seed: list
    pos: np.ndarray
    yaw: float
    R: np.ndarray
    q_goal: np.ndarray
    attempts: int = 0
    rejects: dict = field(default_factory=dict)
    validated: bool = True
    ik_pos_err: float = 0.0
    ik_ori_err: float = 0.0
    sigma_min: float = 0.0


class TargetGenerationError(RuntimeError):
    def __init__(self, msg, rejects, attempts):
        super().__init__(msg)
        self.rejects = rejects
        self.attempts = attempts


class Planner:
    def __init__(self, robot, cfg):
        self.robot = robot
        self.m = robot.model
        self.plan_data = mujoco.MjData(self.m)
        # 런타임 검사: 플래너용 데이터가 시뮬레이션 데이터와 분리되어 있어야 한다.
        assert self.plan_data is not robot.data, "plan_data가 시뮬레이션 data와 같은 객체입니다"
        for f in ("qpos", "qvel", "ctrl", "act"):
            assert not np.shares_memory(getattr(self.plan_data, f), getattr(robot.data, f)), \
                f"plan_data.{f}가 시뮬레이션 data와 메모리를 공유합니다"
        self.tc, self.ik = cfg.target, cfg.ik
        self.home = robot.home
        self.lo, self.hi = robot.arm_range[:, 0], robot.arm_range[:, 1]
        self.robot_bodies = robot.robot_bodies
        finger_qadr = [self.m.jnt_qposadr[self.m.actuator_trnid[robot.finger_act, 0]]]
        eq = [j for j in range(self.m.njnt) if self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE]
        self.finger_qadr = np.array(sorted(set(finger_qadr + [self.m.jnt_qposadr[j] for j in eq])))
        self.jacp = np.zeros((3, self.m.nv))
        self.jacr = np.zeros((3, self.m.nv))
        self.home_yaw = yaw_of(self.fk(self.home)[1])

    # --- plan_data 상의 기구학 ---
    def _set(self, q):
        self.plan_data.qpos[self.robot.arm_qadr] = q
        self.plan_data.qpos[self.finger_qadr] = self.robot.finger_open

    def fk(self, q):
        self._set(q)
        mujoco.mj_kinematics(self.m, self.plan_data)
        s = self.robot.tcp_site
        return self.plan_data.site_xpos[s].copy(), self.plan_data.site_xmat[s].reshape(3, 3).copy()

    def jacobian(self, q):
        self._set(q)
        mujoco.mj_kinematics(self.m, self.plan_data)
        mujoco.mj_comPos(self.m, self.plan_data)
        mujoco.mj_jacSite(self.m, self.plan_data, self.jacp, self.jacr, self.robot.tcp_site)
        cols = self.robot.arm_dofadr
        return np.vstack([self.jacp[:, cols], self.jacr[:, cols]])

    def contacts(self, q):
        """q에서 로봇이 관련된 접촉 목록 [(body1, body2, dist)]."""
        self._set(q)
        mujoco.mj_fwdPosition(self.m, self.plan_data)
        out = []
        for c in self.plan_data.contact[:self.plan_data.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if b1 in self.robot_bodies or b2 in self.robot_bodies:
                out.append((self.robot.body_name(b1), self.robot.body_name(b2), float(c.dist)))
        return out

    # --- IK (감쇠 최소제곱 + 영공간 home 유지) ---
    def solve_ik(self, p_t, R_t, q_init):
        ik = self.ik
        q = np.clip(np.asarray(q_init, float).copy(), self.lo + 1e-4, self.hi - 1e-4)
        lam2 = ik.damping ** 2
        ori_tol = math.radians(ik.ori_tol_deg)
        best = (q.copy(), math.inf, math.inf)
        for _ in range(ik.max_iters):
            p, R = self.fk(q)
            e_p = p_t - p
            e_o, ang = rotation_error(R_t, R)
            perr = float(np.linalg.norm(e_p))
            if perr + 0.1 * ang < best[1] + 0.1 * best[2]:
                best = (q.copy(), perr, ang)
            if perr < ik.pos_tol and ang < ori_tol:
                return q, True, perr, ang
            J = self.jacobian(q)
            e = np.concatenate([e_p, e_o])
            JJt = J @ J.T + lam2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            N = np.eye(7) - J.T @ np.linalg.solve(JJt, J)
            dq += N @ (ik.nullspace_gain * (self.home - q))
            n = np.linalg.norm(dq)
            if n > ik.max_step:
                dq *= ik.max_step / n
            q = np.clip(q + dq, self.lo + 1e-4, self.hi - 1e-4)
        return best[0], False, best[1], best[2]

    def sigma_min(self, q):
        return float(np.linalg.svd(self.jacobian(q), compute_uv=False)[-1])

    # --- 목표 생성 ---
    def _ik_with_restarts(self, p, R, rng):
        q, ok, pe, oe = self.solve_ik(p, R, self.home)
        for _ in range(self.ik.restarts):
            if ok:
                break
            q0 = rng.uniform(self.lo + 0.1, self.hi - 0.1)
            q, ok, pe, oe = self.solve_ik(p, R, q0)
        return q, ok, pe, oe

    def validate(self, q):
        """유효성 검사. 통과하면 (None, σ_min), 실패하면 (사유, 세부)."""
        tc = self.tc
        if np.any(q < self.lo + tc.joint_margin) or np.any(q > self.hi - tc.joint_margin):
            return "joint_margin", None
        if self.contacts(q):
            return "contact_goal", None
        for s in np.linspace(0.0, 1.0, tc.path_samples):
            if self.contacts(self.home + s * (q - self.home)):
                return "contact_path", None
        sm = self.sigma_min(q)
        if sm < tc.sigma_min:
            return "singular", sm
        return None, sm

    def sample(self, cycle, master_seed):
        """사이클 시드로 유효한 랜덤 목표를 찾는다. 실패하면 TargetGenerationError."""
        tc = self.tc
        seed = [int(master_seed), int(cycle)]
        rng = np.random.default_rng(seed)
        rejects = {}
        shoulder = np.array(tc.shoulder)
        for attempt in range(1, tc.max_attempts + 1):
            r = rng.uniform(*tc.r)
            phi = math.radians(rng.uniform(*tc.phi_deg))
            z = rng.uniform(*tc.z)
            yaw = rng.uniform(-math.pi, math.pi)
            p = np.array([r * math.cos(phi), r * math.sin(phi), z])
            R = target_rotation(yaw)
            if np.linalg.norm(p - shoulder) > tc.max_shoulder_dist:
                rejects["shoulder_dist"] = rejects.get("shoulder_dist", 0) + 1
                continue
            q, ok, pe, oe = self._ik_with_restarts(p, R, rng)
            if not ok:
                rejects["ik_fail"] = rejects.get("ik_fail", 0) + 1
                continue
            reason, sm = self.validate(q)
            if reason:
                rejects[reason] = rejects.get(reason, 0) + 1
                continue
            return Target(cycle, seed, p, yaw, R, q, attempt, rejects, True, pe, oe, sm)
        raise TargetGenerationError(f"{tc.max_attempts}회 안에 유효한 목표를 찾지 못함", rejects, tc.max_attempts)

    def fixed(self, cycle, pos, yaw=None, skip_validation=False):
        """검증용 고정 목표. skip_validation이면 IK가 수렴하지 않아도 최선해로 진행한다."""
        yaw = self.home_yaw if yaw is None else yaw
        p, R = np.asarray(pos, float), target_rotation(yaw)
        rng = np.random.default_rng([0, cycle])
        q, ok, pe, oe = self._ik_with_restarts(p, R, rng)
        if skip_validation:
            return Target(cycle, [0, cycle], p, yaw, R, q, 1, {}, False, pe, oe, self.sigma_min(q))
        if not ok:
            raise TargetGenerationError("고정 목표의 IK가 수렴하지 않음", {"ik_fail": 1}, 1)
        reason, sm = self.validate(q)
        if reason:
            raise TargetGenerationError(f"고정 목표 유효성 실패: {reason}", {reason: 1}, 1)
        return Target(cycle, [0, cycle], p, yaw, R, q, 1, {}, True, pe, oe, sm)

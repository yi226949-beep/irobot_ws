"""도달·복귀 판정과 매 스텝 안전 감시. 데이터를 읽기만 한다."""

import math

import mujoco
import numpy as np

from .planning import rotation_error


class Hold:
    """조건이 연속으로 유지된 시뮬레이션 시간을 센다. 조건이 깨지면 0으로 돌아간다."""

    def __init__(self, need):
        self.need = float(need)
        self.t = 0.0

    def update(self, ok, dt):
        self.t = self.t + dt if ok else 0.0
        return self.t >= self.need - 1e-9


def reach_metrics(robot, target):
    p, R = robot.tcp_pose()
    _, ang = rotation_error(target.R, R)
    return {
        "pos_err": float(np.linalg.norm(target.pos - p)),
        "ori_err_deg": math.degrees(ang),
        "qd_max": float(np.max(np.abs(robot.qd()))),
        "tcp": p,
    }


def home_metrics(robot):
    return {
        "joint_err_max": float(np.max(np.abs(robot.q() - robot.home))),
        "qd_max": float(np.max(np.abs(robot.qd()))),
    }


WARNINGS = (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
            mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADCTRL)


class SafetyMonitor:
    """매 스텝 공통 감시. 위반이 있으면 (실패 유형, 세부 dict)를 돌려준다."""

    def __init__(self, robot, cfg):
        self.r = robot
        self.s = cfg.safety
        self.q_prev = None
        self.step_limit = robot.vmax * robot.dt * self.s.discontinuity_factor

    def reset_prev(self):
        self.q_prev = self.r.q()

    def check(self, q_ref=None):
        r, m, d = self.r, self.r.model, self.r.data
        # 1) 수치 발산
        bad_arrays = [n for n in ("qpos", "qvel", "qacc") if not np.all(np.isfinite(getattr(d, n)))]
        warns = {mujoco.mjtWarning(w).name: int(d.warning[w].number) for w in WARNINGS if d.warning[w].number}
        if bad_arrays or warns:
            return "NUMERIC_DIVERGENCE", {"non_finite": bad_arrays, "warnings": warns}

        q = r.q()
        # 2) 상태 불연속(외부 대입·리셋 감지)
        if self.q_prev is not None:
            jump = np.abs(q - self.q_prev)
            if np.any(jump > self.step_limit):
                j = int(np.argmax(jump / self.step_limit))
                self.q_prev = q
                return "STATE_DISCONTINUITY", {"joint": j + 1, "dq": float(jump[j]),
                                               "limit": float(self.step_limit[j])}
        self.q_prev = q

        # 3) 관절 한계
        lo, hi = r.arm_range[:, 0], r.arm_range[:, 1]
        mg = self.s.joint_limit_margin
        viol = np.where((q < lo - mg) | (q > hi + mg))[0]
        if len(viol):
            j = int(viol[0])
            return "JOINT_LIMIT", {"joint": j + 1, "q": float(q[j]), "range": [float(lo[j]), float(hi[j])]}

        # 4) 예기치 않은 충돌(로봇이 관련된 모든 접촉)
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 in r.robot_bodies or b2 in r.robot_bodies:
                return "UNEXPECTED_CONTACT", {"body1": r.body_name(b1), "body2": r.body_name(b2),
                                              "dist": float(c.dist), "pos": c.pos.tolist()}

        # 5) 추종 오차(REACH, RETURN_HOME에서만 q_ref가 주어짐)
        if q_ref is not None:
            err = np.abs(q - q_ref)
            if np.any(err > self.s.tracking_error_max):
                j = int(np.argmax(err))
                return "TRACKING_ERROR", {"joint": j + 1, "err": float(err[j]),
                                          "limit": float(self.s.tracking_error_max)}
        return None, None

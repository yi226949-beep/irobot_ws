"""모델 로드, 이름→인덱스 매핑, 초기 keyframe 적용.

초기화 이후 시뮬레이션 데이터의 qpos/qvel 대입은 금지다.
허용되는 유일한 지점은 apply_initial_keyframe()이다(check_forbidden.py R1-a).
"""

import mujoco
import numpy as np

from .config import resolve


class ModelLoadError(RuntimeError):
    pass


class Robot:
    def __init__(self, cfg):
        path = resolve(cfg, cfg.model.scene)
        try:
            self.model = mujoco.MjModel.from_xml_path(str(path))
        except Exception as e:
            raise ModelLoadError(f"{path}: {e}") from e
        m = self.model
        self.data = mujoco.MjData(m)
        self.scene_path = path

        if not m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_AUTORESET:
            raise ModelLoadError("모델 옵션에서 autoreset이 꺼져 있지 않습니다(자동 리셋 금지).")

        try:
            joints = [m.joint(n) for n in cfg.model.arm_joints]
            self.arm_qadr = np.array([j.qposadr[0] for j in joints])
            self.arm_dofadr = np.array([j.dofadr[0] for j in joints])
            self.arm_range = np.array([j.range for j in joints])
            self.arm_act = np.array([m.actuator(n).id for n in cfg.model.arm_actuators])
            self.finger_act = m.actuator(cfg.model.finger_actuator).id
            self.tcp_site = m.site(cfg.model.tcp_site).id
            self.target_mocap = m.body(cfg.model.target_body).mocapid[0]
            self.target_geom = m.geom(cfg.model.target_geom).id
            self.home_key = m.key(cfg.model.home_key).id
        except KeyError as e:
            raise ModelLoadError(f"모델에 필요한 이름이 없습니다: {e}") from e

        for i, a in enumerate(self.arm_act):
            if m.actuator_trnid[a, 0] != m.joint(cfg.model.arm_joints[i]).id:
                raise ModelLoadError(f"액추에이터 {cfg.model.arm_actuators[i]}가 관절과 맞지 않습니다.")

        self.home = np.array(cfg.model.home, float)
        self.vmax = np.array(cfg.model.vmax, float)
        self.finger_open = float(cfg.model.finger_open)
        if not np.allclose(m.key_qpos[self.home_key][self.arm_qadr], self.home, atol=1e-6):
            raise ModelLoadError("모델의 home keyframe과 설정의 home이 다릅니다.")
        if np.any(self.home < self.arm_range[:, 0]) or np.any(self.home > self.arm_range[:, 1]):
            raise ModelLoadError("home이 관절 한계 밖입니다.")

        # 로봇 body(이름이 fr3_로 시작) 집합: 충돌 감시에 사용
        self.robot_bodies = {b for b in range(m.nbody)
                             if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith("fr3_")}
        self.dt = m.opt.timestep

    def apply_initial_keyframe(self):
        """초기 조건 설정(프로그램 시작 시 1회). 복귀 용도로 쓰지 않는다."""
        m, d = self.model, self.data
        d.qpos[:] = m.key_qpos[self.home_key]
        d.qvel[:] = 0.0
        d.ctrl[:] = m.key_ctrl[self.home_key]
        mujoco.mj_forward(m, d)

    # --- 읽기 전용 조회 ---
    def q(self):
        return self.data.qpos[self.arm_qadr].copy()

    def qd(self):
        return self.data.qvel[self.arm_dofadr].copy()

    def tcp_pose(self):
        d = self.data
        return d.site_xpos[self.tcp_site].copy(), d.site_xmat[self.tcp_site].reshape(3, 3).copy()

    def set_arm_ctrl(self, q_ref):
        self.data.ctrl[self.arm_act] = q_ref
        self.data.ctrl[self.finger_act] = self.finger_open

    def body_name(self, b):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b)

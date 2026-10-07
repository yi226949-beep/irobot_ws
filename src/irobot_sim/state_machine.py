"""reach → home 복귀 반복 상태 머신 (계획 §4).

한 물리 스텝마다 pre_step()(제어 입력 계산) → [step_hook] → mj_step → post_step()(감시·판정·전이) 순서로 호출된다.
"""

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from . import viz
from .checks import Hold, SafetyMonitor, home_metrics, reach_metrics
from .planning import TargetGenerationError
from .trajectory import Quintic, quintic_duration


class State(str, Enum):
    INIT = "INIT"
    CHECK_HOME = "CHECK_HOME"
    SAMPLE_TARGET = "SAMPLE_TARGET"
    REACH = "REACH"
    VERIFY_REACH = "VERIFY_REACH"
    RETURN_HOME = "RETURN_HOME"
    VERIFY_HOME = "VERIFY_HOME"
    FAILED = "FAILED"
    SHUTDOWN = "SHUTDOWN"


@dataclass(frozen=True)
class StepContext:
    """step_hook에 넘기는 읽기 전용 정보."""
    state: str
    cycle: int
    time: float
    time_in_state: float
    arm_qadr: tuple
    arm_dofadr: tuple


@dataclass
class RunOptions:
    master_seed: int
    max_cycles: int | None = None
    start_cycle: int = 1
    fixed_target: tuple | None = None
    fixed_yaw: float | None = None
    skip_validation: bool = False


class StateMachine:
    def __init__(self, robot, planner, cfg, log, opts):
        self.r, self.planner, self.cfg, self.log, self.opts = robot, planner, cfg, log, opts
        self.monitor = SafetyMonitor(robot, cfg)
        self.state = State.INIT
        self.t_state = 0.0
        self.cycle = opts.start_cycle - 1      # 현재(마지막으로 시작한) 사이클 번호
        self.started = 0                       # 목표가 확정된 사이클 수
        self.success = 0                       # 도달·복귀가 모두 확인된 사이클 수
        self.target = None
        self.traj = None
        self.t_traj = 0.0
        self.t_phase = 0.0                     # reach/return 타임아웃 기준 시각
        self.q_ref = None
        self.ctrl_target = robot.home.copy()
        self.hold = None
        self.home_ok = False
        self.pause_until = None
        self.failure = None
        self.shutdown_reason = None
        self.trace_every = max(1, round(1.0 / (cfg.run.trace_hz * robot.dt))) if cfg.run.trace_hz > 0 else 0
        self.steps = 0
        self.track_max = np.zeros(7)
        self.marker_phase = "IDLE"

    # ------------------------------------------------------------ 공통
    @property
    def now(self):
        return float(self.r.data.time)

    @property
    def finished(self):
        return self.failure is not None or self.shutdown_reason is not None

    def context(self):
        return StepContext(self.state.value, self.cycle, self.now, self.now - self.t_state,
                           tuple(self.r.arm_qadr), tuple(self.r.arm_dofadr))

    def _enter(self, state, **info):
        prev = self.state
        self.state = state
        self.t_state = self.now
        self.log.event("state", frm=prev.value, to=state.value, cycle=self.cycle, **info)
        self.log.info(f"[cycle {self.cycle}] {prev.value} → {state.value}" +
                      (f"  {info}" if info else ""))

    def request_shutdown(self, reason):
        if self.shutdown_reason is None:
            self.shutdown_reason = reason

    # ------------------------------------------------------------ 시작
    def start(self):
        self.r.apply_initial_keyframe()
        self.monitor.reset_prev()
        self.log.sim_time = self.now
        self.log.event("init", home=self.r.home, scene=str(self.r.scene_path), master_seed=self.opts.master_seed,
                       cfg_tracking_error_max=self.cfg.safety.tracking_error_max)
        self.hold = Hold(self.cfg.home.hold_time)
        self.ctrl_target = self.r.home.copy()
        self._enter(State.CHECK_HOME)

    # ------------------------------------------------------------ 스텝 전: 제어 입력
    def pre_step(self):
        if self.state in (State.REACH, State.RETURN_HOME):
            self.q_ref = self.traj(self.now - self.t_traj)
            self.r.set_arm_ctrl(self.q_ref)
        else:
            self.q_ref = None
            self.r.set_arm_ctrl(self.ctrl_target)

    # ------------------------------------------------------------ 스텝 후: 감시·판정·전이
    def post_step(self):
        if self.finished:
            return
        self.steps += 1
        self.log.sim_time = self.now
        dt = self.r.dt

        kind, detail = self.monitor.check(self.q_ref)
        if kind:
            return self._fail(kind, detail)

        if self.q_ref is not None:
            self.track_max = np.maximum(self.track_max, np.abs(self.r.q() - self.q_ref))
            if self.trace_every and self.steps % self.trace_every == 0:
                self.log.event("trace", state=self.state.value, cycle=self.cycle,
                               q=np.round(self.r.q(), 6), q_ref=np.round(self.q_ref, 6))

        s = self.state
        if s == State.CHECK_HOME:
            hm = home_metrics(self.r)
            if self.hold.update(hm["joint_err_max"] < self.cfg.home.joint_tol and hm["qd_max"] < self.cfg.home.vel_tol, dt):
                self.log.event("check_home_ok", **hm)
                self._sample()
            elif self.now - self.t_state > self.cfg.home.init_timeout:
                self._fail("INIT_HOME", hm)

        elif s == State.REACH:
            if self.now - self.t_phase > self.cfg.reach.timeout:
                return self._fail("REACH_TIMEOUT", {"timeout": self.cfg.reach.timeout})
            if self.traj.done(self.now - self.t_traj):
                self.ctrl_target = self.target.q_goal.copy()
                self.hold = Hold(self.cfg.reach.hold_time)
                self._enter(State.VERIFY_REACH)

        elif s == State.VERIFY_REACH:
            rm = reach_metrics(self.r, self.target)
            ok = (rm["pos_err"] < self.cfg.reach.pos_tol and rm["ori_err_deg"] < self.cfg.reach.ori_tol_deg
                  and rm["qd_max"] < self.cfg.reach.vel_tol)
            if self.hold.update(ok, dt):
                self.log.event("reach_verified", cycle=self.cycle, reach_time=self.now - self.t_phase,
                               track_max=self.track_max, **rm)
                self.log.info(f"[cycle {self.cycle}] 도달 확인: 위치오차 {rm['pos_err']*1000:.2f} mm, "
                              f"방향오차 {rm['ori_err_deg']:.2f}°, 소요 {self.now - self.t_phase:.2f} s")
                self._start_return()
            elif self.now - self.t_phase > self.cfg.reach.timeout:
                self._fail("REACH_TIMEOUT", {"timeout": self.cfg.reach.timeout})

        elif s == State.RETURN_HOME:
            if self.now - self.t_phase > self.cfg.home.timeout:
                return self._fail("RETURN_TIMEOUT", {"timeout": self.cfg.home.timeout})
            if self.traj.done(self.now - self.t_traj):
                self.ctrl_target = self.r.home.copy()
                self.hold = Hold(self.cfg.home.hold_time)
                self.home_ok = False
                self.pause_until = None
                self._enter(State.VERIFY_HOME)

        elif s == State.VERIFY_HOME:
            if not self.home_ok:
                hm = home_metrics(self.r)
                if self.hold.update(hm["joint_err_max"] < self.cfg.home.joint_tol and hm["qd_max"] < self.cfg.home.vel_tol, dt):
                    self.home_ok = True
                    self.success += 1
                    self.log.event("home_verified", cycle=self.cycle, return_time=self.now - self.t_phase,
                                   track_max=self.track_max, **hm)
                    self.log.info(f"[cycle {self.cycle}] 복귀 확인: 관절오차 최대 {hm['joint_err_max']:.5f} rad, "
                                  f"소요 {self.now - self.t_phase:.2f} s  (성공 {self.success})")
                    if self.opts.max_cycles is not None and self.started >= self.opts.max_cycles:
                        self.request_shutdown("MAX_CYCLES")
                        return
                    self.pause_until = self.now + self.cfg.run.cycle_pause
                elif self.now - self.t_phase > self.cfg.home.timeout:
                    return self._fail("RETURN_TIMEOUT", {"timeout": self.cfg.home.timeout, **hm})
            elif self.now >= self.pause_until - 1e-9:
                self._sample()

    # ------------------------------------------------------------ 전이 동작
    def _sample(self):
        """SAMPLE_TARGET: home 복귀(또는 초기 home)가 확인된 뒤에만 호출된다."""
        self.cycle += 1
        self._enter(State.SAMPLE_TARGET)
        self.marker_phase = "IDLE"
        try:
            if self.opts.fixed_target is not None:
                tgt = self.planner.fixed(self.cycle, self.opts.fixed_target, self.opts.fixed_yaw,
                                         self.opts.skip_validation)
            else:
                tgt = self.planner.sample(self.cycle, self.opts.master_seed)
        except TargetGenerationError as e:
            return self._fail("TARGET_GENERATION", {"reason": str(e), "attempts": e.attempts, "rejects": e.rejects})
        self.target = tgt
        self.started += 1
        self.track_max = np.zeros(7)
        q0 = self.r.q()
        T = quintic_duration(q0, tgt.q_goal, self.r.vmax, self.cfg.trajectory.vel_scale,
                             self.cfg.trajectory.min_duration, self.cfg.trajectory.max_duration)
        self.traj = Quintic(q0, tgt.q_goal, T)
        self.t_traj = self.t_phase = self.now
        self.log.event("target", cycle=self.cycle, seed=tgt.seed, pos=tgt.pos, yaw=tgt.yaw, q_goal=tgt.q_goal,
                       attempts=tgt.attempts, rejects=tgt.rejects, validated=tgt.validated,
                       ik_pos_err=tgt.ik_pos_err, ik_ori_err=tgt.ik_ori_err, sigma_min=tgt.sigma_min, duration=T)
        self.log.info(f"[cycle {self.cycle}] 목표 확정: pos={np.round(tgt.pos, 3).tolist()} "
                      f"yaw={math.degrees(tgt.yaw):.1f}° 시도 {tgt.attempts}회 기각 {tgt.rejects} 궤적 {T:.2f} s")
        self.marker_phase = "REACH"
        self._enter(State.REACH)

    def _start_return(self):
        q0 = self.r.q()   # 현재 실측 관절값에서 출발
        T = quintic_duration(q0, self.r.home, self.r.vmax, self.cfg.trajectory.vel_scale,
                             self.cfg.trajectory.min_duration, self.cfg.trajectory.max_duration)
        self.traj = Quintic(q0, self.r.home, T)
        self.t_traj = self.t_phase = self.now
        self.marker_phase = "RETURN"
        self._enter(State.RETURN_HOME, duration=round(T, 3))

    # ------------------------------------------------------------ 실패
    def _fail(self, kind, detail):
        r = self.r
        q, qd = r.q(), r.qd()
        p, _ = r.tcp_pose()
        report = {
            "type": kind,
            "cycle": self.cycle,
            "state": self.state.value,
            "time_in_state": self.now - self.t_state,
            "time_in_phase": (self.now - self.t_phase) if self.state in (
                State.REACH, State.VERIFY_REACH, State.RETURN_HOME, State.VERIFY_HOME) else None,
            "sim_time": self.now,
            "target": self.target.pos if self.target is not None else None,
            "tcp": p,
            "q": q,
            "qd": qd,
            "detail": detail,
            "seed": self.target.seed if self.target is not None else [self.opts.master_seed, self.cycle],
        }
        if self.target is not None and self.state in (State.REACH, State.VERIFY_REACH):
            rm = reach_metrics(r, self.target)
            report["pos_err"], report["ori_err_deg"] = rm["pos_err"], rm["ori_err_deg"]
            report["joint_err_vs_goal"] = q - self.target.q_goal
        else:
            report["joint_err_vs_home"] = q - r.home
            if self.target is not None:
                rm = reach_metrics(r, self.target)
                report["pos_err"], report["ori_err_deg"] = rm["pos_err"], rm["ori_err_deg"]
        report["tracking_err"] = (q - self.q_ref) if self.q_ref is not None else None
        self.failure = report
        self.marker_phase = "FAILED"
        self.log.event("failure", **report)
        fmt = lambda v: np.array2string(np.asarray(v, float), precision=4, suppress_small=True) if v is not None else "-"  # noqa: E731
        lines = [
            f"실패 유형      : {kind}",
            f"사이클 / 상태  : {self.cycle} / {self.state.value}",
            f"경과 시간      : 상태 진입 후 {report['time_in_state']:.3f} s"
            + (f", reach/복귀 시작 후 {report['time_in_phase']:.3f} s" if report["time_in_phase"] is not None else "")
            + f", 전체 sim {self.now:.3f} s",
            f"목표 좌표      : {fmt(report['target'])}",
            f"현재 TCP 위치  : {fmt(p)}",
            f"위치/방향 오차 : " + (f"{report['pos_err']*1000:.2f} mm / {report['ori_err_deg']:.2f}°"
                                     if "pos_err" in report else "-"),
            ("관절 오차(목표): " + fmt(report["joint_err_vs_goal"])) if "joint_err_vs_goal" in report
            else ("관절 오차(home): " + fmt(report["joint_err_vs_home"])),
            f"추종 오차      : {fmt(report['tracking_err'])}",
            f"관절 속도      : {fmt(qd)}",
            f"세부           : {detail}",
            f"사이클 시드    : {report['seed']}",
        ]
        self.log.block("실패 보고 (반복 중단)", lines)
        self.state = State.FAILED
        self.log.event("state", frm=report["state"], to="FAILED", cycle=self.cycle)

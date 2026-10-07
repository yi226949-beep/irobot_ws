"""실행 진입점: CLI, 시뮬레이션 루프(뷰어/헤드리스), 종료 처리.

run(cfg, args, step_hook=None)
  step_hook: 매 스텝 mj_step 직전에 step_hook(model, data, ctx)로 호출된다. 기본값 None.
  이 패키지는 훅을 호출만 한다. 훅 구현(검증용 실패 주입)은 scripts/ 아래 검증 전용 스크립트에만 있다.
"""

import argparse
import math
import os
import signal
import time

import mujoco

from . import viz
from .config import apply_overrides, load_config, resolve
from .logs import RunLogger
from .model import ModelLoadError, Robot
from .planning import Planner
from .state_machine import RunOptions, State, StateMachine


def build_arg_parser():
    ap = argparse.ArgumentParser(prog="irobot_sim", description="FR3 랜덤 목표 reach → home 복귀 반복")
    ap.add_argument("--config", help="설정 파일(기본 config/sim.toml)")
    ap.add_argument("--seed", type=int, help="마스터 시드(기본: 무작위, 로그에 기록)")
    ap.add_argument("--max-cycles", type=int, help="검증용: 이 횟수만큼 성공하면 정상 종료(기본: 제한 없음)")
    ap.add_argument("--replay-cycle", type=int, help="검증용: 이 사이클 번호부터 시작(--seed와 함께 재현)")
    ap.add_argument("--headless", action="store_true", help="검증용: 뷰어 없이 최대 속도로 실행")
    ap.add_argument("--quiet", action="store_true", help="콘솔 출력 줄이기(로그 파일은 동일)")
    # 데이터(qpos 등)를 건드리지 않는 검증용 설정 옵션
    ap.add_argument("--fixed-target", help="검증용: 고정 목표 x,y,z [m]")
    ap.add_argument("--fixed-yaw-deg", type=float, help="고정 목표 yaw [deg] (기본: home 자세의 yaw)")
    ap.add_argument("--skip-validation", action="store_true", help="검증용: 고정 목표 유효성 검사 생략")
    ap.add_argument("--reach-timeout", type=float)
    ap.add_argument("--return-timeout", type=float)
    ap.add_argument("--max-target-attempts", type=int)
    ap.add_argument("--tracking-error-max", type=float)
    ap.add_argument("--cycle-pause", type=float)
    ap.add_argument("--model", help="검증용: scene XML 경로 덮어쓰기")
    return ap


class Stop:
    def __init__(self):
        self.reason = None

    def set(self, reason):
        if self.reason is None:
            self.reason = reason


def run(args, step_hook=None):
    cfg = apply_overrides(load_config(args.config), args)
    log = RunLogger(resolve(cfg, cfg.run.log_dir), quiet=args.quiet)
    master_seed = args.seed if args.seed is not None else int.from_bytes(os.urandom(4), "little")
    log.info(f"로그: {log.text_path.name}, {log.jsonl_path.name} / 마스터 시드 {master_seed}")
    log.event("run_start", argv=vars(args), master_seed=master_seed, config=str(cfg.path),
              hook=getattr(step_hook, "__name__", None) if step_hook else None)

    stop = Stop()
    prev_sigint = signal.signal(signal.SIGINT, lambda *_: stop.set("SIGINT"))
    t_wall0 = time.time()
    sm = None
    exit_code = 0
    try:
        try:
            robot = Robot(cfg)
        except ModelLoadError as e:
            log.event("failure", type="MODEL_LOAD", cycle=0, state="INIT", detail=str(e))
            log.block("실패 보고 (반복 중단)", ["실패 유형      : MODEL_LOAD", "사이클 / 상태  : 0 / INIT",
                                             f"세부           : {e}"])
            return 1, None, log
        planner = Planner(robot, cfg)
        fixed = tuple(float(x) for x in args.fixed_target.split(",")) if args.fixed_target else None
        opts = RunOptions(master_seed=master_seed, max_cycles=args.max_cycles,
                          start_cycle=args.replay_cycle or 1, fixed_target=fixed,
                          fixed_yaw=math.radians(args.fixed_yaw_deg) if args.fixed_yaw_deg is not None else None,
                          skip_validation=args.skip_validation)
        sm = StateMachine(robot, planner, cfg, log, opts)
        sm.start()

        def step_once():
            sm.pre_step()
            if step_hook is not None:
                step_hook(robot.model, robot.data, sm.context())
            mujoco.mj_step(robot.model, robot.data)
            sm.post_step()

        if args.headless:
            while not sm.finished and stop.reason is None:
                step_once()
        else:
            _viewer_loop(robot, sm, cfg, stop, step_once, log)

        exit_code = 1 if sm.failure else 0
    finally:
        signal.signal(signal.SIGINT, prev_sigint)
        if sm is not None:
            reason = stop.reason or sm.shutdown_reason or ("FAILED" if sm.failure else "UNKNOWN")
            _summary(sm, log, reason, time.time() - t_wall0)
        log.close()
    return exit_code, sm, log


def _viewer_loop(robot, sm, cfg, stop, step_once, log):
    import mujoco.viewer

    quit_code = ord(cfg.run.quit_key.upper()) if len(cfg.run.quit_key) == 1 else 256  # 256 = GLFW Esc

    def on_key(keycode):
        if keycode == quit_code:
            stop.set("QUIT_KEY")

    frame = 1.0 / cfg.run.viewer_fps
    max_steps = int(4 * frame / robot.dt)
    v = mujoco.viewer.launch_passive(robot.model, robot.data, key_callback=on_key)
    try:
        log.info(f"뷰어 시작. 종료: 창 닫기 / '{cfg.run.quit_key}' 키 / 터미널 Ctrl+C")
        wall0, sim0 = time.perf_counter(), robot.data.time
        failed_reported = False
        while v.is_running() and stop.reason is None and sm.shutdown_reason is None:
            t0 = time.perf_counter()
            with v.lock():
                if sm.failure is None:
                    target_t = sim0 + (t0 - wall0)
                    n = 0
                    while robot.data.time < target_t and n < max_steps and not sm.finished and stop.reason is None:
                        step_once()
                        n += 1
                    if n >= max_steps:   # 계산이 밀리면 실시간 기준을 다시 맞춘다
                        wall0, sim0 = time.perf_counter(), robot.data.time
                elif not failed_reported:
                    log.info("실패: 물리 스텝을 멈췄습니다. 창을 닫거나 종료 키/Ctrl+C로 끝냅니다.")
                    failed_reported = True
                tgt = sm.target.pos if sm.target is not None else None
                viz.show_marker(robot, tgt, sm.marker_phase)
            v.sync()
            time.sleep(max(0.0, frame - (time.perf_counter() - t0)))
        if stop.reason is None and sm.shutdown_reason is None and not v.is_running():
            stop.set("WINDOW_CLOSED")
    finally:
        # 뷰어 스레드가 완전히 정리된 뒤 반환한다. 바로 프로세스를 끝내면 정리 중인
        # 뷰어 스레드와 충돌해 segfault가 난다(D-008).
        v.close()
        deadline = time.perf_counter() + 3.0
        while v.is_running() and time.perf_counter() < deadline:
            time.sleep(0.05)
        time.sleep(0.3)


def _summary(sm, log, reason, wall):
    sm.log.sim_time = sm.now
    if sm.state != State.FAILED:
        sm.state = State.SHUTDOWN
    fail = sm.failure["type"] if sm.failure else None
    log.event("summary", cycles_started=sm.started, cycles_succeeded=sm.success, failure=fail,
              shutdown_reason=reason, sim_time=sm.now, wall_time=wall, master_seed=sm.opts.master_seed)
    log.block("실행 요약", [
        f"총 사이클(목표 확정) : {sm.started}",
        f"성공(도달+복귀 확인) : {sm.success}",
        f"실패                 : {fail or '없음'}",
        f"종료 사유            : {reason}",
        f"sim 시간 / 실제 시간 : {sm.now:.1f} s / {wall:.1f} s",
        f"마스터 시드          : {sm.opts.master_seed}",
        f"로그                 : logs/{log.jsonl_path.name}",
    ])


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    code, _, _ = run(args)
    return code

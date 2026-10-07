"""config/sim.toml 로드와 CLI 덮어쓰기."""

import os
import tomllib
from pathlib import Path
from types import SimpleNamespace

WS = Path(os.environ.get("IROBOT_WS", Path(__file__).resolve().parents[2]))
DEFAULT_CONFIG = WS / "config" / "sim.toml"


def _ns(obj):
    if isinstance(obj, dict):
        return SimpleNamespace(**{k: _ns(v) for k, v in obj.items()})
    return obj


def load_config(path=None):
    path = Path(path) if path else DEFAULT_CONFIG
    with open(path, "rb") as f:
        cfg = _ns(tomllib.load(f))
    cfg.ws = WS
    cfg.path = path
    return cfg


def resolve(cfg, rel):
    p = Path(rel)
    return p if p.is_absolute() else cfg.ws / p


def apply_overrides(cfg, args):
    """검증용 CLI 옵션을 설정에 반영한다. 데이터(qpos 등)를 건드리는 옵션은 여기 없다."""
    if args.model:
        cfg.model.scene = args.model
    if args.reach_timeout is not None:
        cfg.reach.timeout = args.reach_timeout
    if args.return_timeout is not None:
        cfg.home.timeout = args.return_timeout
    if args.max_target_attempts is not None:
        cfg.target.max_attempts = args.max_target_attempts
    if args.tracking_error_max is not None:
        cfg.safety.tracking_error_max = args.tracking_error_max
    if args.cycle_pause is not None:
        cfg.run.cycle_pause = args.cycle_pause
    elif args.headless:
        cfg.run.cycle_pause = 0.0
    return cfg

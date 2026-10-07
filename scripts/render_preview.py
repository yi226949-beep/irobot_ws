"""README용 장면 이미지 생성(오프스크린 렌더링).

시뮬레이션과 무관한 별도 MjData에 home 자세와 예시 목표 도달 자세를 놓고 렌더링한다.
사용법: MUJOCO_GL=egl scripts/run.sh scripts/render_preview.py
출력: docs/images/preview.png
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from irobot_sim.config import load_config  # noqa: E402
from irobot_sim.model import Robot  # noqa: E402
from irobot_sim.planning import Planner  # noqa: E402


def render(m, d, cam, w, h):
    with mujoco.Renderer(m, height=h, width=w) as r:
        r.update_scene(d, camera=cam)
        return r.render()


def main():
    cfg = load_config()
    robot = Robot(cfg)
    planner = Planner(robot, cfg)
    m = robot.model
    tgt = planner.sample(cycle=3, master_seed=1)   # 시드 1의 3번째 사이클 목표

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.15, -0.08, 0.38]
    cam.distance, cam.azimuth, cam.elevation = 1.45, 140, -20

    panels = []
    for q, marker in ((robot.home, False), (tgt.q_goal, True)):
        d = mujoco.MjData(m)
        d.qpos[robot.arm_qadr] = q
        d.qpos[planner.finger_qadr] = robot.finger_open
        d.mocap_pos[robot.target_mocap] = tgt.pos
        m.geom_rgba[robot.target_geom] = (0.1, 0.9, 0.2, 0.9) if marker else (1.0, 0.85, 0.0, 0.85)
        mujoco.mj_forward(m, d)
        panels.append(render(m, d, cam, 640, 480))
    img = np.concatenate(panels, axis=1)

    out = cfg.ws / "docs/images/preview.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    _write_png(out, img)
    print(f"저장: {out.relative_to(cfg.ws)}  ({img.shape[1]}x{img.shape[0]}), 목표 {np.round(tgt.pos, 3).tolist()}")


def _write_png(path, rgb):
    """의존성 없이 PNG 저장(zlib)."""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


if __name__ == "__main__":
    main()

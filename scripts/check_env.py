"""단계 0 환경 확인.

사용법:
    scripts/run.sh scripts/check_env.py            # 환경 점검 + LD_LIBRARY_PATH 영향 판정
    scripts/run.sh scripts/check_env.py --viewer   # 빈 장면 뷰어 창 띄우기(창을 닫으면 종료)
    (내부용) --probe                               # 공유 라이브러리 로드 경로 수집
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

WS = Path(os.environ.get("IROBOT_WS", Path(__file__).resolve().parents[1]))

# 로드 경로를 비교할 공유 라이브러리 이름 접두어
LIB_PREFIXES = (
    "libmujoco", "libglfw", "libGL", "libEGL", "libGLX", "libGLdispatch",
    "libstdc++", "libX11", "libwayland-", "libgbm", "libdrm",
)

EMPTY_SCENE = """
<mujoco model="empty">
  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 .2 .3"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="4 4" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="grid"/>
  </worldbody>
</mujoco>
"""

# position 액추에이터 kv, actuatorgravcomp 지원 여부 확인용 최소 모델
FEATURE_MODEL = """
<mujoco>
  <worldbody>
    <body name="b" gravcomp="1">
      <joint name="j" type="hinge" actuatorgravcomp="true"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
    </body>
  </worldbody>
  <actuator>
    <position name="a" joint="j" kp="100" kv="10" forcerange="-5 5"/>
  </actuator>
</mujoco>
"""


def loaded_libs():
    libs = {}
    with open("/proc/self/maps") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 6:
                continue
            path = parts[5]
            name = os.path.basename(path)
            if name.startswith(LIB_PREFIXES):
                libs[name] = os.path.realpath(path)
    return dict(sorted(libs.items()))


def probe():
    """mujoco/glfw를 로드하고 숨김 창 + GL 컨텍스트를 만든 뒤 로드된 라이브러리를 JSON으로 출력."""
    out = {"ok": False}
    try:
        import glfw
        import mujoco

        if not glfw.init():
            raise RuntimeError("glfw.init() 실패")
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        win = glfw.create_window(64, 64, "probe", None, None)
        if not win:
            raise RuntimeError("glfw.create_window() 실패")
        glfw.make_context_current(win)
        model = mujoco.MjModel.from_xml_string(EMPTY_SCENE)
        ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        from OpenGL import GL
        out["gl_vendor"] = GL.glGetString(GL.GL_VENDOR).decode()
        out["gl_renderer"] = GL.glGetString(GL.GL_RENDERER).decode()
        out["gl_version"] = GL.glGetString(GL.GL_VERSION).decode()
        try:
            platform = glfw.get_platform()
            names = {getattr(glfw, n): n for n in dir(glfw) if n.startswith("PLATFORM_")}
            out["glfw_platform"] = names.get(platform, str(platform))
        except Exception:  # 구버전 glfw
            out["glfw_platform"] = "unknown"
        out["libs"] = loaded_libs()
        del ctx
        glfw.destroy_window(win)
        glfw.terminate()
        out["ok"] = True
    except Exception as e:  # 판정을 위해 실패도 JSON으로 돌려준다
        out["error"] = f"{type(e).__name__}: {e}"
        out["libs"] = loaded_libs()
    print(json.dumps(out))


def run_probe(env):
    r = subprocess.run([sys.executable, __file__, "--probe"], env=env,
                       capture_output=True, text=True, timeout=60)
    try:
        return json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:
        return {"ok": False, "error": f"probe 출력 해석 실패 (rc={r.returncode}): {r.stderr[-500:]}"}


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))
    return ok


def main_check():
    results = []
    print("== Python / venv")
    results.append(check("venv 인터프리터", sys.prefix.startswith(str(WS / ".venv")),
                         f"{sys.executable} ({sys.version.split()[0]})"))
    bad = [p for p in sys.path if "/opt/ros" in p or "drake-env" in p or "ros2_ws" in p]
    results.append(check("sys.path에 ROS/drake-env 경로 없음", not bad, ", ".join(bad) or "깨끗함"))

    print("== MuJoCo")
    import mujoco
    import numpy
    results.append(check("mujoco import", True, f"mujoco {mujoco.__version__}, numpy {numpy.__version__}"))
    try:
        m = mujoco.MjModel.from_xml_string(FEATURE_MODEL)
        kv_ok = m.actuator_biasprm[0, 2] == -10.0
        gc_ok = bool(m.jnt_actgravcomp[0]) and m.body_gravcomp[1] == 1.0
        results.append(check("position 액추에이터 kv 지원", kv_ok))
        results.append(check("gravcomp + actuatorgravcomp 지원", gc_ok))
    except Exception as e:
        results.append(check("기능 확인 모델 컴파일", False, str(e)))

    print("== 표시 환경")
    for k in ("DISPLAY", "WAYLAND_DISPLAY", "XDG_SESSION_TYPE", "MUJOCO_GL", "PYGLFW_LIBRARY_VARIANT"):
        print(f"  {k}={os.environ.get(k, '')}")

    print("== LD_LIBRARY_PATH 영향 판정")
    ld = os.environ.get("LD_LIBRARY_PATH", "")
    print(f"  현재 LD_LIBRARY_PATH={ld or '(없음)'}")
    env_with = dict(os.environ)
    env_without = {k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"}
    if not ld:
        print("  (현재 실행 환경에 LD_LIBRARY_PATH가 없음 — 셸의 원래 값으로 비교하려면 "
              "IROBOT_LD_COMPARE에 값을 넣어 실행)")
        cmp_val = os.environ.get("IROBOT_LD_COMPARE", "")
        if cmp_val:
            env_with["LD_LIBRARY_PATH"] = cmp_val
    p_with = run_probe(env_with)
    p_without = run_probe(env_without)
    for label, p in (("LD 있음", p_with), ("LD 없음", p_without)):
        status = "OK" if p.get("ok") else f"실패: {p.get('error')}"
        print(f"  [{label}] probe {status}")
        if p.get("ok"):
            print(f"      GL: {p['gl_vendor']} / {p['gl_renderer']} / {p['gl_version']}; "
                  f"GLFW platform: {p['glfw_platform']}")
        for name, path in p.get("libs", {}).items():
            print(f"      {name:28s} {path}")
    same = p_with.get("ok") and p_without.get("ok") and p_with["libs"] == p_without["libs"]
    verdict = "영향 없음" if same else "영향 있음 또는 판정 불가"
    print(f"  판정: {verdict}")
    results.append(check("GL 컨텍스트 생성(probe)", bool(p_without.get("ok"))))

    print("== 요약")
    print(f"  {'ALL PASS' if all(results) else 'FAIL 있음'}; LD_LIBRARY_PATH 판정: {verdict}")
    return 0 if all(results) else 1


def main_viewer():
    import time

    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_string(EMPTY_SCENE)
    data = mujoco.MjData(model)
    print("빈 장면 뷰어를 띄웁니다. 창을 닫으면 종료됩니다.")
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(1 / 60)
    print("뷰어 종료.")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    a = ap.parse_args()
    if a.probe:
        probe()
        sys.exit(0)
    sys.exit(main_viewer() if a.viewer else main_check())

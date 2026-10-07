"""FR3 + Franka Hand 모델 변환 파이프라인 (재현 가능).

source/franka_description (수정 금지)
  → [1] xacro 전개                   converted/urdf/fr3_hand.urdf
  → [2] 메시 변환(DAE→OBJ, STL 복사)  converted/meshes/{visual,collision}/
  → [3] URDF 후처리                  converted/urdf/fr3_hand.mj.urdf
  → [4] URDF→MJCF                    converted/mjcf/fr3_raw.xml
  → [5] MJCF 보강                    converted/mjcf/fr3.xml
  → [6] 장면                         converted/mjcf/scene.xml

사용법: scripts/run.sh scripts/build_model.py
"""

import copy
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from lxml import etree

WS = Path(os.environ.get("IROBOT_WS", Path(__file__).resolve().parents[1]))
SRC = WS / "models/franka/source/franka_description"
OUT = WS / "models/franka/converted"
URDF_DIR, MESH_DIR, MJCF_DIR = OUT / "urdf", OUT / "meshes", OUT / "mjcf"

ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["fr3_finger_joint1", "fr3_finger_joint2"]

# home: Franka 예제의 표준 ready 자세 (계획 §7)
HOME_ARM = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
FINGER_OPEN = 0.04

# 관절별 position 액추에이터 게인 초기값 (계획 §3-5; Menagerie FR3 참고, 단계 1에서 튜닝)
KP = [4500, 4500, 4500, 4500, 2000, 2000, 2000]
KV = [450, 450, 450, 450, 200, 200, 200]
FINGER_KP, FINGER_KV, FINGER_FORCE = 100.0, 3.0, 20.0

# 구조상 항상 맞닿는 비인접 body 쌍 (근거는 docs/DECISIONS.md D-005)
CONTACT_EXCLUDES = [
    ("fr3_link7", "fr3_hand"),  # link8(질량·형상 없는 fixed 링크)을 사이에 둔 강체 연결
]


def log(msg):
    print(f"[build] {msg}", flush=True)


def load_yaml_simple(path):
    """franka_description의 단순 중첩 YAML(키: 값, 들여쓰기) 파서. PyYAML 의존을 피하기 위함."""
    root, stack, anchors = {}, [(-1, None)], {}
    stack[0] = (-1, root)
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, _, val = line.strip().partition(":")
        while stack[-1][0] >= indent:
            stack.pop()
        parent = stack[-1][1]
        val = val.strip()
        if val == "" or val.startswith("&"):  # 중첩 맵(앵커 정의 포함)
            parent[key] = {}
            if val.startswith("&"):
                anchors[val[1:]] = parent[key]
            stack.append((indent, parent[key]))
        elif val.startswith("*"):  # 앵커 참조
            parent[key] = anchors[val[1:]]
        else:
            try:
                parent[key] = float(val)
            except ValueError:
                parent[key] = val
    return root


# ---------------------------------------------------------------- [1] xacro
def step_xacro():
    URDF_DIR.mkdir(parents=True, exist_ok=True)
    out = URDF_DIR / "fr3_hand.urdf"
    subprocess.run([str(WS / "scripts/expand_xacro.sh"), str(out)], check=True)
    return out


# ---------------------------------------------------------------- [2] meshes
def mesh_color(geom):
    vis = geom.visual
    mat = getattr(vis, "material", None)
    if mat is not None and getattr(mat, "main_color", None) is not None:
        return tuple(np.round(np.asarray(mat.main_color, float) / 255.0, 3))
    fc = getattr(vis, "face_colors", None)
    if fc is not None and len(fc):
        vals, counts = np.unique(fc, axis=0, return_counts=True)
        return tuple(np.round(vals[np.argmax(counts)] / 255.0, 3))
    return (0.9, 0.9, 0.9, 1.0)


def convert_dae(dae_path, stem):
    """DAE 씬 그래프 변환을 적용하고 색상별로 합쳐 OBJ로 내보낸다. [(obj 상대경로, rgba), ...] 반환."""
    scene = trimesh.load(dae_path, force="scene")
    groups = {}
    for node in scene.graph.nodes_geometry:
        transform, gname = scene.graph[node]
        g = scene.geometry[gname]
        if not isinstance(g, trimesh.Trimesh) or len(g.faces) == 0:
            continue
        m = trimesh.Trimesh(vertices=g.vertices.copy(), faces=g.faces.copy(), process=False)
        m.apply_transform(transform)
        groups.setdefault(mesh_color(g), []).append(m)
    parts = []
    (MESH_DIR / "visual").mkdir(parents=True, exist_ok=True)
    for i, (rgba, meshes) in enumerate(sorted(groups.items(), key=lambda kv: -sum(len(m.faces) for m in kv[1]))):
        merged = trimesh.util.concatenate(meshes)
        rel = f"visual/{stem}_v{i}.obj"
        data = trimesh.exchange.obj.export_obj(merged, include_normals=False, include_color=False,
                                               include_texture=False, write_texture=False)
        (MESH_DIR / rel).write_text(data)
        parts.append((rel, [float(c) for c in rgba], merged.bounds))
    del scene
    return parts


def step_meshes(urdf_path):
    """URDF에 등장하는 메시를 변환한다. 반환: {package 경로: 변환 정보}."""
    tree = etree.parse(str(urdf_path))
    mapping, report = {}, []
    for mesh in tree.iter("mesh"):
        uri = mesh.get("filename")
        if uri in mapping:
            continue
        rel = uri.replace("package://franka_description/", "")
        src = SRC / rel
        robot = "fr3" if "/robots/fr3/" in rel else "hand"
        stem = f"{robot}_{src.stem}"
        if src.suffix.lower() == ".dae":
            parts = convert_dae(src, stem)
            mapping[uri] = {"kind": "visual", "parts": [(p, c) for p, c, _ in parts]}
            vb = np.array([[b[0] for _, _, b in parts], [b[1] for _, _, b in parts]])
            bounds = np.array([vb[0].min(axis=0), vb[1].max(axis=0)])
            mapping[uri]["bounds"] = bounds.tolist()
            log(f"DAE→OBJ {rel}: {len(parts)}개 조각")
        elif src.suffix.lower() == ".stl":
            (MESH_DIR / "collision").mkdir(parents=True, exist_ok=True)
            dst_rel = f"collision/{stem}_c.stl"
            shutil.copyfile(src, MESH_DIR / dst_rel)
            m = trimesh.load(src)
            mapping[uri] = {"kind": "collision", "file": dst_rel, "bounds": m.bounds.tolist()}
        else:
            raise RuntimeError(f"지원하지 않는 메시 형식: {uri}")

    # 시각/충돌 bbox 정합 검사 (같은 링크 이름끼리)
    vis = {Path(u).stem: v for u, v in mapping.items() if v["kind"] == "visual"}
    col = {Path(u).stem: v for u, v in mapping.items() if v["kind"] == "collision"}
    worst = 0.0
    for name in sorted(set(vis) & set(col)):
        cv = np.mean(vis[name]["bounds"], axis=0)
        cc = np.mean(col[name]["bounds"], axis=0)
        d = float(np.linalg.norm(cv - cc))
        worst = max(worst, d)
        report.append((name, d))
        if d > 0.01:
            raise RuntimeError(f"시각/충돌 메시 bbox 중심 불일치: {name} {d*1000:.1f} mm")
    log("bbox 정합 검사 PASS: " + ", ".join(f"{n} {d*1000:.1f}mm" for n, d in report)
        + f" (최대 {worst*1000:.1f} mm)")
    return mapping


# ---------------------------------------------------------------- [3] URDF post-process
def step_urdf_post(urdf_path, mapping):
    tree = etree.parse(str(urdf_path))
    robot = tree.getroot()

    # 가속도계 프레임 링크 제거 (질량·형상 없는 센서 표시용; D-004)
    removed = 0
    for el in list(robot):
        if el.tag in ("link", "joint") and "accelerometer" in (el.get("name") or ""):
            robot.remove(el)
            removed += 1
    log(f"가속도계 프레임 링크/조인트 {removed}개 제거")

    # 메시 경로 치환. 시각 메시는 색상 조각마다 <visual>을 하나씩 만든다.
    for link in robot.iter("link"):
        for vis in list(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            info = mapping[mesh.get("filename")]
            idx = list(link).index(vis)
            link.remove(vis)
            for k, (rel, _rgba) in enumerate(info["parts"]):
                v = copy.deepcopy(vis)
                v.set("name", f"{link.get('name')}_vis{k}")
                v.find("geometry/mesh").set("filename", rel)
                link.insert(idx + k, v)
        for k, col in enumerate(link.findall("collision")):
            col.set("name", f"{link.get('name')}_col{k}")
            colm = col.find("geometry/mesh")
            if colm is not None:
                colm.set("filename", mapping[colm.get("filename")]["file"])

    # MuJoCo 컴파일러 확장 태그
    mj = etree.SubElement(robot, "mujoco")
    etree.SubElement(mj, "compiler", meshdir="../meshes/", strippath="false",
                     fusestatic="false", discardvisual="false", balanceinertia="true")
    out = URDF_DIR / "fr3_hand.mj.urdf"
    tree.write(str(out), pretty_print=True, xml_declaration=True, encoding="utf-8")
    return out


# ---------------------------------------------------------------- [4] URDF → MJCF
def step_to_mjcf(mj_urdf):
    MJCF_DIR.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(mj_urdf))
    raw = MJCF_DIR / "fr3_raw.xml"
    mujoco.mj_saveLastXML(str(raw), model)
    log(f"URDF→MJCF: nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} nmesh={model.nmesh} → {raw.name}")
    return raw, model


# ---------------------------------------------------------------- [5] MJCF 보강
def restore_inertial_precision(root, model):
    """mj_saveLastXML은 유효숫자 6자리로 저장한다. URDF를 직접 컴파일한 모델의 배정밀도 값으로
    <inertial>을 다시 써서 반올림 손실을 없앤다."""
    n = 0
    for b in root.iter("body"):
        inertial = b.find("inertial")
        if inertial is None:
            continue
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b.get("name"))
        f = lambda v: " ".join(f"{x:.17g}" for x in np.atleast_1d(v))  # noqa: E731
        inertial.attrib.clear()
        inertial.set("pos", f(model.body_ipos[i]))
        inertial.set("quat", f(model.body_iquat[i]))
        inertial.set("mass", f(model.body_mass[i]))
        inertial.set("diaginertia", f(model.body_inertia[i]))
        n += 1
    log(f"<inertial> {n}개를 배정밀도로 복원")


def step_augment(raw_path, mapping, urdf_model):
    dyn = load_yaml_simple(SRC / "robots/fr3/dynamics.yaml")
    lim = load_yaml_simple(SRC / "robots/fr3/joint_limits.yaml")
    rgba_of = {}
    for info in mapping.values():
        if info["kind"] == "visual":
            for rel, rgba in info["parts"]:
                rgba_of[Path(rel).stem] = rgba

    tree = etree.parse(str(raw_path))
    root = tree.getroot()
    root.set("model", "fr3_hand")
    restore_inertial_precision(root, urdf_model)

    comp = root.find("compiler")
    comp.set("autolimits", "true")
    comp.set("meshdir", "../meshes/")

    opt = root.find("option")
    if opt is None:
        opt = etree.Element("option")
        root.insert(list(root).index(comp) + 1, opt)
    opt.set("timestep", "0.002")
    opt.set("integrator", "implicitfast")

    # geom 분류: 시각(OBJ 메시) / 충돌(STL 메시, 손가락 박스)
    nvis = ncol = 0
    for g in root.iter("geom"):
        mesh = g.get("mesh")
        if mesh and mesh in rgba_of:
            g.set("contype", "0")
            g.set("conaffinity", "0")
            g.set("group", "2")
            g.set("rgba", " ".join(f"{c:g}" for c in rgba_of[mesh]))
            nvis += 1
        else:
            g.set("group", "3")
            g.set("rgba", "0.8 0.3 0.3 0.4")
            ncol += 1
    log(f"geom 분류: 시각 {nvis}, 충돌 {ncol}")

    # 관절 동역학: armature = motor_inertia * gear^2, damping/frictionloss는 dynamics.yaml
    for j in root.iter("joint"):
        name = j.get("name")
        if name in ARM_JOINTS:
            key = name.replace("fr3_", "")
            d = dyn[key]["dynamic"]
            arm = d["motor_inertia"] * d["gear_ratio"] ** 2
            j.set("armature", f"{arm:.4f}")
            j.set("damping", f"{d['damping']:g}")
            j.set("frictionloss", f"{d['friction']:g}")
            j.set("actuatorgravcomp", "true")
            lo, hi = lim[key]["limit"]["lower"], lim[key]["limit"]["upper"]
            j.set("range", f"{lo:g} {hi:g}")
        elif name in FINGER_JOINTS:
            j.set("actuatorgravcomp", "true")

    # 중력 보상: 로봇의 모든 움직이는 body
    for b in root.iter("body"):
        if b.get("name") not in ("base", "fr3_link0"):
            b.set("gravcomp", "1")

    # TCP site
    tcp = next(b for b in root.iter("body") if b.get("name") == "fr3_hand_tcp")
    etree.SubElement(tcp, "site", name="tcp", pos="0 0 0", size="0.006", rgba="1 0.2 0.2 1", group="4")

    # 손가락 mimic: 임포터가 equality를 만들지 않았으면 추가
    eq = root.find("equality")
    has_mimic = eq is not None and any(
        e.get("joint1") in FINGER_JOINTS and e.get("joint2") in FINGER_JOINTS for e in eq)
    if not has_mimic:
        if eq is None:
            eq = etree.SubElement(root, "equality")
        etree.SubElement(eq, "joint", name="finger_mimic", joint1="fr3_finger_joint2",
                         joint2="fr3_finger_joint1", polycoef="0 1 0 0 0", solimp="0.95 0.99 0.001",
                         solref="0.005 1")
        log("손가락 mimic equality 추가")
    else:
        log("손가락 mimic equality: 임포터가 이미 생성")

    # contact exclude
    contact = root.find("contact")
    if contact is None:
        contact = etree.SubElement(root, "contact")
    for b1, b2 in CONTACT_EXCLUDES:
        etree.SubElement(contact, "exclude", body1=b1, body2=b2)

    # 액추에이터
    old = root.find("actuator")
    if old is not None:
        root.remove(old)
    act = etree.SubElement(root, "actuator")
    for i, jn in enumerate(ARM_JOINTS):
        key = jn.replace("fr3_", "")
        eff = lim[key]["limit"]["effort"]
        lo, hi = lim[key]["limit"]["lower"], lim[key]["limit"]["upper"]
        etree.SubElement(act, "position", name=f"act_{key}", joint=jn, kp=f"{KP[i]:g}", kv=f"{KV[i]:g}",
                         ctrlrange=f"{lo:g} {hi:g}", forcerange=f"{-eff:g} {eff:g}")
    etree.SubElement(act, "position", name="act_finger", joint="fr3_finger_joint1",
                     kp=f"{FINGER_KP:g}", kv=f"{FINGER_KV:g}", ctrlrange="0 0.04",
                     forcerange=f"{-FINGER_FORCE:g} {FINGER_FORCE:g}")

    # home keyframe (초기 조건 1회용)
    old = root.find("keyframe")
    if old is not None:
        root.remove(old)
    kf = etree.SubElement(root, "keyframe")
    qpos = HOME_ARM + [FINGER_OPEN, FINGER_OPEN]
    ctrl = HOME_ARM + [FINGER_OPEN]
    etree.SubElement(kf, "key", name="home", qpos=" ".join(f"{q:.10g}" for q in qpos),
                     ctrl=" ".join(f"{c:.10g}" for c in ctrl))

    out = MJCF_DIR / "fr3.xml"
    etree.indent(tree, space="  ")
    tree.write(str(out), pretty_print=True, encoding="utf-8")
    return out


# ---------------------------------------------------------------- [6] scene
SCENE = """<mujoco model="fr3_scene">
  <include file="fr3.xml"/>

  <statistic center="0.35 0 0.35" extent="1.1"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="150" elevation="-25"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture name="groundplane" type="2d" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"/>
    <!-- 목표 마커: mocap body. 충돌하지 않음(contype/conaffinity 0). -->
    <body name="target" mocap="true" pos="0.5 0 0.4">
      <geom name="target_marker" type="sphere" size="0.015" rgba="1 0.85 0 0.85"
            contype="0" conaffinity="0" group="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def step_scene():
    out = MJCF_DIR / "scene.xml"
    out.write_text(SCENE)
    model = mujoco.MjModel.from_xml_path(str(out))
    log(f"scene.xml 로드 OK: nq={model.nq} nu={model.nu} nbody={model.nbody} ngeom={model.ngeom} nmocap={model.nmocap}")
    return out


def main():
    if not SRC.is_dir():
        sys.exit("원본이 없습니다. scripts/fetch_model.sh 를 먼저 실행하세요.")
    for d in (MESH_DIR, MJCF_DIR):
        if d.exists():
            shutil.rmtree(d)
    urdf = step_xacro()
    mapping = step_meshes(urdf)
    (OUT / "mesh_map.json").write_text(json.dumps(mapping, indent=1, ensure_ascii=False))
    mj_urdf = step_urdf_post(urdf, mapping)
    raw, urdf_model = step_to_mjcf(mj_urdf)
    step_augment(raw, mapping, urdf_model)
    step_scene()
    log("완료")


if __name__ == "__main__":
    main()

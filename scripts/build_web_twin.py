"""Build the browser digital-twin assets for the Blazor control UI (src/PoRobotTrain.Client/wwwroot/twin/).

Everything is exported from the *compiled* MuJoCo stand model, so the browser sees exactly what MuJoCo simulates:

* ``r1_stand.glb``      visual meshes (MuJoCo's own vertices + per-corner normals), one glTF node per body with the
                        body's local transform; Draco-compressed with gltf-transform when ``--compress`` is given.
* ``twin_manifest.json`` joint order (policy order from r1_joint_map.yaml), axes, limits, defaults, body tree,
                        MuJoCo collision geoms (for the browser's Rapier pose check), stand geoms, and golden
                        forward-kinematics poses (MuJoCo xpos/xquat) used by the C# and JS FK tests.
* ``policy.onnx``       byte copy of the validated Level 0 policy (sha256 recorded; never retrained here).
* ``policy_golden.json`` observations from the baseline rollout and the actions Python onnxruntime produces for them,
                        so the browser's ONNX Runtime Web run can be checked against the deployment contract.

The browser never simulates the robot: MuJoCo owns physics. These assets only display and check poses.

Usage:  .venv/Scripts/python scripts/build_web_twin.py [--compress]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from RobotControl import MJCF_DIR, MODELS_DIR  # noqa: E402
from RobotControl.configs.joint_map import JointMap  # noqa: E402

OUT_DIR = ROOT / "src" / "PoRobotTrain.Client" / "wwwroot" / "twin"
POLICY_DIR = MODELS_DIR / "r1_level0_upperbody"
GLTF_TRANSFORM = "@gltf-transform/cli@4.5.1"
VISUAL_GROUPS = (0, 1, 2)  # what MuJoCo's default renderer shows
COLLISION_GROUP = 3

# MuJoCo geom types
GEOM_PLANE, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER, GEOM_BOX, GEOM_MESH = 0, 2, 3, 5, 6, 7
PRIM_NAMES = {GEOM_SPHERE: "sphere", GEOM_CAPSULE: "capsule", GEOM_CYLINDER: "cylinder", GEOM_BOX: "box"}


def _wxyz_to_xyzw(q) -> list:
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def _r(v, n=6) -> list:
    return [round(float(x), n) for x in v]


def _mesh_arrays(m, mesh_id: int):
    """MuJoCo mesh -> (positions, normals, indices) with one vertex per unique (position, normal) corner."""
    vadr, vnum = int(m.mesh_vertadr[mesh_id]), int(m.mesh_vertnum[mesh_id])
    fadr, fnum = int(m.mesh_faceadr[mesh_id]), int(m.mesh_facenum[mesh_id])
    nadr = int(m.mesh_normaladr[mesh_id])
    verts = np.asarray(m.mesh_vert[vadr : vadr + vnum], dtype=np.float32)
    faces = np.asarray(m.mesh_face[fadr : fadr + fnum], dtype=np.int64)
    fnorm = np.asarray(m.mesh_facenormal[fadr : fadr + fnum], dtype=np.int64)
    normals_all = np.asarray(m.mesh_normal[nadr : nadr + int(m.mesh_normalnum[mesh_id])], dtype=np.float32)
    corners = np.stack([faces.reshape(-1), fnorm.reshape(-1)], axis=1)
    uniq, inverse = np.unique(corners, axis=0, return_inverse=True)
    pos = verts[uniq[:, 0]]
    nrm = normals_all[uniq[:, 1]]
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    return pos, nrm.astype(np.float32), inverse.reshape(-1).astype(np.uint32)


class _GlbWriter:
    def __init__(self) -> None:
        self.bin = bytearray()
        self.buffer_views: list = []
        self.accessors: list = []
        self.meshes: list = []
        self.materials: list = []
        self.nodes: list = []
        self._mat_index: dict = {}

    def _view(self, data: bytes, target: int) -> int:
        while len(self.bin) % 4:
            self.bin.append(0)
        self.buffer_views.append({"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data), "target": target})
        self.bin.extend(data)
        return len(self.buffer_views) - 1

    def _accessor(self, arr: np.ndarray, comp: int, typ: str, target: int, minmax: bool = False) -> int:
        acc = {
            "bufferView": self._view(arr.tobytes(), target),
            "componentType": comp,
            "count": int(arr.shape[0]),
            "type": typ,
        }
        if minmax:
            acc["min"] = _r(arr.min(axis=0))
            acc["max"] = _r(arr.max(axis=0))
        self.accessors.append(acc)
        return len(self.accessors) - 1

    def material(self, rgba) -> int:
        key = tuple(round(float(c), 3) for c in rgba)
        if key not in self._mat_index:
            self.materials.append(
                {
                    "name": "rgba_" + "_".join(f"{c:.2f}" for c in key),
                    "pbrMetallicRoughness": {
                        "baseColorFactor": list(key),
                        "metallicFactor": 0.35,
                        "roughnessFactor": 0.45,
                    },
                }
            )
            self._mat_index[key] = len(self.materials) - 1
        return self._mat_index[key]

    def mesh(self, name: str, pos: np.ndarray, nrm: np.ndarray, idx: np.ndarray, material: int) -> int:
        prim = {
            "attributes": {
                "POSITION": self._accessor(pos, 5126, "VEC3", 34962, minmax=True),
                "NORMAL": self._accessor(nrm, 5126, "VEC3", 34962),
            },
            "indices": self._accessor(idx, 5125, "SCALAR", 34963),
            "material": material,
        }
        self.meshes.append({"name": name, "primitives": [prim]})
        return len(self.meshes) - 1

    def node(self, **kw) -> int:
        self.nodes.append({k: v for k, v in kw.items() if v is not None})
        return len(self.nodes) - 1

    def write(self, path: Path, scene_nodes: list) -> None:
        while len(self.bin) % 4:
            self.bin.append(0)
        gltf = {
            "asset": {"version": "2.0", "generator": "PoRobotTrain scripts/build_web_twin.py"},
            "scene": 0,
            "scenes": [{"name": "r1_stand", "nodes": scene_nodes}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "accessors": self.accessors,
            "bufferViews": self.buffer_views,
            "buffers": [{"byteLength": len(self.bin)}],
        }
        js = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
        js += b" " * ((4 - len(js) % 4) % 4)
        total = 12 + 8 + len(js) + 8 + len(self.bin)
        with open(path, "wb") as f:
            f.write(struct.pack("<III", 0x46546C67, 2, total))
            f.write(struct.pack("<II", len(js), 0x4E4F534A))
            f.write(js)
            f.write(struct.pack("<II", len(self.bin), 0x004E4942))
            f.write(bytes(self.bin))


def _geom_rgba(m, g: int):
    mat = int(m.geom_matid[g])
    return m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]


def build(compress: bool) -> dict:
    import mujoco

    jm = JointMap.load()
    m = mujoco.MjModel.from_xml_path(str(MJCF_DIR / "scene_stand.xml"))
    d = mujoco.MjData(m)
    home = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0:
        mujoco.mj_resetDataKeyframe(m, d, home)
    mujoco.mj_forward(m, d)

    def name(obj, i):
        return mujoco.mj_id2name(m, obj, i) or ""

    pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    if pelvis < 0:
        raise SystemExit("scene_stand.xml has no 'pelvis' body")

    # ---- joints, in policy order (r1_joint_map.yaml is the source of truth)
    joints = []
    body_joint = {}
    for pi, jname in enumerate(jm.names):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            raise SystemExit(f"joint map joint {jname!r} missing from scene_stand.xml")
        if int(m.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise SystemExit(f"{jname} is not a hinge")
        if np.abs(m.jnt_pos[jid]).max() > 1e-9:
            raise SystemExit(f"{jname} has a joint offset; the browser twin assumes joints at body origins")
        bid = int(m.jnt_bodyid[jid])
        body_joint[bid] = pi
        joints.append(
            {
                "name": jname,
                "policyIndex": pi,
                "body": name(mujoco.mjtObj.mjOBJ_BODY, bid),
                "axis": _r(m.jnt_axis[jid]),
                "minDeg": round(float(np.degrees(jm.limits_lo[pi])), 3),
                "maxDeg": round(float(np.degrees(jm.limits_hi[pi])), 3),
                "defaultDeg": round(float(np.degrees(jm.defaults[pi])), 3),
            }
        )

    # ---- body tree under the pelvis (pelvis gets its welded home world pose)
    robot_bodies = [b for b in range(m.nbody) if b == pelvis or _is_descendant(m, b, pelvis)]
    bodies = []
    for b in robot_bodies:
        if b == pelvis:
            pos, quat = d.xpos[b], d.xquat[b]
        else:
            pos, quat = m.body_pos[b], m.body_quat[b]
        bodies.append(
            {
                "name": name(mujoco.mjtObj.mjOBJ_BODY, b),
                "parent": None if b == pelvis else name(mujoco.mjtObj.mjOBJ_BODY, int(m.body_parentid[b])),
                "pos": _r(pos),
                "quat": _wxyz_to_xyzw(quat),
                "joint": body_joint.get(b),
            }
        )

    # ---- glTF: one node per body, visual mesh geoms as child nodes
    w = _GlbWriter()
    mesh_cache: dict = {}
    body_node: dict = {}
    for b in robot_bodies:  # MuJoCo bodies are topologically ordered (parents first)
        info = next(x for x in bodies if x["name"] == name(mujoco.mjtObj.mjOBJ_BODY, b))
        children = []
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) != b or int(m.geom_type[g]) != GEOM_MESH:
                continue
            if int(m.geom_group[g]) not in VISUAL_GROUPS:
                continue
            mid = int(m.geom_dataid[g])
            mat = w.material(_geom_rgba(m, g))
            key = (mid, mat)
            if key not in mesh_cache:
                pos, nrm, idx = _mesh_arrays(m, mid)
                mesh_cache[key] = w.mesh(name(mujoco.mjtObj.mjOBJ_MESH, mid), pos, nrm, idx, mat)
            children.append(
                w.node(
                    name=f"geom:{g}",
                    mesh=mesh_cache[key],
                    translation=_r(m.geom_pos[g]),
                    rotation=_wxyz_to_xyzw(m.geom_quat[g]),
                )
            )
        extras = {"joint": jm.names[info["joint"]]} if info["joint"] is not None else None
        body_node[b] = w.node(
            name=info["name"], translation=info["pos"], rotation=info["quat"], children=children or None, extras=extras
        )
    for b in robot_bodies:
        if b == pelvis:
            continue
        parent = w.nodes[body_node[int(m.body_parentid[b])]]
        parent.setdefault("children", []).append(body_node[b])

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    raw_glb = OUT_DIR / "r1_stand.raw.glb"
    glb = OUT_DIR / "r1_stand.glb"
    w.write(raw_glb, [body_node[pelvis]])
    if compress:
        _compress(raw_glb, glb)
        raw_glb.unlink()
    else:
        shutil.move(raw_glb, glb)

    # ---- collision geoms (MuJoCo's own contact model) and the stand
    def geom_entry(g):
        typ = int(m.geom_type[g])
        return {
            "body": name(mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])),
            "type": PRIM_NAMES[typ],
            "size": _r(m.geom_size[g]),
            "pos": _r(m.geom_pos[g]),
            "quat": _wxyz_to_xyzw(m.geom_quat[g]),
            "contype": int(m.geom_contype[g]),
            "conaffinity": int(m.geom_conaffinity[g]),
            "rgba": _r(_geom_rgba(m, g), 3),
        }

    colliders, stand = [], []
    for g in range(m.ngeom):
        typ = int(m.geom_type[g])
        if typ not in PRIM_NAMES:
            continue
        bid = int(m.geom_bodyid[g])
        if bid in robot_bodies and int(m.geom_group[g]) == COLLISION_GROUP:
            colliders.append(geom_entry(g))
        elif bid not in robot_bodies and int(m.geom_group[g]) in VISUAL_GROUPS:
            e = geom_entry(g)
            # world-frame pose for geoms on static bodies
            e["pos"] = _r(d.geom_xpos[g])
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, d.geom_xmat[g])
            e["quat"] = _wxyz_to_xyzw(q)
            stand.append(e)

    fk_golden = _fk_golden(m, d, jm, robot_bodies, home)

    policy_src = POLICY_DIR / "policy.onnx"
    policy_dst = OUT_DIR / "policy.onnx"
    shutil.copyfile(policy_src, policy_dst)
    golden = _policy_golden(policy_src)
    (OUT_DIR / "policy_golden.json").write_text(json.dumps(golden, separators=(",", ":")), encoding="utf-8")

    manifest = {
        "schema": 1,
        "model": "UnrealProject/MJCF/scene_stand.xml",
        "jointMap": "RobotControl/configs/r1_joint_map.yaml",
        "units": {"length": "m", "angle": "deg (joints), quaternions xyzw"},
        "up": "z",
        "joints": joints,
        "bodies": bodies,
        "colliders": colliders,
        "stand": stand,
        "fkGolden": fk_golden,
        "policy": {
            "file": "policy.onnx",
            "sha256": _sha256(policy_dst),
            "sourceSha256": _sha256(policy_src),
            "obsDim": golden["obsDim"],
            "actDim": golden["actDim"],
        },
        "glb": {"file": "r1_stand.glb", "sha256": _sha256(glb), "bytes": glb.stat().st_size, "draco": compress},
    }
    (OUT_DIR / "twin_manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    return manifest


def _is_descendant(m, b: int, root: int) -> bool:
    while b > 0:
        b = int(m.body_parentid[b])
        if b == root:
            return True
    return False


def _fk_golden(m, d, jm: JointMap, robot_bodies, home: int) -> list:
    """MuJoCo world poses of every robot body for a few joint configurations (degrees, policy order)."""
    import mujoco

    rng = np.random.default_rng(7)
    configs = [np.degrees(jm.defaults)]
    for _ in range(2):
        u = rng.uniform(0.15, 0.85, size=len(jm.names))
        configs.append(np.degrees(jm.limits_lo + u * (jm.limits_hi - jm.limits_lo)))
    out = []
    for qdeg in configs:
        if home >= 0:
            mujoco.mj_resetDataKeyframe(m, d, home)
        for pi, jname in enumerate(jm.names):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
            d.qpos[m.jnt_qposadr[jid]] = np.radians(qdeg[pi])
        mujoco.mj_kinematics(m, d)
        out.append(
            {
                "qDeg": _r(qdeg, 4),
                "bodies": {
                    mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b): {
                        "pos": _r(d.xpos[b]),
                        "quat": _wxyz_to_xyzw(d.xquat[b]),
                    }
                    for b in robot_bodies
                },
            }
        )
    return out


def _policy_golden(policy: Path, n: int = 16) -> dict:
    import onnxruntime as ort

    roll = np.load(POLICY_DIR / "baseline_rollout.npz")
    obs = np.asarray(roll["obs"], dtype=np.float32)
    pick = np.linspace(0, len(obs) - 1, n).round().astype(int)
    obs = obs[pick]
    sess = ort.InferenceSession(str(policy), providers=["CPUExecutionProvider"])
    inp, out = sess.get_inputs()[0].name, sess.get_outputs()[0].name
    act = sess.run([out], {inp: obs})[0]
    return {
        "input": inp,
        "output": out,
        "obsDim": int(obs.shape[1]),
        "actDim": int(act.shape[1]),
        "tolerance": 1e-4,
        "obs": [[float(x) for x in row] for row in obs],
        "actions": [[float(x) for x in row] for row in act],
        "source": "RobotControl/models/r1_level0_upperbody/baseline_rollout.npz + onnxruntime CPU",
    }


def _compress(src: Path, dst: Path) -> None:
    npx = shutil.which("npx") or shutil.which("npx.cmd")
    if npx is None:
        raise SystemExit("--compress needs Node.js (npx) for gltf-transform")
    tmp = dst.with_suffix(".weld.glb")
    steps = [
        ["weld", str(src), str(tmp)],
        ["simplify", str(tmp), str(tmp), "--ratio", "0.5", "--error", "0.0004"],
        ["draco", str(tmp), str(dst)],
    ]
    for args in steps:
        subprocess.run([npx, "--yes", GLTF_TRANSFORM, *args], check=True)
    tmp.unlink()


def _sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--compress", action="store_true", help="weld + simplify + Draco via gltf-transform (needs npx)")
    a = ap.parse_args()
    man = build(a.compress)
    print(
        f"twin assets -> {OUT_DIR.relative_to(ROOT)}: {len(man['joints'])} joints, {len(man['bodies'])} bodies, "
        f"{len(man['colliders'])} colliders, glb {man['glb']['bytes'] / 1e6:.2f} MB (draco={man['glb']['draco']})"
    )


if __name__ == "__main__":
    main()

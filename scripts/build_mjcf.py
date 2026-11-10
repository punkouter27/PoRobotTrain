"""Build the R1 digital-twin MJCF files from Unitree's official model + the joint map.

Inputs : external/unitree_rl_mjlab/src/assets/robots/unitree_r1/xmls/r1.xml  (body tree, collision capsules)
         external/unitree_ros_r1/R1.urdf                                      (head joint origins/inertials, camera)
         RobotControl/configs/r1_joint_map.yaml                               (limits, gains, defaults)
Outputs: UnrealProject/MJCF/r1_twin.xml         robot only, pelvis free, standing height
         UnrealProject/MJCF/r1_twin_stand.xml   robot welded to a gantry stand (Level 0)
         UnrealProject/MJCF/scene_ground.xml    floor + light + robot
         UnrealProject/MJCF/scene_stand.xml     floor + light + stand + robot

Changes vs. Unitree's MJCF:
  * head_pitch / head_yaw become real bodies+joints (URDF origins), head meshes & collision move onto them
  * <position> actuators with per-joint kp/kv/forcerange from the joint map (26, no dummy motors)
  * head camera + site matching the URDF camera_link, IMU quat sensor, subtree angmom
  * 'home' keyframe = Unitree default standing pose
  * joint_pos / joint_vel / actuator force sensors for every joint (URLab streams sensors to Python)
"""
from __future__ import annotations

import copy
import pathlib
import xml.etree.ElementTree as ET

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
BASE_XML = ROOT / "external/unitree_rl_mjlab/src/assets/robots/unitree_r1/xmls/r1.xml"
JOINT_MAP = ROOT / "RobotControl/configs/r1_joint_map.yaml"
OUT_DIR = ROOT / "UnrealProject/MJCF"

STAND_HEIGHT = 1.00   # pelvis z when hanging on the stand (feet ~0.3 m above floor)
GROUND_HEIGHT = 0.7332  # pelvis z with foot capsules touching the floor (Unitree default 0.74 floats 7 mm)
GREY = "0.79216 0.81961 0.93333 1"
STAND_RGBA = "0.25 0.25 0.28 1"


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for e in elem:
            indent(e, level + 1)
        if not e.tail or not e.tail.strip():
            e.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def build_robot(jm: dict, pelvis_z: float, with_stand: bool) -> ET.Element:
    tree = ET.parse(BASE_XML)
    mj = tree.getroot()
    mj.set("model", "r1_twin_stand" if with_stand else "r1_twin")
    mj.find("compiler").set("meshdir", "meshes")
    mj.find("compiler").set("autolimits", "true")

    # 5 ms step + implicitfast integrator: identical to mjlab training settings.
    opt = ET.Element("option", timestep="0.005", integrator="implicitfast",
                     iterations="10", ls_iterations="20")
    mj.insert(1, opt)

    # Drop contact excludes that reference wrist_pitch bodies absent from the A5 model.
    contact = mj.find("contact")
    for ex in list(contact):
        if "wrist_pitch" in ex.get("body1", "") + ex.get("body2", ""):
            contact.remove(ex)

    wb = mj.find("worldbody")
    pelvis = wb.find("body[@name='pelvis']")
    pelvis.set("pos", fmt([0, 0, pelvis_z]))
    torso = pelvis.find(".//body[@name='torso_link']")

    # Head: remove fixed head geoms from torso, add articulated head bodies from the URDF.
    for g in list(torso.findall("geom")):
        if g.get("mesh") in ("head_pitch_link", "head_yaw_link") or g.get("name") == "head_collision":
            torso.remove(g)
    head_pitch = ET.SubElement(torso, "body", name="head_pitch_link", pos="-0.006 0.03155 0.255")
    ET.SubElement(head_pitch, "inertial", pos="-0.00006956 -0.03077972 0.08058589", mass="0.50391253",
                  fullinertia="0.00095995 0.00091344 0.00045356 -0.00000045 0.00000624 0.00000588")
    ET.SubElement(head_pitch, "joint", name="head_pitch_joint", axis="0 1 0", range="-0.62832 0.62832")
    ET.SubElement(head_pitch, "geom", attrib={"class": "visual", "rgba": GREY, "mesh": "head_pitch_link"})
    head_yaw = ET.SubElement(head_pitch, "body", name="head_yaw_link", pos="0.0010126 -0.03155 0.1165")
    ET.SubElement(head_yaw, "inertial", pos="0.02211657 -0.00004725 0.00199411", mass="0.7143224",
                  fullinertia="0.00168182 0.00202452 0.00181448 0.0000001 0.00029553 0.0000001")
    ET.SubElement(head_yaw, "joint", name="head_yaw_joint", axis="0 0 1", range="-2.0071 2.0071")
    ET.SubElement(head_yaw, "geom", attrib={"class": "visual", "rgba": GREY, "mesh": "head_yaw_link"})
    ET.SubElement(head_yaw, "geom", attrib={"name": "head_collision", "class": "collision",
                                            "type": "sphere", "size": "0.055", "pos": "0.0199874 0 -0.0215"})
    # URDF camera_link: xyz 0.066971 0 -0.021637, pitched down 20 deg (0.34907 rad).
    # MuJoCo cameras look along -z with y up, hence the xyaxes below.
    cam_pos = "0.066971 0 -0.021637"
    ET.SubElement(head_yaw, "site", name="head_camera", pos=cam_pos, euler="0 0.34907 0",
                  size="0.005", rgba="0 1 0 1")
    ET.SubElement(head_yaw, "camera", name="head_rgbd", pos=cam_pos,
                  xyaxes="0 -1 0 0.34202 0 0.93969", fovy="58")

    if with_stand:
        stand = ET.SubElement(wb, "body", name="stand", pos="0 0 0")
        post_h = pelvis_z + 0.45
        nocol = {"contype": "0", "conaffinity": "0", "rgba": STAND_RGBA}
        for y in (-0.45, 0.45):
            ET.SubElement(stand, "geom", type="capsule", size="0.025",
                          fromto=fmt([-0.35, y, 0.02, -0.35, y, post_h]), **nocol)
            ET.SubElement(stand, "geom", type="box", size="0.25 0.06 0.02",
                          pos=fmt([-0.35, y, 0.02]), **nocol)
        ET.SubElement(stand, "geom", type="capsule", size="0.025",
                      fromto=fmt([-0.35, -0.45, post_h, -0.35, 0.45, post_h]), **nocol)
        ET.SubElement(stand, "geom", type="capsule", size="0.02",
                      fromto=fmt([-0.35, 0, post_h, 0.0, 0, post_h]), **nocol)
        ET.SubElement(stand, "geom", type="capsule", size="0.012",
                      fromto=fmt([0, 0, post_h, 0, 0, pelvis_z + 0.33]),
                      contype="0", conaffinity="0", rgba="0.6 0.6 0.1 1")
        eq = ET.SubElement(mj, "equality")
        # Weld pelvis to world at its qpos0 pose (relpose omitted => initial configuration).
        ET.SubElement(eq, "weld", name="stand_weld", body1="pelvis", body2="world",
                      solref="0.005 1", solimp="0.99 0.999 0.001")

    # Position servos: the MJCF twin of mjlab's BuiltinPositionActuator / the robot's PD loop.
    # On the stand the legs hang in passive damping (kp 0, kd 3), exactly as in Level 0 training and the
    # real-robot procedure. Baking this into the stand MJCF makes engines that cannot change actuator gains at
    # runtime (URLab raw mode) match training. Tools that set gains per command (training env, MujocoInterface)
    # override these values anyway.
    passive = set(jm["groups"]["left_leg"] + jm["groups"]["right_leg"]) if with_stand else set()
    act = ET.SubElement(mj, "actuator")
    for i, j in enumerate(jm["joints"]):
        lo, hi = j["limits"]
        kp, kv = (0.0, 3.0) if i in passive else (j["kp"], j["kd"])
        ET.SubElement(act, "position", name=j["name"].replace("_joint", ""), joint=j["name"],
                      kp=str(kp), kv=str(kv), ctrlrange=fmt([lo, hi]),
                      forcerange=fmt([-j["effort"], j["effort"]]))

    sens = mj.find("sensor")
    ET.SubElement(sens, "framequat", name="imu_quat", objtype="site", objname="imu")
    for j in jm["joints"]:
        n = j["name"].replace("_joint", "")
        ET.SubElement(sens, "jointpos", name=f"{n}_pos", joint=j["name"])
        ET.SubElement(sens, "jointvel", name=f"{n}_vel", joint=j["name"])
        ET.SubElement(sens, "actuatorfrc", name=f"{n}_frc", actuator=n)

    qpos = [0, 0, pelvis_z, 1, 0, 0, 0] + [j["default"] for j in jm["joints"]]
    key = ET.SubElement(mj, "keyframe")
    ET.SubElement(key, "key", name="home", qpos=fmt(qpos), ctrl=fmt([j["default"] for j in jm["joints"]]))
    return mj


def add_scene(mj: ET.Element, name: str) -> ET.Element:
    mj = copy.deepcopy(mj)
    mj.set("model", name)
    vis = ET.Element("visual")
    ET.SubElement(vis, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0")
    ET.SubElement(vis, "global", azimuth="140", elevation="-20", offwidth="1280", offheight="720")
    mj.insert(2, vis)
    asset = mj.find("asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7",
                  rgb2="0 0 0", width="512", height="3072")
    ET.SubElement(asset, "texture", type="2d", name="groundplane", builtin="checker", mark="edge",
                  rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3", markrgb="0.8 0.8 0.8", width="300", height="300")
    ET.SubElement(asset, "material", name="groundplane", texture="groundplane", texuniform="true",
                  texrepeat="5 5", reflectance="0.2")
    wb = mj.find("worldbody")
    wb.insert(0, ET.Element("light", pos="0 0 3.5", dir="0 0 -1", directional="true"))
    wb.insert(1, ET.Element("geom", name="floor", type="plane", size="0 0 0.05", material="groundplane",
                            friction="0.9 0.005 0.0001", condim="3"))
    return mj


def strip_visuals(mj: ET.Element, name: str) -> ET.Element:
    """Training variant: drop mesh assets and visual geoms (density 0, no collision => physics identical).
    Keeps per-env MjModel copies small enough for hundreds of parallel environments."""
    mj = copy.deepcopy(mj)
    mj.set("model", name)
    asset = mj.find("asset")
    for mesh in list(asset.findall("mesh")):
        asset.remove(mesh)
    for parent in mj.iter():
        for g in list(parent.findall("geom")):
            if g.get("class") == "visual" or g.get("mesh") is not None:
                parent.remove(g)
    return mj


def write(mj: ET.Element, path: pathlib.Path):
    indent(mj)
    path.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(mj, encoding="unicode"),
                    encoding="utf-8")
    print("wrote", path.relative_to(ROOT))


if __name__ == "__main__":
    jm = yaml.safe_load(JOINT_MAP.read_text())
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ground = build_robot(jm, GROUND_HEIGHT, with_stand=False)
    stand = build_robot(jm, STAND_HEIGHT, with_stand=True)
    write(ground, OUT_DIR / "r1_twin.xml")
    write(stand, OUT_DIR / "r1_twin_stand.xml")
    write(add_scene(ground, "scene_ground"), OUT_DIR / "scene_ground.xml")
    write(add_scene(stand, "scene_stand"), OUT_DIR / "scene_stand.xml")
    write(strip_visuals(stand, "r1_twin_stand_train"), OUT_DIR / "r1_twin_stand_train.xml")
    write(strip_visuals(ground, "r1_twin_train"), OUT_DIR / "r1_twin_train.xml")

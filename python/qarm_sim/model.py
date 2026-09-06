from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import xacro
from numpy.typing import NDArray

from qarm_sim.config import DEFAULT_XACRO, JointMap, MotorParameters


@dataclass
class ArmScene:
    model: mujoco.MjModel
    data: mujoco.MjData
    joint_names: tuple[str, ...]
    joint_ids: NDArray[np.int32]
    qpos_addresses: NDArray[np.int32]
    dof_addresses: NDArray[np.int32]
    tool_site_id: int
    mjcf: str


def _mesh_assets(xacro_path: Path) -> dict[str, bytes]:
    mesh_root = xacro_path.parent / "meshes/visual"
    assets = {
        f"meshes/visual/{path.name}": path.read_bytes()
        for path in mesh_root.glob("*.stl")
    }
    if not assets:
        raise FileNotFoundError(f"no STL meshes found under {mesh_root}")
    return assets


def _expanded_urdf(xacro_path: Path) -> str:
    root = ET.fromstring(xacro.process_file(str(xacro_path)).toxml())
    extension = root.find("mujoco")
    if extension is None:
        extension = ET.SubElement(root, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.set("discardvisual", "false")
    compiler.set("fusestatic", "false")
    return ET.tostring(root, encoding="unicode")


def _normalize_to_mjcf(urdf: str, assets: dict[str, bytes]) -> ET.Element:
    imported = mujoco.MjModel.from_xml_string(urdf, assets=assets)
    with tempfile.NamedTemporaryFile(suffix=".xml") as output:
        mujoco.mj_saveLastXML(output.name, imported)
        return ET.parse(output.name).getroot()


def _body(root: ET.Element, name: str) -> ET.Element:
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise ValueError(f"normalized MuJoCo model has no body {name!r}")
    return body


def _configure_mjcf(
    root: ET.Element,
    mapping: JointMap,
    motor: MotorParameters,
) -> None:
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.001")
    option.set("gravity", "0 0 -9.80665")
    option.set("integrator", "implicitfast")
    option.set("solver", "Newton")
    option.set("iterations", "100")

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "1920")
    global_visual.set("offheight", "1080")
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ET.SubElement(visual, "headlight")
    headlight.set("ambient", "0.35 0.35 0.35")
    headlight.set("diffuse", "0.8 0.8 0.8")
    headlight.set("specular", "0.25 0.25 0.25")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("normalized MuJoCo model has no worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": "0 0 -0.0233",
            "size": "1.5 1.5 0.05",
            "rgba": "0.18 0.20 0.23 1",
            "friction": "0.8 0.01 0.0001",
            "contype": "1",
            "conaffinity": "1",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "key_light",
            "pos": "1.5 -1.5 2.5",
            "dir": "-0.3 0.3 -1",
            "diffuse": "0.9 0.9 0.9",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "overview",
            "mode": "targetbody",
            "target": "link_3",
            "pos": "0.95 -1.20 0.72",
        },
    )

    tool_body = _body(root, "tool0")
    ET.SubElement(
        tool_body,
        "site",
        {
            "name": "tool0",
            "type": "sphere",
            "size": "0.012",
            "rgba": "0.95 0.25 0.10 1",
            "group": "3",
        },
    )

    actuator = ET.SubElement(root, "actuator")
    sensor = ET.SubElement(root, "sensor")
    for joint_name in mapping.joint_names:
        joint = root.find(f".//joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"normalized MuJoCo model has no joint {joint_name!r}")
        # CAD-derived damping is retained. Motor rotor inertia, dry friction,
        # and backlash remain zero until they are identified on hardware.
        joint.set("armature", "0")
        joint.set("frictionloss", "0")
        name = f"{joint_name}_motor"
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": name,
                "joint": joint_name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": (
                    f"{-motor.peak_torque_nm} {motor.peak_torque_nm}"
                ),
            },
        )
        ET.SubElement(
            sensor,
            "jointpos",
            {"name": f"{joint_name}_position", "joint": joint_name},
        )
        ET.SubElement(
            sensor,
            "jointvel",
            {"name": f"{joint_name}_velocity", "joint": joint_name},
        )
        ET.SubElement(
            sensor,
            "actuatorfrc",
            {"name": f"{joint_name}_torque", "actuator": name},
        )
    ET.SubElement(
        sensor,
        "framepos",
        {"name": "tool0_position", "objtype": "site", "objname": "tool0"},
    )
    ET.SubElement(
        sensor,
        "framequat",
        {"name": "tool0_orientation", "objtype": "site", "objname": "tool0"},
    )


def build_scene(
    *,
    xacro_path: Path = DEFAULT_XACRO,
    mapping: JointMap | None = None,
    motor: MotorParameters | None = None,
) -> ArmScene:
    mapping = JointMap.load() if mapping is None else mapping
    motor = MotorParameters.load() if motor is None else motor
    assets = _mesh_assets(xacro_path)
    root = _normalize_to_mjcf(_expanded_urdf(xacro_path), assets)
    _configure_mjcf(root, mapping, motor)
    mjcf = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(mjcf, assets=assets)
    if model.nq != len(mapping.joint_names) or model.nv != len(mapping.joint_names):
        raise ValueError("joint map must cover every actuated joint in the MuJoCo model")
    joint_ids = np.array(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in mapping.joint_names
        ],
        dtype=np.int32,
    )
    if np.any(joint_ids < 0):
        raise ValueError("one or more arm joints are absent after MuJoCo import")
    tool_site_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, "tool0"
    )
    if tool_site_id < 0:
        raise ValueError("tool0 site is absent after MuJoCo import")
    if model.nu != len(mapping.joint_names):
        raise ValueError(
            f"expected {len(mapping.joint_names)} actuators, found {model.nu}"
        )
    return ArmScene(
        model=model,
        data=mujoco.MjData(model),
        joint_names=mapping.joint_names,
        joint_ids=joint_ids,
        qpos_addresses=model.jnt_qposadr[joint_ids].copy(),
        dof_addresses=model.jnt_dofadr[joint_ids].copy(),
        tool_site_id=tool_site_id,
        mjcf=mjcf,
    )


def set_mirrored_state(
    scene: ArmScene,
    position: NDArray[np.float64],
    velocity: NDArray[np.float64] | None = None,
) -> None:
    position = np.asarray(position, dtype=np.float64)
    if position.shape != scene.qpos_addresses.shape:
        raise ValueError(
            f"position shape {position.shape} does not match {len(scene.joint_names)}-joint model"
        )
    scene.data.qpos[scene.qpos_addresses] = position
    if velocity is None:
        scene.data.qvel[scene.dof_addresses] = 0.0
    else:
        velocity = np.asarray(velocity, dtype=np.float64)
        if velocity.shape != scene.dof_addresses.shape:
            raise ValueError(
                f"velocity shape {velocity.shape} does not match model"
            )
        scene.data.qvel[scene.dof_addresses] = velocity
    mujoco.mj_forward(scene.model, scene.data)

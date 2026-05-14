"""URDF resolver for robot dynamics models.

The resolver keeps URDF parsing separate from simulation/model code.  It
extracts the robot structure and inertial data into plain dataclasses, and it
keeps payload/gripper parameters explicit instead of depending on an implicit
URDF choice.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import copy
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


REVOLUTE_TYPES = {"revolute", "continuous"}


@dataclass(frozen=True)
class InertialSpec:
    mass: float
    com_xyz_m: np.ndarray
    inertia_kg_m2: np.ndarray
    rpy_rad: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


@dataclass(frozen=True)
class ResolvedLink:
    name: str
    inertial: InertialSpec | None = None


@dataclass(frozen=True)
class JointLimit:
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None


@dataclass(frozen=True)
class JointDynamics:
    damping: float = 0.0
    friction: float = 0.0


@dataclass(frozen=True)
class ResolvedJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz_m: np.ndarray
    origin_rpy_rad: np.ndarray
    axis: np.ndarray
    limit: JointLimit | None = None
    dynamics: JointDynamics = field(default_factory=JointDynamics)

    @property
    def is_active(self) -> bool:
        return self.joint_type in REVOLUTE_TYPES


@dataclass(frozen=True)
class PayloadSpec:
    name: str
    mass_kg: float
    com_xyz_m: np.ndarray
    inertia_kg_m2: np.ndarray
    attach_link: str = "link6"
    joint_origin_xyz_m: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    joint_origin_rpy_rad: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    apply_to_model: bool = False


@dataclass
class ResolvedRobot:
    name: str
    urdf_path: Path
    links: list[ResolvedLink]
    joints: list[ResolvedJoint]
    payload: PayloadSpec | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def active_joints(self) -> list[ResolvedJoint]:
        return [joint for joint in self.joints if joint.is_active]

    @property
    def joint_count(self) -> int:
        return len(self.active_joints)

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.active_joints]


BUILTIN_PAYLOADS = {
    "none": None,
    "xarm_gripper_g2": {
        "name": "xarm_gripper_g2",
        "mass_kg": 0.78,
        "com_xyz_m": [0.12, -0.04, 0.158],
        "inertia_kg_m2": [
            [0.0008, 0.0, 0.0],
            [0.0, 0.0008, 0.0],
            [0.0, 0.0, 0.0004],
        ],
        "attach_link": "link6",
    },
}


def _array(values: Any, length: int) -> np.ndarray:
    if values is None:
        return np.zeros(length, dtype=np.float64)
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (length,):
        raise ValueError(f"expected {length} values, got shape {arr.shape}")
    return arr


def _matrix(values: Any) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape == (3,):
        return np.diag(arr)
    if arr.shape != (3, 3):
        raise ValueError(f"expected inertia as 3 values or 3x3 matrix, got {arr.shape}")
    return arr


def _attr_float(node: ET.Element | None, name: str) -> float | None:
    if node is None or name not in node.attrib:
        return None
    return float(node.attrib[name])


def _attr_vec(node: ET.Element | None, name: str, length: int = 3) -> np.ndarray:
    if node is None or name not in node.attrib:
        return np.zeros(length, dtype=np.float64)
    return _array([float(x) for x in node.attrib[name].split()], length)


def _parse_inertial(link_node: ET.Element) -> InertialSpec | None:
    inertial = link_node.find("inertial")
    if inertial is None:
        return None
    mass_node = inertial.find("mass")
    inertia_node = inertial.find("inertia")
    if mass_node is None or inertia_node is None:
        return None
    origin = inertial.find("origin")
    inertia = np.array(
        [
            [
                float(inertia_node.attrib.get("ixx", "0")),
                float(inertia_node.attrib.get("ixy", "0")),
                float(inertia_node.attrib.get("ixz", "0")),
            ],
            [
                float(inertia_node.attrib.get("ixy", "0")),
                float(inertia_node.attrib.get("iyy", "0")),
                float(inertia_node.attrib.get("iyz", "0")),
            ],
            [
                float(inertia_node.attrib.get("ixz", "0")),
                float(inertia_node.attrib.get("iyz", "0")),
                float(inertia_node.attrib.get("izz", "0")),
            ],
        ],
        dtype=np.float64,
    )
    return InertialSpec(
        mass=float(mass_node.attrib["value"]),
        com_xyz_m=_attr_vec(origin, "xyz"),
        rpy_rad=_attr_vec(origin, "rpy"),
        inertia_kg_m2=inertia,
    )


def _parse_link(link_node: ET.Element) -> ResolvedLink:
    return ResolvedLink(
        name=link_node.attrib["name"],
        inertial=_parse_inertial(link_node),
    )


def _parse_joint(joint_node: ET.Element) -> ResolvedJoint:
    parent = joint_node.find("parent")
    child = joint_node.find("child")
    if parent is None or child is None:
        raise ValueError(f"joint {joint_node.attrib.get('name', '<unnamed>')} missing parent/child")
    origin = joint_node.find("origin")
    axis = joint_node.find("axis")
    limit_node = joint_node.find("limit")
    dyn_node = joint_node.find("dynamics")
    limit = None
    if limit_node is not None:
        limit = JointLimit(
            lower=_attr_float(limit_node, "lower"),
            upper=_attr_float(limit_node, "upper"),
            effort=_attr_float(limit_node, "effort"),
            velocity=_attr_float(limit_node, "velocity"),
        )
    return ResolvedJoint(
        name=joint_node.attrib["name"],
        joint_type=joint_node.attrib.get("type", "fixed"),
        parent=parent.attrib["link"],
        child=child.attrib["link"],
        origin_xyz_m=_attr_vec(origin, "xyz"),
        origin_rpy_rad=_attr_vec(origin, "rpy"),
        axis=_attr_vec(axis, "xyz") if axis is not None else np.array([0.0, 0.0, 1.0], dtype=np.float64),
        limit=limit,
        dynamics=JointDynamics(
            damping=float(dyn_node.attrib.get("damping", "0")) if dyn_node is not None else 0.0,
            friction=float(dyn_node.attrib.get("friction", "0")) if dyn_node is not None else 0.0,
        ),
    )


def payload_from_config(payload_config: dict[str, Any] | None) -> PayloadSpec | None:
    if not payload_config or payload_config.get("enabled", False) is False:
        return None
    profile_name = payload_config.get("profile", "none")
    profile = BUILTIN_PAYLOADS.get(profile_name)
    if profile is None and profile_name != "none":
        raise ValueError(f"unknown payload profile: {profile_name}")
    merged: dict[str, Any] = copy.deepcopy(profile or {})
    merged.update({k: v for k, v in payload_config.items() if k not in {"enabled", "profile", "mode"}})
    name = str(merged.get("name", profile_name))
    mode = payload_config.get("mode", "metadata_only")
    return PayloadSpec(
        name=name,
        mass_kg=float(merged["mass_kg"]),
        com_xyz_m=_array(merged["com_xyz_m"], 3),
        inertia_kg_m2=_matrix(merged["inertia_kg_m2"]),
        attach_link=str(merged.get("attach_link", "link6")),
        joint_origin_xyz_m=_array(merged.get("joint_origin_xyz_m"), 3),
        joint_origin_rpy_rad=_array(merged.get("joint_origin_rpy_rad"), 3),
        apply_to_model=mode == "augment_urdf",
    )


def resolve_robot(
    urdf_path: str | Path,
    *,
    name: str | None = None,
    payload: PayloadSpec | dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
) -> ResolvedRobot:
    path = Path(urdf_path).expanduser().resolve()
    root = ET.parse(path).getroot()
    robot_name = name or root.attrib.get("name", path.stem)
    payload_spec = payload_from_config(payload) if isinstance(payload, dict) else payload
    return ResolvedRobot(
        name=robot_name,
        urdf_path=path,
        links=[_parse_link(node) for node in root.findall("link")],
        joints=[_parse_joint(node) for node in root.findall("joint")],
        payload=payload_spec,
        metadata=metadata or {},
    )


def write_payload_augmented_urdf(robot: ResolvedRobot) -> Path:
    """Write a temporary URDF with the configured payload appended.

    This is only used when payload.apply_to_model is true.  The caller owns
    cleanup of the returned temporary path.
    """
    if robot.payload is None:
        return robot.urdf_path
    payload = robot.payload
    root = ET.parse(robot.urdf_path).getroot()
    link_name = f"{payload.name}_payload"
    joint_name = f"{payload.name}_payload_joint"

    link = ET.SubElement(root, "link", {"name": link_name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(
        inertial,
        "origin",
        {
            "xyz": " ".join(f"{v:.9g}" for v in payload.com_xyz_m),
            "rpy": "0 0 0",
        },
    )
    ET.SubElement(inertial, "mass", {"value": f"{payload.mass_kg:.9g}"})
    inertia = payload.inertia_kg_m2
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": f"{inertia[0, 0]:.9g}",
            "ixy": f"{inertia[0, 1]:.9g}",
            "ixz": f"{inertia[0, 2]:.9g}",
            "iyy": f"{inertia[1, 1]:.9g}",
            "iyz": f"{inertia[1, 2]:.9g}",
            "izz": f"{inertia[2, 2]:.9g}",
        },
    )
    joint = ET.SubElement(root, "joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": payload.attach_link})
    ET.SubElement(joint, "child", {"link": link_name})
    ET.SubElement(
        joint,
        "origin",
        {
            "xyz": " ".join(f"{v:.9g}" for v in payload.joint_origin_xyz_m),
            "rpy": " ".join(f"{v:.9g}" for v in payload.joint_origin_rpy_rad),
        },
    )

    tmp = tempfile.NamedTemporaryFile(prefix=f"{robot.name}_", suffix=".urdf", delete=False)
    tmp.close()
    out = Path(tmp.name)
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=True)
    return out

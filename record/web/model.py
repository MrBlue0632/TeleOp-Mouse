"""URDF and primitive geometry metadata helpers for the web dashboard.

Inputs: repository root plus an xArm URDF path under ``assets``.
Returns: JSON-safe robot model metadata, joint contracts, and primitive visuals.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


DEFAULT_URDF = Path("assets/urdf/xarm6/xarm6/xarm6.urdf")


def _rel(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def _float_list(text: str | None, size: int | None = None, default: float = 0.0) -> list[float]:
    values: list[float] = []
    for raw in str(text or "").replace(",", " ").split():
        try:
            values.append(float(raw))
        except ValueError:
            values.append(default)
    if size is not None:
        if len(values) < size:
            values.extend([default] * (size - len(values)))
        values = values[:size]
    return values


def _parse_origin(node: ET.Element | None) -> dict[str, list[float]]:
    if node is None:
        return {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    return {
        "xyz": _float_list(node.attrib.get("xyz"), 3),
        "rpy": _float_list(node.attrib.get("rpy"), 3),
    }


def _resolve_mesh(filename: str, *, repo_root: Path, urdf_path: Path) -> tuple[Path | None, str | None]:
    if not filename:
        return None, None
    if filename.startswith("package://"):
        rel = filename.removeprefix("package://")
        candidates = [
            repo_root / "assets" / "urdf" / rel,
            repo_root / "assets" / rel,
            repo_root / rel,
        ]
    elif filename.startswith("file://"):
        candidates = [Path(filename.removeprefix("file://"))]
    else:
        raw = Path(filename)
        candidates = [raw] if raw.is_absolute() else [urdf_path.parent / raw, repo_root / "assets" / raw]

    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate.exists():
            url = None
            try:
                assets_rel = candidate.relative_to(repo_root / "assets").as_posix()
                url = f"/assets/{assets_rel}"
            except ValueError:
                pass
            return candidate, url
    first = candidates[0].resolve()
    return first, None


def _material_lookup(robot: ET.Element) -> dict[str, list[float]]:
    lookup: dict[str, list[float]] = {}
    for material in robot.findall("material"):
        name = material.attrib.get("name")
        color = material.find("color")
        if name and color is not None:
            lookup[name] = _float_list(color.attrib.get("rgba"), 4, 1.0)
    return lookup


def _parse_material(visual: ET.Element, materials: dict[str, list[float]]) -> dict[str, Any]:
    material = visual.find("material")
    if material is None:
        return {"name": None, "rgba": [0.78, 0.78, 0.78, 1.0]}
    name = material.attrib.get("name")
    color = material.find("color")
    rgba = _float_list(color.attrib.get("rgba"), 4, 1.0) if color is not None else materials.get(name or "")
    return {"name": name, "rgba": rgba or [0.78, 0.78, 0.78, 1.0]}


def _geometry_payload(
    geometry: ET.Element | None,
    *,
    repo_root: Path,
    urdf_path: Path,
) -> dict[str, Any] | None:
    if geometry is None:
        return None
    for child in list(geometry):
        tag = child.tag.split("}", 1)[-1]
        if tag == "box":
            return {"type": "box", "size": _float_list(child.attrib.get("size"), 3, 0.04)}
        if tag == "cylinder":
            return {
                "type": "cylinder",
                "radius": float(child.attrib.get("radius", 0.02)),
                "length": float(child.attrib.get("length", 0.04)),
            }
        if tag == "sphere":
            return {"type": "sphere", "radius": float(child.attrib.get("radius", 0.02))}
        if tag == "mesh":
            filename = child.attrib.get("filename", "")
            path, url = _resolve_mesh(filename, repo_root=repo_root, urdf_path=urdf_path)
            return {
                "type": "mesh",
                "filename": filename,
                "path": _rel(path, repo_root) if path else None,
                "exists": bool(path and path.exists()),
                "url": url,
                "scale": _float_list(child.attrib.get("scale"), 3, 1.0),
            }
    return None


def _visual_payloads(
    link: ET.Element,
    *,
    materials: dict[str, list[float]],
    repo_root: Path,
    urdf_path: Path,
) -> list[dict[str, Any]]:
    visuals: list[dict[str, Any]] = []
    for index, visual in enumerate(link.findall("visual")):
        geometry = _geometry_payload(visual.find("geometry"), repo_root=repo_root, urdf_path=urdf_path)
        if geometry is None:
            continue
        visuals.append(
            {
                "name": visual.attrib.get("name") or f"{link.attrib.get('name', 'link')}_visual_{index}",
                "origin": _parse_origin(visual.find("origin")),
                "geometry": geometry,
                "material": _parse_material(visual, materials),
            }
        )
    return visuals


def _joint_payload(joint: ET.Element) -> dict[str, Any]:
    parent = joint.find("parent")
    child = joint.find("child")
    axis = joint.find("axis")
    limit = joint.find("limit")
    limit_payload: dict[str, float] = {}
    if limit is not None:
        for key in ("lower", "upper", "effort", "velocity"):
            if key in limit.attrib:
                try:
                    limit_payload[key] = float(limit.attrib[key])
                except ValueError:
                    pass
    return {
        "name": joint.attrib.get("name", ""),
        "type": joint.attrib.get("type", "fixed"),
        "parent": parent.attrib.get("link") if parent is not None else None,
        "child": child.attrib.get("link") if child is not None else None,
        "origin": _parse_origin(joint.find("origin")),
        "axis": _float_list(axis.attrib.get("xyz"), 3) if axis is not None else [0.0, 0.0, 1.0],
        "limit": limit_payload,
    }


def _robot_contract(robot: ET.Element, *, repo_root: Path, urdf_path: Path) -> dict[str, Any]:
    materials = _material_lookup(robot)
    links = {
        link.attrib.get("name", ""): {
            "name": link.attrib.get("name", ""),
            "visuals": _visual_payloads(link, materials=materials, repo_root=repo_root, urdf_path=urdf_path),
        }
        for link in robot.findall("link")
        if link.attrib.get("name")
    }
    joints = [_joint_payload(joint) for joint in robot.findall("joint")]
    child_links = {joint["child"] for joint in joints if joint.get("child")}
    root_links = [name for name in links if name not in child_links]
    movable_joints = [joint["name"] for joint in joints if joint.get("type") != "fixed"]
    return {
        "links": links,
        "joints": joints,
        "root_links": root_links,
        "movable_joints": movable_joints,
    }


def inspect_robot_model(
    repo_root: str | Path,
    *,
    urdf_rel: str | Path = DEFAULT_URDF,
) -> dict[str, object]:
    """Return dashboard-ready metadata for the robot URDF and visual assets."""
    root_dir = Path(repo_root).resolve()
    urdf_path = (root_dir / urdf_rel).resolve()
    if root_dir not in urdf_path.parents and urdf_path != root_dir:
        return {"ok": False, "error": "urdf path escapes repository root"}
    if not urdf_path.exists():
        return {
            "ok": False,
            "error": "urdf not found",
            "urdf_path": _rel(urdf_path, root_dir),
            "render_mode": "unavailable",
        }

    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError as exc:
        return {
            "ok": False,
            "error": str(exc),
            "urdf_path": _rel(urdf_path, root_dir),
            "render_mode": "unavailable",
        }

    robot = tree.getroot()
    geometry_counts: dict[str, int] = {}
    mesh_files: list[dict[str, object]] = []
    for geometry in robot.findall(".//geometry"):
        for child in list(geometry):
            tag = child.tag.split("}", 1)[-1]
            geometry_counts[tag] = geometry_counts.get(tag, 0) + 1
            if tag == "mesh":
                filename = child.attrib.get("filename", "")
                path, url = _resolve_mesh(filename, repo_root=root_dir, urdf_path=urdf_path)
                mesh_files.append(
                    {
                        "filename": filename,
                        "path": _rel(path, root_dir) if path else None,
                        "exists": bool(path and path.exists()),
                        "url": url,
                    }
                )

    joint_count = len(robot.findall(".//joint"))
    movable_joint_count = sum(1 for joint in robot.findall(".//joint") if joint.attrib.get("type") != "fixed")
    mesh_count = len(mesh_files)
    missing_mesh_count = sum(1 for item in mesh_files if not item["exists"])
    if mesh_count == 0:
        render_mode = "primitive"
        note = "Current URDF uses primitive visual geometry; no mesh files were found in assets."
    elif missing_mesh_count:
        render_mode = "mesh_missing"
        note = "URDF references mesh files, but at least one mesh asset is missing."
    else:
        render_mode = "mesh"
        note = "URDF references mesh assets that are available through /assets."

    return {
        "ok": True,
        "name": robot.attrib.get("name", "robot"),
        "urdf_path": _rel(urdf_path, root_dir),
        "urdf_url": f"/assets/{_rel(urdf_path, root_dir).removeprefix('assets/')}",
        "link_count": len(robot.findall(".//link")),
        "joint_count": joint_count,
        "movable_joint_count": movable_joint_count,
        "geometry_counts": geometry_counts,
        "geometry_types": sorted(geometry_counts),
        "mesh_count": mesh_count,
        "missing_mesh_count": missing_mesh_count,
        "mesh_files": mesh_files,
        "render_mode": render_mode,
        "note": note,
        **_robot_contract(robot, repo_root=root_dir, urdf_path=urdf_path),
    }

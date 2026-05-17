"""URDF and mesh metadata helpers for the web dashboard."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_URDF = Path("assets/urdf/xarm6/xarm6/xarm6.urdf")


def _rel(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


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


def inspect_robot_model(
    repo_root: str | Path,
    *,
    urdf_rel: str | Path = DEFAULT_URDF,
) -> dict[str, object]:
    """Return dashboard-ready metadata for the robot URDF and any mesh assets."""
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
    }

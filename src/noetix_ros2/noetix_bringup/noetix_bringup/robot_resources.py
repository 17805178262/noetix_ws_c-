from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class RobotResources:
    name: str
    robot_root: Path
    dof: int
    real_description: Path
    real_rviz: Optional[Path]


def _optional_resolve(robot_root: Path, value: Optional[str]) -> Optional[Path]:
    if value is None or str(value).strip() == "":
        return None

    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = robot_root / path
    path = path.resolve()
    return path if path.is_file() else None


def _require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise RuntimeError(f"{label} not found: {path}")
    return path


def load_robot_resource(robot_name: str) -> RobotResources:
    """Load one installed noetix_robots/<robot_name> resource.

    This is the single resource-resolution entry point used by launch files.
    Launch files must never directly use raw relative path strings from YAML.
    """
    if not robot_name or not robot_name.strip():
        raise RuntimeError("robot_name is empty")

    robots_share = Path(
        get_package_share_directory("noetix_robots")
    ).resolve()

    robot_root = (robots_share / robot_name).resolve()
    config_dir = robot_root / "config"

    if not robot_root.is_dir():
        raise RuntimeError(
            f"Robot resource directory not found: {robot_root}")

    robot_yaml = _require_file(config_dir / "robot.yaml", "robot.yaml")
    _require_file(config_dir / "hardware.yaml", "hardware.yaml")

    with robot_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    robot = data.get("robot") or {}

    configured_name = str(robot.get("name", robot_name))
    if configured_name != robot_name:
        raise RuntimeError(
            f"robot.yaml name mismatch: requested={robot_name}, "
            f"configured={configured_name}"
        )

    joint_order = list(robot.get("joint_order") or [])
    if not joint_order:
        raise RuntimeError(
            f"robot.joint_order is empty/missing: {robot_yaml}")

    if len(set(joint_order)) != len(joint_order):
        raise RuntimeError(
            f"robot.joint_order contains duplicate names: {robot_yaml}")

    configured_dof = robot.get("dof")
    if configured_dof is not None and int(configured_dof) != len(joint_order):
        raise RuntimeError(
            f"robot.dof={configured_dof} != "
            f"joint_order size={len(joint_order)}: {robot_yaml}"
        )

    description = robot.get("description") or {}
    if not description.get("real"):
        raise RuntimeError(
            f"robot.description.real is missing: {robot_yaml}")

    def resolve_required(value: str, label: str) -> Path:
        p = Path(str(value)).expanduser()
        if not p.is_absolute():
            p = robot_root / p
        p = p.resolve()
        return _require_file(p, label)

    real_description = resolve_required(
        description["real"], "real robot description")
    visualization = robot.get("visualization") or {}

    real_rviz = _optional_resolve(
        robot_root, visualization.get("real_rviz"))

    return RobotResources(
        name=robot_name,
        robot_root=robot_root,
        dof=len(joint_order),
        real_description=real_description,
        real_rviz=real_rviz,
    )

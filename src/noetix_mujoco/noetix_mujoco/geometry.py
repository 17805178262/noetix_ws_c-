"""URDF geometric helpers adapted from the existing noetix_gazebo/model.py.
The geometry calculation matches Gazebo zero-pose spawn clearance.
"""
from __future__ import annotations
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

def _vec(text: str | None, n: int, default: float = 0.0) -> list[float]:
    if not text:
        return [default] * n
    values = [float(x) for x in text.split()]
    if len(values) != n or not all(math.isfinite(v) for v in values):
        raise RuntimeError(f"Expected {n} numbers, got: {text}")
    return values

def _identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def _mm(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]

def _xyz_rpy(xyz, rpy):
    x, y, z = xyz
    r, p, yaw = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # URDF fixed-axis RPY: Rz(yaw) * Ry(pitch) * Rx(roll)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    out = _identity()
    for i in range(3):
        for j in range(3):
            out[i][j] = rot[i][j]
    out[0][3], out[1][3], out[2][3] = x, y, z
    return out

def _axis_angle(axis, angle):
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1.0e-12 or abs(angle) < 1.0e-15:
        return _identity()
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    out = _identity()
    out[0][0] = x * x * v + c
    out[0][1] = x * y * v - z * s
    out[0][2] = x * z * v + y * s
    out[1][0] = y * x * v + z * s
    out[1][1] = y * y * v + c
    out[1][2] = y * z * v - x * s
    out[2][0] = z * x * v - y * s
    out[2][1] = z * y * v + x * s
    out[2][2] = z * z * v + c
    return out

def _point(t, p):
    x, y, z = p
    return [
        t[0][0] * x + t[0][1] * y + t[0][2] * z + t[0][3],
        t[1][0] * x + t[1][1] * y + t[1][2] * z + t[1][3],
        t[2][0] * x + t[2][1] * y + t[2][2] * z + t[2][3],
    ]

def _pose_map(robot_yaml: dict, key: str) -> dict[str, float]:
    result: dict[str, float] = {}
    for item in robot_yaml.get(key, []) or []:
        result[str(item["name"])] = float(item.get("pos", 0.0))
    return result

def _link_transforms(
    urdf_root: ET.Element,
    base_link: str,
    joint_positions: dict[str, float],
) -> dict[str, list[list[float]]]:
    children: dict[str, list[tuple[str, str, list[float], list[float], list[float], str]]] = {}

    for joint in urdf_root.findall("joint"):
        name = str(joint.attrib["name"])
        joint_type = str(joint.attrib.get("type", "fixed"))
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = str(parent.attrib["link"])
        child_name = str(child.attrib["link"])
        origin = joint.find("origin")
        xyz = _vec(origin.attrib.get("xyz") if origin is not None else None, 3)
        rpy = _vec(origin.attrib.get("rpy") if origin is not None else None, 3)
        axis_element = joint.find("axis")
        axis = _vec(
            axis_element.attrib.get("xyz") if axis_element is not None else "1 0 0",
            3,
        )
        children.setdefault(parent_name, []).append(
            (name, child_name, xyz, rpy, axis, joint_type)
        )

    transforms = {base_link: _identity()}
    queue = [base_link]
    while queue:
        parent_name = queue.pop(0)
        parent_t = transforms[parent_name]
        for name, child_name, xyz, rpy, axis, joint_type in children.get(parent_name, []):
            t = _mm(parent_t, _xyz_rpy(xyz, rpy))
            if joint_type in ("revolute", "continuous"):
                t = _mm(t, _axis_angle(axis, float(joint_positions.get(name, 0.0))))
            elif joint_type == "prismatic":
                q = float(joint_positions.get(name, 0.0))
                t = _mm(t, _xyz_rpy([q * a for a in axis], [0, 0, 0]))
            elif joint_type != "fixed":
                raise ValueError(f"Unsupported URDF joint type: {joint_type}")
            if child_name in transforms:
                raise ValueError(f"URDF is not a tree at {child_name}")
            transforms[child_name] = t
            queue.append(child_name)
    return transforms

def _origin(element):
    origin = element.find("origin")
    return _xyz_rpy(
        _vec(origin.get("xyz") if origin is not None else None, 3),
        _vec(origin.get("rpy") if origin is not None else None, 3))

def _mesh_path(filename: str, urdf_dir: Path, robots_share: Path) -> Path:
    if filename.startswith("package://"):
        package, relative = filename[len("package://"):].split("/", 1)
        if package == "noetix_robots":
            path = robots_share / relative
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory(package)) / relative
    else:
        path = Path(filename.removeprefix("file://")).expanduser()
        if not path.is_absolute():
            path = urdf_dir / path
    if not path.is_file():
        raise ValueError(f"URDF mesh not found: {path}")
    return path.resolve()

def _mesh_bottom(path: Path, transform, scale) -> float:
    row = [transform[2][i] * scale[i] for i in range(3)]

    def z(vertex):
        return sum(row[i] * vertex[i] for i in range(3)) + transform[2][3]

    data = path.read_bytes()
    if path.suffix.lower() == ".stl":
        count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
        if len(data) == 84 + count * 50:
            return min(z(tri[start:start+3])
                       for tri in struct.iter_unpack("<12fH", data[84:])
                       for start in (3, 6, 9))
        vertices = (line.split()[1:] for line in data.decode("ascii").splitlines()
                    if line.strip().startswith("vertex "))
    elif path.suffix.lower() == ".obj":
        vertices = (line.split()[1:4] for line in data.decode().splitlines()
                    if line.startswith("v "))
    else:
        raise ValueError(f"Spawn height supports STL/OBJ collision meshes: {path}")
    return min(z([float(v) for v in vertex]) for vertex in vertices)

def collision_base_height(root, base_link, positions, urdf_dir, robots_share) -> float:
    """Place the lowest collision surface above the ground, including bent limbs."""
    transforms = _link_transforms(root, base_link, positions)
    bottoms = []
    for link in root.findall("link"):
        name = link.get("name")
        if name not in transforms:
            raise ValueError(f"URDF link disconnected from {base_link}: {name}")
        for collision in link.findall("collision"):
            t = _mm(transforms[name], _origin(collision))
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1:
                raise ValueError(f"Invalid collision geometry on {name}")
            shape = geometry[0]
            if shape.tag == "sphere":
                bottom = t[2][3] - float(shape.get("radius"))
            elif shape.tag == "box":
                size = _vec(shape.get("size"), 3)
                bottom = t[2][3] - sum(abs(t[2][i]) * size[i] / 2 for i in range(3))
            elif shape.tag == "cylinder":
                bottom = (t[2][3] - float(shape.get("radius")) * math.hypot(t[2][0], t[2][1])
                          - float(shape.get("length")) / 2 * abs(t[2][2]))
            elif shape.tag == "mesh":
                path = _mesh_path(shape.get("filename"), urdf_dir, robots_share)
                bottom = _mesh_bottom(path, t, _vec(shape.get("scale"), 3, 1.0))
            else:
                raise ValueError(f"Unsupported collision shape: {shape.tag}")
            if not math.isfinite(bottom):
                raise ValueError(f"Nonfinite collision height on {name}")
            bottoms.append(bottom)
    if not bottoms:
        raise ValueError("No collision geometry available for spawn height")
    return -min(bottoms)

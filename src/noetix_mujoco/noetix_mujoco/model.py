"""Generate MuJoCo MJCF directly from the real noetix_robots URDF assets.

No permanent model copy, Gazebo dependency, or separate joint mapping is required.
MuJoCo meshes are convexified for contact; exact Gazebo contact parity is not implied.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
from pathlib import Path
from dataclasses import dataclass

import yaml

from .geometry import _mesh_path, _vec, collision_base_height


def _fmt(values):
    return ' '.join(f'{float(x):.12g}' for x in values)


def _quat(rpy):
    """URDF Rz(yaw) Ry(pitch) Rx(roll) to MuJoCo wxyz."""
    roll, pitch, yaw = [v * 0.5 for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]


def _full_inertia_in_body_frame(values, rpy):
    """Convert a URDF inertial-frame tensor into the MJCF body frame.

    URDF's <inertial><origin rpy=...> rotates the inertia frame into the
    parent link. MJCF ``fullinertia`` already defines a full tensor in
    body coordinates, so an inertial ``quat`` must not accompany it.
    """
    ixx, iyy, izz, ixy, ixz, iyz = values
    inertia = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ], dtype=np.float64)
    w, x, y, z = _quat(rpy)
    rotation = np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),     1 - 2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z+w*x),     1 - 2*(x*x+y*y)],
    ], dtype=np.float64)
    body_inertia = rotation @ inertia @ rotation.T
    return [body_inertia[0, 0], body_inertia[1, 1], body_inertia[2, 2],
            body_inertia[0, 1], body_inertia[0, 2], body_inertia[1, 2]]


def _origin_attrs(element):
    origin = element.find('origin')
    xyz = _vec(origin.get('xyz') if origin is not None else None, 3)
    rpy = _vec(origin.get('rpy') if origin is not None else None, 3)
    return {'pos': _fmt(xyz), 'quat': _fmt(_quat(rpy))}


def _merged_config(config_dir: Path, robot_name: str, override: str) -> dict:
    result = {}
    for path in [config_dir / 'default.yaml', config_dir / f'{robot_name}.yaml'] + (
            [Path(override).expanduser().resolve()] if override else []):
        if not path.exists():
            if path.name == f'{robot_name}.yaml':
                continue
            raise FileNotFoundError(f'MuJoCo simulation config not found: {path}')
        cfg = (yaml.safe_load(path.read_text()) or {}).get('simulation', {})
        for key, value in cfg.items():
            if isinstance(value, dict) and isinstance(result.get(key), dict):
                result[key] = {**result[key], **value}
            else:
                result[key] = value
    for key in ('physics_hz','control_frequency','state_publish_hz','clock_publish_hz','gui_fps'):
        v = float(result[key])
        if not math.isfinite(v) or v <= 0:
            raise ValueError(f'{key} must be positive and finite')
        result[key] = v
    for key in ('control_frequency','state_publish_hz','clock_publish_hz'):
        ratio = result['physics_hz'] / result[key]
        if ratio < 1 or abs(ratio - round(ratio)) > 1e-8:
            raise ValueError(f'physics_hz must be an integer multiple of {key}')
    for key in ('spawn_clearance_m', 'hoist_lift_height_m', 'command_timeout_s',
                'joint_viscous_damping','joint_coulomb_friction'):
        v = float(result[key])
        if not math.isfinite(v) or v < 0:
            raise ValueError(f'{key} must be finite and nonnegative')
        result[key] = v
    # Solver and rendering controls are declared in default.yaml. Every preset
    # uses integral physical/update ratios to avoid drifting schedules.
    solver = result.get('solver_iterations', 40)
    if isinstance(solver, bool) or not isinstance(solver, int) or not 1 <= solver <= 200:
        raise ValueError('solver_iterations must be an integer in [1, 200]')
    rendering = result.get('render', {})
    if not isinstance(rendering, dict):
        raise ValueError('render must be a mapping')
    for key in ('shadows', 'reflections'):
        if not isinstance(rendering.get(key), bool):
            raise ValueError(f'render.{key} must be a boolean')
    limits = {
        'offscreen_samples': (0, 16),
        'shadow_map_size': (64, 8192),
        'checker_texture_size': (8, 2048),
        'primitive_slices': (4, 128),
        'primitive_stacks': (2, 128),
    }
    for key, (minimum, maximum) in limits.items():
        value = rendering.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
            raise ValueError(f'render.{key} must be an integer in [{minimum}, {maximum}]')
    if result['actuator_mode'] not in ('explicit_pd','predictive_pd'):
        raise ValueError('actuator_mode must be explicit_pd or predictive_pd')
    if result['initial_pose'] not in ('zero','stand','enable'):
        raise ValueError('initial_pose must be zero, stand or enable')
    return result


@dataclass(frozen=True)
class JointSpec:
    name: str
    actuator: str
    direction: float
    offset: float
    effort: float
    initial_q: float


@dataclass
class PreparedModel:
    mjcf: str
    urdf: str
    joints: list[JointSpec]
    robot_name: str
    imu_link: str
    base_link: str
    spawn_z: float
    config: dict


def build_model(robot_root: Path, config_dir: Path, config_override: str = '') -> PreparedModel:
    robot_root = Path(robot_root).resolve()
    cfg = _merged_config(Path(config_dir), robot_root.name, config_override)
    data = yaml.safe_load((robot_root / 'config/robot.yaml').read_text())
    robot = data['robot']
    order = list(robot['joint_order'])
    if robot['name'] != robot_root.name or int(robot['dof']) != len(order) or len(set(order)) != len(order):
        raise ValueError('Invalid robot name/dof/joint_order')
    mapping = {x['name']:x for x in data['joints']}
    if len(mapping) != len(data['joints']) or set(mapping) != set(order):
        raise ValueError('Robot joint mapping differs from joint_order')
    if len({mapping[n]['actuator'] for n in order}) != len(order):
        raise ValueError('Duplicated actuator name')
    path = robot_root / robot['description']['real']
    if not path.is_file():
        raise FileNotFoundError(f'URDF not found: {path}')
    urdf_root = ET.parse(path).getroot()
    links = {l.get('name'): l for l in urdf_root.findall('link')}
    joints = {j.get('name'): j for j in urdf_root.findall('joint')}
    base = robot['frames']['base']
    imu = robot['frames']['imu']
    for foot in cfg.get('foot_links', []):
        if foot not in links or not links[foot].findall('collision'):
            raise ValueError(f'Configured foot collision link is missing: {foot}')
    if base not in links or imu not in links:
        raise ValueError('Missing base or IMU frame')

    initial = {}
    if cfg['initial_pose'] != 'zero':
        key = f"{cfg['initial_pose']}_joint_angles"
        entries = data.get(key, []) or []
        if len(entries) != len(order) or {e['name'] for e in entries} != set(order):
            raise ValueError(f'{key} must specify each joint')
        initial = {str(e['name']): float(e['pos']) for e in entries}
    controlled = []
    for name in order:
        j = joints.get(name)
        if j is None or j.get('type') not in ('revolute','continuous'):
            raise ValueError(f'Unsupported or missing controlled joint: {name}')
        limit = j.find('limit')
        effort = float(limit.get('effort')) if limit is not None else 0.0
        q = initial.get(name,0.0)
        if not math.isfinite(effort) or effort <= 0 or not math.isfinite(q):
            raise ValueError(f'Invalid effort or initial position: {name}')
        if j.get('type') != 'continuous' and not float(limit.get('lower')) <= q <= float(limit.get('upper')):
            raise ValueError(f'Initial pose out of joint limits: {name}')
        item = mapping[name]
        d, offset = float(item['direction']), float(item.get('offset',0.0))
        if d not in (-1.0,1.0) or not math.isfinite(offset):
            raise ValueError(f'Invalid actuator direction or offset: {name}')
        controlled.append(JointSpec(name,str(item['actuator']),d,offset,effort,q))

    spawn_z = collision_base_height(urdf_root, base, initial, path.parent, robot_root.parent)
    spawn_z += cfg['spawn_clearance_m']
    lift_z = collision_base_height(urdf_root, base, {}, path.parent, robot_root.parent)
    lift_z += cfg['spawn_clearance_m'] + cfg['hoist_lift_height_m']
    cfg['hoist_base_z'] = lift_z

    mjcf = ET.Element('mujoco', model=f'noetix_{robot_root.name}')
    ET.SubElement(mjcf, 'compiler', angle='radian', fusestatic='false',
                  autolimits='true', inertiafromgeom='false')
    ET.SubElement(mjcf, 'option', timestep=_fmt([1.0/cfg['physics_hz']]),
                  gravity='0 0 -9.81', integrator='implicitfast', solver='Newton',
                  iterations=str(cfg['solver_iterations']))
    visual = ET.SubElement(mjcf, 'visual')
    ET.SubElement(visual, 'quality',
                  shadowsize=str(cfg['render']['shadow_map_size']),
                  offsamples=str(cfg['render']['offscreen_samples']),
                  numslices=str(cfg['render']['primitive_slices']),
                  numstacks=str(cfg['render']['primitive_stacks']))
    defaults = ET.SubElement(mjcf, 'default')
    ET.SubElement(defaults, 'joint', armature='0')
    ET.SubElement(defaults, 'geom', condim='3', friction=_fmt([
        cfg['foot_contact']['mu1'], 0.005, 0.0001]))
    assets = ET.SubElement(mjcf,'asset')
    # Blue/white procedural checkerboard: purely visual; ground friction and
    # collision properties are exactly the same as before.
    ET.SubElement(assets, 'texture', name='noetix_ground_checker', type='2d',
                  builtin='checker', rgb1='0.18 0.35 0.61',
                  rgb2='0.91 0.95 1.0',
                  width=str(cfg['render']['checker_texture_size']),
                  height=str(cfg['render']['checker_texture_size']))
    ET.SubElement(assets, 'material', name='noetix_ground_material',
                  texture='noetix_ground_checker', texrepeat='2 2',
                  texuniform='true',
                  reflectance='0.15' if cfg['render']['reflections'] else '0')
    world = ET.SubElement(mjcf,'worldbody')
    ET.SubElement(world,'light', pos='1 -2 5', dir='-0.2 0.1 -1',
                  diffuse='0.8 0.8 0.8', castshadow='true' if cfg['render']['shadows'] else 'false')
    ET.SubElement(world,'geom', name='ground', type='plane', pos='0 0 0', size='0 0 0.1',
                  material='noetix_ground_material',
                  friction=_fmt([cfg['foot_contact']['mu1'],0.005,0.0001]))
    ET.SubElement(world,'body', name='noetix_hoist_anchor', mocap='true',
                  pos=_fmt([0,0, cfg['hoist_base_z']]))
    actuator = ET.SubElement(mjcf,'actuator')
    eq = ET.SubElement(mjcf,'equality')
    ET.SubElement(eq,'weld',name='noetix_hoist_weld', body1=base,
                  body2='noetix_hoist_anchor', active='false', solref='0.002 1')
    sensor = ET.SubElement(mjcf,'sensor')
    children = {}
    for j in joints.values():
        parent, child = j.find('parent').get('link'), j.find('child').get('link')
        children.setdefault(parent,[]).append((j,child))
    visited = set()
    def visit(name, parent, attrs):
        if name in visited:
            raise ValueError(f'Cycle or repeated child link: {name}')
        visited.add(name)
        body = ET.SubElement(parent, 'body', name=name, **attrs)
        if name == base:
            ET.SubElement(body,'freejoint',name='noetix_floating_base')
        link = links[name]
        inertia = link.find('inertial')
        if inertia is None:
            raise ValueError(f'Link missing inertial: {name}')
        mass = float(inertia.find('mass').get('value'))
        i = inertia.find('inertia')
        mat = [float(i.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
        if not math.isfinite(mass) or mass <= 0 or not all(math.isfinite(x) for x in mat):
            raise ValueError(f'Invalid mass/inertia: {name}')
        # MuJoCo does not allow ``fullinertia`` together with inertial
        # orientation. Preserve the URDF COM position and rotate its full
        # inertia tensor into the link frame before writing MJCF.
        origin = inertia.find('origin')
        inertial_xyz = _vec(origin.get('xyz') if origin is not None else None, 3)
        inertial_rpy = _vec(origin.get('rpy') if origin is not None else None, 3)
        body_inertia = _full_inertia_in_body_frame(mat, inertial_rpy)
        ET.SubElement(body, 'inertial',
                      pos=_fmt(inertial_xyz), mass=_fmt([mass]),
                      fullinertia=_fmt(body_inertia))
        for cat in ('collision','visual'):
            for index, element in enumerate(link.findall(cat)):
                geometry = element.find('geometry')
                if geometry is None or len(geometry)!=1:
                    raise ValueError(f'Invalid {cat} geometry on {name}')
                shape = geometry[0]
                geom_attrs = {'name':f'{name}_{cat}_{index}', **_origin_attrs(element)}
                if cat == 'visual':
                    geom_attrs.update(contype='0',conaffinity='0',group='1')
                    color = element.find('material/color')
                    geom_attrs['rgba'] = color.get('rgba') if color is not None else '0.7 0.75 0.8 1'
                else:
                    geom_attrs.update(contype='1',conaffinity='1',group='0',rgba='0.4 0.5 0.6 0.18')
                if shape.tag == 'mesh':
                    mesh_name = f'{name}_{cat}_{index}_mesh'
                    mesh_path = _mesh_path(shape.get('filename'), path.parent, robot_root.parent)
                    mesh_attrs = {'name':mesh_name,'file':str(mesh_path)}
                    if shape.get('scale'):
                        mesh_attrs['scale'] = _fmt(_vec(shape.get('scale'),3,1.0))
                    ET.SubElement(assets,'mesh',**mesh_attrs)
                    geom_attrs.update(type='mesh',mesh=mesh_name)
                elif shape.tag == 'box':
                    geom_attrs.update(type='box',size=_fmt([x/2 for x in _vec(shape.get('size'),3)]))
                elif shape.tag == 'sphere':
                    geom_attrs.update(type='sphere',size=shape.get('radius'))
                elif shape.tag == 'cylinder':
                    geom_attrs.update(type='cylinder',size=_fmt([float(shape.get('radius')),
                                                                float(shape.get('length'))/2]))
                else:
                    raise ValueError(f'Unsupported URDF geometry: {shape.tag}')
                ET.SubElement(body,'geom',**geom_attrs)
        if name == imu:
            ET.SubElement(body,'site',name='noetix_imu_site',size='0.001',rgba='0 0 0 0')
        for j, child in children.get(name,[]):
            body_child = visit(child,body,_origin_attrs(j))
            if j.get('type') == 'fixed':
                continue
            axis = j.find('axis')
            axis_vector = axis.get('xyz') if axis is not None else '1 0 0'
            joint_attrs = {'name':j.get('name'),'type':'hinge','pos':'0 0 0','axis':axis_vector}
            limit = j.find('limit')
            if j.get('type') != 'continuous':
                joint_attrs.update(limited='true',range=_fmt([float(limit.get('lower')),
                                                              float(limit.get('upper'))]))
            dynamics = j.find('dynamics')
            if dynamics is not None:
                joint_attrs['damping'] = str(max(0.,float(dynamics.get('damping','0'))))
                joint_attrs['frictionloss'] = str(max(0.,float(dynamics.get('friction','0'))))
            ET.SubElement(body_child,'joint',**joint_attrs)
        return body
    visit(base,world,{'pos':_fmt([0,0,spawn_z])})
    if visited != set(links):
        raise ValueError(f'URDF disconnected links: {set(links)-visited}')
    for item in controlled:
        ET.SubElement(actuator,'motor',name=f'{item.name}_motor',joint=item.name,
                      gear='1',ctrllimited='true',ctrlrange=_fmt([-item.effort,item.effort]))
    ET.SubElement(sensor,'framequat',name='noetix_imu_quat',objtype='site',objname='noetix_imu_site')
    ET.SubElement(sensor,'gyro',name='noetix_imu_gyro',site='noetix_imu_site')
    ET.SubElement(sensor,'accelerometer',name='noetix_imu_accel',site='noetix_imu_site')
    return PreparedModel(ET.tostring(mjcf,encoding='unicode'),str(path), controlled,
                         robot_root.name, imu,base,spawn_z,cfg)

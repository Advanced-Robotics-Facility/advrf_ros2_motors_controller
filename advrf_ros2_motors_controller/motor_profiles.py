"""Resolve middleware motor profiles into parameters for the selected ROS joints."""
import math
from pathlib import Path
import yaml
from .launch_config import robot_reference
from .panel_config import load_parameters

MODES = {'position': 0x3B, 'impedance': 0xD4, 'velocity': 0x71,
         'torque': 0xCC, 'current': 0xDD}


def profile_parameters(joint_limits, namespace, mode, directory='', profile='', ec_cfg=''):
    """Exact-name/explicit alias match; uniform profiles also support renamed joints.

    Return None if no profile source was configured. An explicit/discovered source
    with missing/ambiguous mode or incomplete joints is an error, never a silent fallback.
    """
    if not profile and not directory and ec_cfg:
        control = robot_reference(Path(ec_cfg).expanduser().resolve(strict=True), 'robot_control_path')
        directory = str(control.parent / 'motor')
    if not profile and not directory:
        return None
    paths = [Path(profile).expanduser()] if profile else sorted(Path(directory).expanduser().glob('*.yaml'))
    if not paths:
        raise ValueError(f'No motor profiles found in {directory}')
    candidates = []
    for path in paths:
        with path.open() as stream:
            doc = yaml.safe_load(stream)
        if not isinstance(doc, dict):
            raise ValueError(f'Expected motor profile mapping: {path}')
        motors = {name: entry for name, entry in doc.items()
                  if isinstance(entry, dict) and 'control_mode' in entry}
        if not motors or any(entry['control_mode'] != MODES[mode] for entry in motors.values()):
            continue
        gains = {}
        for name, entry in motors.items():
            values = entry.get('gains')
            if (not isinstance(values, list) or len(values) != 5
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) or abs(v) > 3.402823466e38 for v in values)):
                raise ValueError(f'{path}: expected five finite float32 gains for {name}')
            gains[name] = [float(v) for v in values]
        candidates.append((path, gains))
    if len(candidates) != 1:
        raise ValueError(f'Expected one {mode} motor profile, found {len(candidates)}; pass motor_profile explicitly')
    path, gains = candidates[0]
    params = load_parameters(joint_limits, namespace)
    joints = params.get('joints', [])
    aliases = params.get('profile_joints', joints)
    if not joints or len(aliases) != len(joints) or len(set(aliases)) != len(aliases):
        raise ValueError('profile_joints must contain one unique profile name per configured joint')
    uniform = len({tuple(values) for values in gains.values()}) == 1
    selected = []
    for joint, alias in zip(joints, aliases):
        if alias in gains:
            selected.extend(gains[alias])
        elif uniform and 'profile_joints' not in params:
            selected.extend(next(iter(gains.values())))
        else:
            raise ValueError(f'{path}: no profile entry for {joint} ({alias}); configure profile_joints')
    return {'path': str(path.resolve()), 'joint_gains': selected, 'start_joint_gains': selected}

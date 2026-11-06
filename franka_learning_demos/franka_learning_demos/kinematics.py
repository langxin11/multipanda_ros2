"""Forward kinematics from the loaded URDF, without a hard-coded Panda model."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    length = np.linalg.norm(axis)
    if length < 1e-12 or not np.isfinite(length):
        raise ValueError('Invalid URDF joint axis')
    x, y, z = axis / length
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


class SerialChain:
    """URDF fixed/revolute/prismatic chain; cross-checked against MoveIt before use."""

    def __init__(self, description, base, tip):
        root = ET.fromstring(description)
        parents = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        chain, seen = [], set()
        while tip != base:
            if tip in seen or tip not in parents:
                raise ValueError(f'No valid URDF chain from {base} to {tip}')
            seen.add(tip)
            joint = parents[tip]
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError(f'Unsupported joint type: {kind}')
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            transform = np.eye(4)
            transform[:3, :3] = (rotation([0, 0, 1], rpy[2]) @
                                 rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0]))
            transform[:3, 3] = xyz
            axis = joint.find('axis')
            axis = np.array([float(v) for v in axis.get('xyz', '1 0 0').split()]) if axis is not None else np.array([1., 0., 0.])
            chain.append((joint.get('name'), kind, transform, axis))
            tip = joint.find('parent').get('link')
        self.chain = list(reversed(chain))
        self.joint_names = [name for name, kind, _, _ in self.chain if kind != 'fixed']

    def fk(self, positions):
        transform = np.eye(4)
        for name, kind, origin, axis in self.chain:
            transform = transform @ origin
            motion = np.eye(4)
            if kind != 'fixed':
                value = float(positions[name])
                if not math.isfinite(value):
                    raise ValueError(f'Non-finite joint position: {name}')
                if kind == 'prismatic':
                    motion[:3, 3] = axis * value
                else:
                    motion[:3, :3] = rotation(axis, value)
            transform = transform @ motion
        return transform


def quaternion_matrix(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = values / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

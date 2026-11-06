"""Independent analytic examples for the URDF transform conventions."""
import math

import numpy as np
import pytest

from franka_learning_demos.kinematics import SerialChain


MODEL = '''<robot name="test">
<joint name="mount" type="fixed">
  <parent link="base"/><child link="shoulder"/>
  <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
</joint>
<joint name="hinge" type="revolute">
  <parent link="shoulder"/><child link="arm"/><axis xyz="0 0 1"/>
</joint>
<joint name="tool" type="fixed">
  <parent link="arm"/><child link="tip"/><origin xyz="2 0 0"/>
</joint>
</robot>'''


def test_origin_rotation_precedes_joint_rotation():
    chain = SerialChain(MODEL, 'base', 'tip')
    np.testing.assert_allclose(chain.fk({'hinge': 0})[:3, 3], [1, 4, 3], atol=1e-12)
    np.testing.assert_allclose(chain.fk({'hinge': math.pi/2})[:3, 3], [-1, 2, 3], atol=1e-12)


def test_subchain_and_missing_state():
    chain = SerialChain(MODEL, 'shoulder', 'tip')
    np.testing.assert_allclose(chain.fk({'hinge': 0})[:3, 3], [2, 0, 0])
    with pytest.raises(KeyError):
        chain.fk({})
    with pytest.raises(ValueError):
        chain.fk({'hinge': float('nan')})


def test_disconnected_and_unsupported_chains_fail():
    with pytest.raises(ValueError):
        SerialChain(MODEL, 'base', 'unknown')
    with pytest.raises(ValueError):
        SerialChain(MODEL.replace('revolute', 'floating'), 'base', 'tip')


def test_prismatic_axis_is_in_joint_frame():
    chain = SerialChain(MODEL.replace('revolute', 'prismatic').replace('axis xyz="0 0 1"', 'axis xyz="1 0 0"'), 'base', 'tip')
    np.testing.assert_allclose(chain.fk({'hinge': 0.5})[:3, 3], [1, 4.5, 3], atol=1e-12)

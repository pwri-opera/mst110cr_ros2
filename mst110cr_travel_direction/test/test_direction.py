import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from mst110cr_travel_direction.adapter import TravelDirectionAdapter
from mst110cr_travel_direction.policy import DirectionPolicy


@pytest.mark.parametrize('degrees, expected', [
    (0, 'forward'), (89, 'forward'), (-89, 'forward'),
    (91, 'reverse'), (-91, 'reverse'), (180, 'reverse'),
    (-180, 'reverse'), (360, 'forward'), (450, 'boundary_stop'),
    (90, 'boundary_stop'), (-90, 'boundary_stop'),
])
def test_direction_and_wrapping(degrees, expected):
    policy = DirectionPolicy(enabled=True)
    policy.update(math.radians(degrees), 10, 100)
    assert policy.state(10.1, 100.1) == expected


def test_freshness_and_invalid_samples():
    policy = DirectionPolicy(enabled=True)
    assert policy.state(10, 100) == 'waiting_for_angle'
    policy.update(0, 10, 100)
    assert policy.state(10.6, 100.1) == 'stale_angle'
    assert policy.state(10.1, 100.6) == 'stale_angle'
    assert policy.state(10.1, 99) == 'stale_angle'
    policy.update(float('nan'), 10, 100)
    assert policy.state(10, 100) == 'waiting_for_angle'


def test_disabled_does_not_require_angle():
    assert DirectionPolicy().state(10, 100) == 'passthrough'


def test_stop_band():
    policy = DirectionPolicy(enabled=True, stop_band_deg=5)
    for degrees in (86, 90, 94, -86, -94):
        policy.update(math.radians(degrees), 10, 100)
        assert policy.state(10, 100) == 'boundary_stop'


@pytest.mark.parametrize('kwargs', [
    {'timeout': 0}, {'timeout': float('nan')}, {'threshold_deg': 180},
    {'threshold_deg': 0}, {'stop_band_deg': -1}, {'stop_band_deg': 90},
])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        DirectionPolicy(**kwargs)


@pytest.mark.parametrize('state, invert, expected', [
    ('passthrough', False, (1.0, 0.2)),
    ('forward', False, (1.0, 0.2)),
    ('reverse', False, (-1.0, 0.2)),
    ('reverse', True, (-1.0, -0.2)),
    ('waiting_for_angle', False, (0.0, 0.0)),
    ('stale_angle', False, (0.0, 0.0)),
    ('boundary_stop', False, (0.0, 0.0)),
])
def test_command_output_and_input_not_modified(state, invert, expected):
    published = []
    adapter = SimpleNamespace(
        current_state=lambda: state, invert_angular_z=invert,
        output=SimpleNamespace(publish=published.append))
    message = Twist()
    message.linear.x = 1.0
    message.angular.z = 0.2
    TravelDirectionAdapter.on_command(adapter, message)
    assert len(published) == 1
    assert (published[0].linear.x, published[0].angular.z) == expected
    assert message.linear.x == 1.0
    assert message.angular.z == 0.2


def test_joint_name_lookup_and_missing_position():
    adapter = SimpleNamespace(joint_name='rotate_joint', policy=DirectionPolicy(enabled=True))
    message = JointState()
    message.name = ['vessel_pin_joint', 'rotate_joint']
    message.position = [0.1, 3.0]
    message.header.stamp.sec = 100
    TravelDirectionAdapter.on_joints(adapter, message)
    assert adapter.policy.angle == 3.0
    received = adapter.policy.received
    message.name = ['another_joint']
    TravelDirectionAdapter.on_joints(adapter, message)
    assert adapter.policy.received == received
    message.name = ['rotate_joint']
    message.position = []
    TravelDirectionAdapter.on_joints(adapter, message)
    assert adapter.policy.angle is None


def test_nonfinite_command_stops_when_enabled():
    published = []
    adapter = SimpleNamespace(
        current_state=lambda: 'forward', invert_angular_z=False,
        output=SimpleNamespace(publish=published.append))
    message = Twist()
    message.linear.x = float('nan')
    TravelDirectionAdapter.on_command(adapter, message)
    assert published == [Twist()]

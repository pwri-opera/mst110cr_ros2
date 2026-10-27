import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from mst110cr_navigation.gnss_ekf_initializer import (
    initial_pose, valid_pose, lower_heading_pose, GnssEkfInitializer)


def sample():
    m=PoseWithCovarianceStamped();m.header.frame_id='world';m.header.stamp.sec=10
    m.pose.pose.position.x=2.;m.pose.pose.position.y=3.;m.pose.pose.orientation.w=1.
    m.pose.covariance=np.diag([4.,9.,16.,.1,.2,.3]).ravel().tolist()
    return m


def test_rotated_map_pose_and_covariance():
    m=sample();t=TransformStamped()
    t.transform.translation.x=10.;t.transform.translation.y=-5.
    t.transform.rotation.z=math.sin(math.pi/4);t.transform.rotation.w=math.cos(math.pi/4)
    out=initial_pose(m,t,'map')
    assert out.header.frame_id=='map'
    assert out.pose.pose.position.x==pytest.approx(7.)
    assert out.pose.pose.position.y==pytest.approx(-3.)
    assert out.pose.pose.orientation.z==pytest.approx(math.sin(math.pi/4))
    assert out.pose.covariance[0]==pytest.approx(9.)
    assert out.pose.covariance[7]==pytest.approx(4.)
    assert out.pose.covariance[35]==pytest.approx(.3)
    assert m.header.frame_id=='world'
    assert m.pose.pose.position.x==2.


def test_invalid_initial_pose_is_rejected():
    m=sample();assert valid_pose(m)
    m.pose.pose.orientation.w=0.;assert not valid_pose(m)
    m=sample();m.pose.pose.position.x=float('nan');assert not valid_pose(m)
    m=sample();m.pose.covariance[35]=0.;assert not valid_pose(m)


def heading_sample():
    m = Imu()
    m.header.frame_id = 'world'
    m.header.stamp.sec = 10
    m.orientation.z = 2 * math.sin(0.6 / 2)
    m.orientation.w = 2 * math.cos(0.6 / 2)
    m.orientation_covariance[0] = 1e6
    m.orientation_covariance[4] = 1e6
    m.orientation_covariance[8] = 0.01
    return m


def test_lower_heading_preserves_world_yaw_and_variance():
    m = heading_sample()
    pose = lower_heading_pose(m, 10_100_000_000)
    assert pose.header == m.header
    assert pose.pose.pose.orientation.z == pytest.approx(math.sin(0.3))
    assert pose.pose.pose.orientation.w == pytest.approx(math.cos(0.3))
    assert pose.pose.covariance[35] == pytest.approx(0.01)


@pytest.mark.parametrize('case', ['unavailable', 'zero_variance', 'nan_variance',
                                  'zero_quaternion', 'nan_quaternion', 'sensor_frame',
                                  'stale', 'future'])
def test_invalid_lower_heading_is_rejected(case):
    m = heading_sample()
    now = 10_100_000_000
    if case == 'unavailable': m.orientation_covariance[0] = -1.
    if case == 'zero_variance': m.orientation_covariance[8] = 0.
    if case == 'nan_variance': m.orientation_covariance[8] = float('nan')
    if case == 'zero_quaternion': m.orientation.z = m.orientation.w = 0.
    if case == 'nan_quaternion': m.orientation.z = float('nan')
    if case == 'sensor_frame': m.header.frame_id = 'imu_link'
    if case == 'stale': now = 10_600_000_000
    if case == 'future': now = 9_800_000_000
    assert lower_heading_pose(m, now) is None


def test_heading_waits_for_initialization_and_rejects_duplicates():
    from types import SimpleNamespace
    published = []
    state = SimpleNamespace(
        ready=False, sent=False, initial_stamp=9_000_000_000, last_heading_stamp=0,
        heading_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)))
    m = heading_sample()
    GnssEkfInitializer.heading_callback(state, m)
    assert not published
    assert state.pending_heading == m
    state.ready = True
    m.header.stamp.nanosec = 100_000_000
    GnssEkfInitializer.heading_callback(state, m)
    assert len(published) == 1
    GnssEkfInitializer.heading_callback(state, m)
    assert len(published) == 1


def initializer_state():
    from types import SimpleNamespace
    published = []
    transform = TransformStamped()
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    state = SimpleNamespace(
        sent=False, ready=False, pending=sample(), pending_heading=heading_sample(),
        target='map', initial_stamp=0,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
        initial_pub=SimpleNamespace(
            get_subscription_count=lambda: 1, publish=published.append),
        buffer=SimpleNamespace(lookup_transform=lambda *args: transform))
    return state, published


def test_initial_yaw_uses_lower_heading_in_map_and_its_variance():
    state, published = initializer_state()
    state.pending_heading.header.stamp.nanosec = 50_000_000
    GnssEkfInitializer.initialize(state)
    assert state.sent
    assert len(published) == 1
    pose = published[0]
    # GNSS yaw is zero; lower-body yaw is 0.6, then map rotation adds pi/2.
    assert pose.pose.pose.orientation.z == pytest.approx(math.sin((0.6 + math.pi/2)/2))
    assert pose.pose.pose.orientation.w == pytest.approx(math.cos((0.6 + math.pi/2)/2))
    assert pose.pose.pose.position.x == pytest.approx(-3.)
    assert pose.pose.pose.position.y == pytest.approx(2.)
    assert pose.pose.covariance[35] == pytest.approx(0.01)
    assert state.initial_stamp == 10_050_000_000
    GnssEkfInitializer.initialize(state)
    assert len(published) == 1


@pytest.mark.parametrize('case', ['no_heading', 'no_gnss', 'invalid_heading',
                                  'stale_heading', 'stale_gnss'])
def test_initialization_waits_for_valid_fresh_position_and_heading(case):
    state, published = initializer_state()
    if case == 'no_heading': state.pending_heading = None
    if case == 'no_gnss': state.pending = None
    if case == 'invalid_heading': state.pending_heading.orientation_covariance[0] = -1.
    if case == 'stale_heading': state.pending_heading.header.stamp.sec = 9
    if case == 'stale_gnss': state.pending.header.stamp.sec = 9
    GnssEkfInitializer.initialize(state)
    assert not published
    assert not state.sent


def test_pending_gnss_is_updated_while_waiting_for_heading():
    state, _ = initializer_state()
    newer = sample()
    newer.header.stamp.nanosec = 100_000_000
    newer.pose.pose.position.x = 5.
    GnssEkfInitializer.pose_callback(state, newer)
    assert state.pending == newer

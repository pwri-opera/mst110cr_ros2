import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from mst110cr_navigation.gnss_ekf_initializer import initial_pose, valid_pose


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

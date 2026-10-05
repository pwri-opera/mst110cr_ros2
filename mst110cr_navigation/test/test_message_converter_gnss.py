"""GNSS conversion preserves coordinate semantics and rejects unusable covariance."""
import math

import pytest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from mst110cr_navigation.message_converter_gnss import convert_pose, covariance_diagonal


def test_conversion_preserves_world_pose_and_measurement_time():
    source = PoseStamped()
    source.header.frame_id = 'world'
    source.header.stamp.sec = 1790927982
    source.header.stamp.nanosec = 201436975
    source.pose.position.x = 21482.3
    source.pose.position.y = 14077.3
    source.pose.position.z = 28.5
    source.pose.orientation.z = math.sin(-1.3)
    source.pose.orientation.w = math.cos(-1.3)
    covariance = covariance_diagonal([25.0, 25.0, 100.0], [math.pi**2]*3)

    output = convert_pose(source, covariance)

    assert isinstance(output, PoseWithCovarianceStamped)
    assert output.header == source.header
    assert output.pose.pose == source.pose
    assert list(output.pose.covariance)[::7] == [25.0, 25.0, 100.0] + [math.pi**2]*3
    assert all(value == 0.0 for i, value in enumerate(output.pose.covariance) if i % 7)
    # The output owns its data; a downstream change must not relabel the source.
    output.header.frame_id = 'map'
    output.pose.pose.position.x = 0.0
    assert source.header.frame_id == 'world'
    assert source.pose.position.x == 21482.3


@pytest.mark.parametrize('invalid', [[1.0, 2.0], [1.0, 2.0, 3.0, 4.0],
                                    [0.0, 1.0, 1.0], [-1.0, 1.0, 1.0],
                                    [float('nan'), 1.0, 1.0], [float('inf'), 1.0, 1.0]])
def test_invalid_position_or_orientation_uncertainty_is_rejected(invalid):
    with pytest.raises(ValueError):
        covariance_diagonal(invalid, [1.0]*3)
    with pytest.raises(ValueError):
        covariance_diagonal([1.0]*3, invalid)

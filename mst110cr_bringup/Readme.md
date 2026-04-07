# mst110cr_bringup

## launch

### mst110cr_remote.launch.py

* rviz2を起動する

### mst110cr_vehicle.launch.py

以下の4つのlaunchを起動する
* mst110cr_com3ros/mst110cr_com3_ros.launch.py
* gnss_localizer_ros2/gnss_localizer_ros2.launch.py
* mst110cr_navigation/ekf_localization.launch.py
* mst110cr_navigation/mst110cr_navigation.launch.py

* TF: world -> map を起動する
* robot_state_publisher を起動する
    * urdf/mst110cr.xacro 


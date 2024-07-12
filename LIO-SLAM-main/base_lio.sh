#roslaunch hunter_bringup hunter_robot_base.launch  & sleep 3;
roslaunch ouster_ros sensor.launch & sleep 10
roslaunch fast_lio mapping_ouster64.launch

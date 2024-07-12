# LIO-SLAM

## Update 9.15

Local point cloud filtering, see rostopic "/Local_map"
Parameters local_x, local_y, local_z in the "ouster64.yaml" are used to adjust the range.

## Update 7.1

Lidar Imu Odometry for ouster lidars (https://github.com/ouster-lidar/ouster-ros/tree/master#getting-started)

The repository is based on FASTLIO (https://github.com/hku-mars/FAST_LIO). 

It is easy and quick to start from nothing.

1. build
``
catkin_make 
``
If there's any dependency problem, check the github links above.

2. check your lidar connection and change the "sensor_hostname" in the /livox_ros_driver/livox_ros_driver/launch/sensor.launch
``
roslaunch ouster_ros sensor.launch 
``

3. run the lio by the sh "lio.sh"

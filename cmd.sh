# 1. Turn on everything, unclock robot
# 2. start robot
bash frankapy/bash_scripts/start_control_pc.sh -u katefgroup -i franka-control -d /home/katefgroup/franka-interface -a 192.168.1.1

# 3. camera
roslaunch azure_kinect_ros_driver kinect_rgbd.launch fps:=30 color_resolution:=1080P depth_unit:=32FC1
cp camera/publish.launch ~/catkin_ws/src/ar_track_alvar/ar_track_alvar/launch/; roslaunch ar_track_alvar publish.launch

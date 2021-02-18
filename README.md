# perception_pipeline
perception pipeline for multiple LIDAR streams

## Startup Procedure after restart of computer:

1.
check these services are running, if not start.
sudo systemctl restart ptp4l
sudo systemctl restart phc2sys

2.
roscore

3.
cd ~/dev/catkinws-ouster/
source devel_release/setup.bash
roslaunch ouster_ros ouster-quad.launch 

4.
cd ~/dev/perception_ws/
source devel_debug/setup.bash
roslaunch perception_pipeline transform_node.launch

5.
rviz -d ~/dev/perception_ws/src/perception_pipeline/config/rvizconfig.rviz

6.
After closing the nodes makes sure to stop the roscore and restart it again as well.

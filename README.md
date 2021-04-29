# Perception Pipeline

* perception pipeline for multiple LIDAR streams. Currently the package is ocnfigured for 4 Ouster-128 sensors
* Bounding box, OctreeGrid and VoxelGrid fiters are implemented.
* Output can either be saved to *.pcd files or streamed with ROS messages.
* Current test setup achieved 20hz resolution with 100-150ms. Highly depended on the host system. 

## Startup Procedure:

1. check these services are running, if not start.
sudo systemctl restart ptp4l
sudo systemctl restart phc2sys

2. Start roscore
roscore

3. Run ouster lidar client for ROS 
cd ~/dev/catkinws-ouster/
source devel_release/setup.bash
roslaunch ouster_ros ouster-quad.launch 

4. Run perception pipeline
cd ~/dev/perceptionws/
source devel_release/setup.bash
roslaunch perception_pipeline perception_pipeline.launch

5. Run Rviz to visualize
rviz -d ~/dev/perception_ws/src/perception_pipeline/config/rvizconfig.rviz

6. After closing the nodes makes sure to stop the roscore and restart it again as well.

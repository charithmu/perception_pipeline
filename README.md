# Perception Pipeline

### perception pipeline for multiple LIDAR streams. Currently the package is ocnfigured for 4 Ouster-128 sensors
### Bounding box, OctreeGrid and VoxelGrid fiters are implemented.
### Output can either be saved to *.pcd files or streamed with ROS messages.
### Current test setup achieved 20hz resolution with 100-150ms. Highly depended on the host system. 

## Startup Procedure after restart of computer:

1. check these services are running, if not start.
sudo systemctl restart ptp4l
sudo systemctl restart phc2sys

2. Start roscore
roscore

3. Run ouster lidar client for ROS 
source ~/dev/catkinws-ouster/devel_release/setup.bash
roslaunch ouster_ros ouster-quad.launch 

4. Run perception pipeline
cd ~/dev/perceptionws/
source ~/dev/perceptionws/devel_release/setup.bash
roslaunch perception_pipeline perception_pipeline.launch

5. Run Rviz to visualize
conda deactivate
rviz -d ~/dev/perception_pipeline/config/rvizconfig.rviz

6. After closing the nodes makes sure to stop the roscore and restart it again as well.

7. Run Service Nodes and wait
source ~/dev/perceptionws/devel_release/setup.bash  
roslaunch perception_pipeline semseg_node.launch 

8. Run Prdictor Models and wait
source ~/dev/perceptionws/devel_release/setup.bash 
roslaunch perception_pipeline prediction_services.launch

9. rosrun topic_tools relay /env/combined/passfilter env/semseg_input
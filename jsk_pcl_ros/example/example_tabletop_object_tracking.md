This page shows an end to end example of running tabletop tracking code (https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_pcl_ros/launch/tracking.launch) to track objects kept on the table.

Note: This example is conducted with Microsoft Kinect Camera. One big table is placed infront of Kinect Camera. Then we placed one square shaped box on the table and followed below steps to enable tracking.

Following are the steps to run this example:

1. Run command: source /opt/ros/kinetic/setup.bash

2. Run: roslaunch openni_launch openni.launch

3. Run in a seperate terminal after running step 1: roslaunch jsk_pcl_ros tracking.launch input_point:=/camera/depth_registered/points

4. Run in a seperate terminal after running step 1: rviz, subscribe /camera/depth/points or /camera/depth_registered/points
    
    4.1. Then visualize '/particle_filter_tracker/track_result_posein rviz by selecting 'Pose' in the display and further selecting ''/particle_filter_tracker/track_result_pose' in topic.
    
    4.2. Add panel named "SelectPointCloudPublishAction" (see http://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/panels/select_point_cloud_publish_action.html)
    
    4.3. Follow instruction Push the "Select" button at the top bar , drag and surround the target poincloud which you                                              want to track in the rectangle area.Then, finally, push the "SelectPointCloudPublishActoin" button at SelectPointCloudPublishAction Panel. The tracker will start tracking the target. (https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/doc/jsk_pcl_ros/nodes/particle_filter_tracking.md)

After running above steps, we moved the box from its position to new position. The output can be seen as follows:
![tabletop_success_example](https://github.com/sanketrahul/jsk_recognition/blob/master/jsk_pcl_ros/example/success_tabletop.png)

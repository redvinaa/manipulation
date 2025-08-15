# Geometric pose estimation

This excercise is about estimating the pose of a known object using a depth camera
and a basic ICP algorithm. It is based on
[MIT Robotic Manipulation - Geometric Pose Estimation](https://manipulation.csail.mit.edu/pose.html)

1. Load cup model .obj file
1. Subscribe to camera PCL
1. Run RANSAC to find and remove ground plane from pointcloud
1. Use ICP to estimate optimal transform that moves cup points to origin-centered model
(This helps convergence by decreasing partial-views effect)
1. Invert the transform and visualize cup model and TF moved to sensed position

## Usage

1. Build the package
1. Launch
```bash
ros2 launch geometric_pose_estimation gazebo_with_camera_and_icp.launch.py
```
1. Visualize the pointcloud and the marker array in RViz. You can step using the rviz_visual_tools panel.

Check out the demo:
<p align="center">
  <img src="icp_cup_demo.gif" alt="ICP cup demo" width="800">
</p>

# Basic pick and place

This package implements excercises from
[MIT Robotic Manipulation - Basic Pick and Place](https://manipulation.csail.mit.edu/pick.html)

It uses the UR5 robot in Gazebo simulation with MoveIt!.
A differential inverse kinematics optimization is used to control the robot.
As an optimization constraint a virtual wall is also added, which the end-effector should not cross.
Individual joint velocities are also constrained.

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
ros2 control load_controller --set-state active forward_velocity_controller
ros2 run basic_pick_and_place_ur differential_ik_optimization --ros-args -p use_sim_time:true
```

Check out the demo:
<p align="center">
  <img src="differential_ik_optimization.gif" alt="Differential IK control demo" width="800">
</p>

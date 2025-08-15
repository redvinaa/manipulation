# Simple pendulum

This package implements the simplest possible manipulator: a pendulum with a motor.
It simulates the effect of reflected inertia from the motor through the gearbox on the pendulum.

1. Simulates the pendulum as a control system, calculating inertias and visualizing it in RViz
1. Implements two nodes:
   - `pendulum_torque_control`: You can control motor torque directly
   - `pendulum_PID_control`: You control desired pendulum angle through PID controller

Based on excercise 2.1 of [MIT Robotic Manipulation - Let's get you a robot](https://manipulation.csail.mit.edu/robot.html#section7)

## Usage
1. Build the package
1. Run node
```bash
ros2 run pendulum pendulum_PID_control
```
1. Visualize marker array in RViz
1. Run joint_state_publisher_gui
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui /home/ubuntu/manipulation/workspace/src/pendulum/pendulum.urdf
```
1. Play around with the PID controller

Check out the demo:
<p align="center">
  <img src="pendulum_PID_control.gif" alt="Pendulum PID control" width="800">
</p>

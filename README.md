# Robotic manipulation

This repo stores my code for going through the [MIT course](https://manipulation.csail.mit.edu/index.html) on robotic manipulation.

*Disclaimer*: because this is just practice code, there isn't excessive documentation, check the source code.

## Usage

Everything is dockerized, run
```bash
docker compose build  # build the image
docker compose up -d  # start the container headlessly
docker exec -it manipulation bash  # enter the container
```

When you enter, the `setup_environment` script is sourced automatically, so all ros commands and
already built packages are available.

You start out in the `workspace` directory, where you can directly build the packages:
```bash
colcon build
```

Then you might need to source the workspace again (if you built a package for the first time):
```bash
source install/setup.bash
```

Then you can run any executable, for example:
```bash
ros2 run pendulum pendulum_PID_control
```

**Check each package's `README.md` for more details on how to run the code.**

### Notes

To get a working UR stack in simulation, it's enough to clone [the UR_ROS2_GZ repo](git@github.com:UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git) and install the dependencies with
```bash
vcs import < ur_simulation_gz.jazzy.repos

Export compilation database to have intellisense:
```bash
colcon build --packages-select basic_pick_and_place_ur --cmake-args ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
```

Set `use_sim_time` to `true` when running in simulation:
```bash
ros2 run basic_pick_and_place_ur robot_painter --ros-args -p use_sim_time:=true
```

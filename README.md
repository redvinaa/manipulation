# Robotic manipulation

This repo stores my code for going through the [MIT course](https://manipulation.csail.mit.edu/index.html) on robotic manipulation.

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

*Disclaimer*: because this is just practice code, there isn't excessive documentation, check the source code.

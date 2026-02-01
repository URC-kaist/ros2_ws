### Quick memo on useful commands

```bash
ros2 run tf2_tools view_frames
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


Launch ros2_ws (please change sim to real for rover.):

```bash
# under rover/ros2-ws/
source install/setup.bash

# General entry
ros2 launch mr2_launch rover.launch.py mode:=sim

# Autonomous mission entry
# You MUST ensure that EVERYTHING is brought up by rover.launch.py beforehand.
ros2 launch mr2_rover_auto navigation.launch.py mode:=sim 2>&1 | tee navigation.log

ros2 launch mr2_rover_auto navigation.launch.py mode:=sim \
  --ros-args --log-level controller_server:=debug \
  2>&1 | tee navigation.log

```

Scripts to host and recieve web:

``` bash
# Rover hosting:
# under rover/
. scripts/deploy_dashboard.bash 2> dashboard_err

# Base station SIK interface:
# under base/gateway/
npm start -- --device /tmp/sik_sim1 --baud 57600 --port 8081
```
cd ~/mr2-stack/scripts
./can0.bash can0
cd ~/mr2-stack/rover/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mr2_launch rover_real.launch.py can_iface:=can0 enable_manipulator_module:=false enable_video_streaming:=true enable_latency_diagnostics:=true video_base_host:=192.168.1.101 enable_autonomous_module:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mr2_rover_description.launch_common import (
    controller_spawners,
    declare_can_iface,
    declare_controller_config,
    declare_use_mock_servos,
    mock_servo_nodes,
    robot_description_from_xacro,
    robot_state_publisher_node,
    ros2_control_node,
)


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "rover.urdf.xacro"])
    default_controller = PathJoinSubstitution(
        [desc_pkg, "config", "controllers", "rover_controllers.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_manipulator_module = LaunchConfiguration("enable_manipulator_module")
    enable_autonomous_module = LaunchConfiguration("enable_autonomous_module")
    start_manipulator_controllers_active = LaunchConfiguration(
        "start_manipulator_controllers_active"
    )
    controller_spawn_delay = LaunchConfiguration("controller_spawn_delay")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time; normally false for hardware",
    )
    enable_manipulator_module_arg = DeclareLaunchArgument(
        "enable_manipulator_module",
        default_value="true",
        description="Enable manipulator URDF, ros2_control, and MoveIt2 components",
    )
    enable_autonomous_module_arg = DeclareLaunchArgument(
        "enable_autonomous_module",
        default_value="true",
        description="Enable autonomous camera module (front_camera) in URDF",
    )
    controller_spawn_delay_arg = DeclareLaunchArgument(
        "controller_spawn_delay",
        default_value="2.0",
        description="Delay (seconds) before spawning ros2_control controllers",
    )
    start_manipulator_controllers_active_arg = DeclareLaunchArgument(
        "start_manipulator_controllers_active",
        default_value="false",
        description="Activate manipulator and gripper controllers immediately after spawning",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface connected to the rover hardware",
    )

    controller_config, controller_config_arg = declare_controller_config(
        default_value=default_controller,
        description="YAML file with controller manager configuration",
    )

    use_mock_servos, use_mock_servos_arg = declare_use_mock_servos(
        description="Start mock AK servo nodes that emulate the manipulator CAN motors",
    )

    robot_description = robot_description_from_xacro(
        xacro_file,
        {
            "ros2_control_mode": "real_hardware",
            "can_iface": can_iface,
            "ros2_control_config": controller_config,
            "enable_manipulator_module": enable_manipulator_module,
            "enable_autonomous_module": enable_autonomous_module,
        },
    )

    rsp = robot_state_publisher_node(robot_description, use_sim_time)

    # The real CAN hardware exposes only actuated joints. MoveIt Servo still
    # needs the passive rocker state before it considers the robot state
    # complete and starts forwarding arm commands.
    passive_rocker_joint_state = Node(
        package="mr2_rover_description",
        executable="static_joint_state_publisher",
        name="passive_rocker_joint_state",
        output="screen",
        condition=IfCondition(enable_manipulator_module),
        parameters=[
            {"joint_names": ["left_rocker_joint"]},
            {"positions": [0.0]},
            {"publish_rate": 10.0},
            {"use_sim_time": use_sim_time},
        ],
    )

    mock_servos = mock_servo_nodes(
        can_iface,
        motor_ids=range(1, 7),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_manipulator_module,
                    "' == 'true' and '",
                    use_mock_servos,
                    "' == 'true'",
                ]
            )
        ),
    )

    ros2_control = ros2_control_node(controller_config, robot_description, use_sim_time)
    spawners = controller_spawners(
        ["joint_state_broadcaster", "rover_controller"],
        start_after=controller_spawn_delay,
        interval=2.0,
    )
    manipulator_active_spawner = TimerAction(
        period=PythonExpression([controller_spawn_delay, " + 4.0"]),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["manipulator_controller", "gripper_controller"],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_manipulator_module,
                            "' == 'true' and '",
                            start_manipulator_controllers_active,
                            "' == 'true'",
                        ]
                    )
                ),
            )
        ],
    )
    manipulator_inactive_spawner = TimerAction(
        period=PythonExpression([controller_spawn_delay, " + 4.0"]),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["manipulator_controller", "gripper_controller", "--inactive"],
                output="screen",
                condition=IfCondition(enable_manipulator_module),
            )
        ],
        condition=UnlessCondition(start_manipulator_controllers_active),
    )

    battery_monitor = Node(
        package="mr2_battery_monitor",
        executable="battery_monitor_node",
        name="battery_1",
        output="screen",
        parameters=[{"can_iface": can_iface}],
        remappings=[
            ("battery/telemetry", "battery_1/telemetry"),
            ("battery/state", "battery_1/state"),
        ],
    )

    battery_monitor_secondary = Node(
        package="mr2_battery_monitor",
        executable="battery_monitor_node",
        name="battery_2",
        output="screen",
        parameters=[
            {
                "can_iface": can_iface,
                "summary_can_id": 0x320,
                "metadata_can_id": 0x321,
                "cell_base_can_id": 0x330,
            }
        ],
        remappings=[
            ("battery/telemetry", "battery_2/telemetry"),
            ("battery/state", "battery_2/state"),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            enable_manipulator_module_arg,
            enable_autonomous_module_arg,
            controller_spawn_delay_arg,
            start_manipulator_controllers_active_arg,
            can_iface_arg,
            controller_config_arg,
            use_mock_servos_arg,
            rsp,
            passive_rocker_joint_state,
            *mock_servos,
            ros2_control,
            *spawners,
            manipulator_active_spawner,
            manipulator_inactive_spawner,
            battery_monitor,
            battery_monitor_secondary,
        ]
    )

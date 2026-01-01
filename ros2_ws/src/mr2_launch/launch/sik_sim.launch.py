from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Always used in sim mode; condition allows caller to turn it off if needed.
    condition = IfCondition(LaunchConfiguration("enable_sik_sim", default="true"))

    socat_pty = ExecuteProcess(
        cmd=[
            "socat",
            "-d",
            "-d",
            "pty,raw,echo=0,link=" + LaunchConfiguration("sik_sim_device"),
            "pty,raw,echo=0,link=" + LaunchConfiguration("sik_sim_peer"),
        ],
        output="screen",
        condition=condition,
    )

    return LaunchDescription([socat_pty])

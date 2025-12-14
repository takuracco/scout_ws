# scout_gazebo/launch/minimal.launch.py
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def _pkg_share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        here = os.path.dirname(__file__)
        return os.path.normpath(os.path.join(here, ".."))

def generate_launch_description():
    PKG   = "scout_gazebo"
    SHARE = _pkg_share(PKG)

    # 引数
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("rviz")
    args = [
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("rviz",    default_value="true"),
    ]

    # パス決め
    urdf = os.path.join(SHARE, "urdf", "scout.urdf")   # 必要なら scout.xacro に変更
    world = os.path.join(SHARE, "worlds", "slope.world")  # 無ければ default.world に

    # robot_description
    robot_description = (
        Command(["xacro", urdf]) if urdf.endswith(".xacro")
        else Command(["bash", "-lc", f'cat "{urdf}"'])
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Gazebo（Factory を必ずロード）
    gzserver = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", world],
        output="screen",
        # 必要ならモデルパスを限定（警告対策）
        # env={"GAZEBO_MODEL_PATH": os.path.expanduser("~/.gazebo/models") + ":" + os.path.join(SHARE, "models")},
    )
    # GUI
    gzclient = ExecuteProcess(
        cmd=["gazebo", "--verbose", "--gui"], output="screen", condition=IfCondition(use_gui)
    )

    # spawn（少し遅らせる）
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_scout",
        output="screen",
        arguments=[
            "-topic","robot_description",
            "-entity","scout",
            "-x","-1","-y","6","-z","0.24","-Y","0"
        ],
    )
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    # 任意: RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
        arguments=[
            "-d", os.path.join(SHARE, "rviz", "scout_gazebo.rviz")
        ] if os.path.exists(os.path.join(SHARE, "rviz", "scout_gazebo.rviz")) else []
    )

    return LaunchDescription(args + [rsp, gzserver, gzclient, delayed_spawn, rviz])

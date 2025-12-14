# scout_gazebo/launch/launch.py
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def _pkg_share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        # 開発中（install されていない）でも動くフォールバック
        here = os.path.dirname(__file__)
        return os.path.normpath(os.path.join(here, ".."))

def generate_launch_description():
    PKG = "scout_gazebo"
    SHARE = _pkg_share(PKG)

    # ------------- 引数（デフォルト ON）-------------
    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="true")
    rviz_arg    = DeclareLaunchArgument("rviz",    default_value="true")

    use_gui = LaunchConfiguration("use_gui")
    rviz    = LaunchConfiguration("rviz")

    # ------------- 固定パス（install or src）-------------
    urdf_candidates = [
        os.path.join(SHARE, "urdf", "scout.urdf"),
        os.path.join(SHARE, "urdf", "scout.xacro"),
    ]
    world_candidates = [
        os.path.join(SHARE, "worlds", "inspection.world"),
        os.path.join(SHARE, "worlds", "default.world"),
    ]
    URDF = next((p for p in urdf_candidates if os.path.exists(p)), urdf_candidates[0])
    WORLD = next((p for p in world_candidates if os.path.exists(p)), world_candidates[0])

    # ------------- robot_description -------------
    robot_description = (
        Command(["xacro", URDF]) if URDF.endswith(".xacro")
        else Command(["cat ", URDF])       
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # ------------- Gazebo -------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_pkg_share("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"verbose": "true", "world": WORLD}.items(),
    )

    # robot を Gazebo に spawn（Gazebo起動後に遅延実行）
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_scout",
        output="screen",
        arguments=[
  "-topic","robot_description","-entity","scout",
  "-x","0","-y","0","-z","0.24","-Y","0"   # ← z を 0.15〜0.25m に
],
    )
    delayed_spawn = TimerAction(period=1.0, actions=[spawn])

    # ------------- GUI / RViz（条件付き）-------------
    # jsp_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     condition=IfCondition(use_gui),
    #     output="screen",
    # )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        output="screen",
        arguments=["-d", os.path.join(SHARE, "rviz", "scout_gazebo.rviz")]
    )

    cmd_joy = Node(
        package='cmd_vel_joy',          
        executable='joy_cmd',
        name='joy_cmd',  
        output='screen'    
    )
    
    joy = Node(
        package='joy',          
        executable='joy_node',
        name='joy_node',  
        output='screen'    
    )

    ld = LaunchDescription([
        use_gui_arg, rviz_arg,
        rsp, gazebo, delayed_spawn,
        # jsp_gui, 
        rviz2, cmd_joy, joy
    ])
    return ld

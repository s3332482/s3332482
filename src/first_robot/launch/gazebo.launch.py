from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join 
from launch.substitutions import Command
from pathlib import Path

def generate_launch_description():

    # Start a simulation with the cafe world
    cafe_world_uri = join(get_package_share_directory("krytn"), "models", "simple_gamecity_world.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    base_path = join(get_package_share_directory("first_robot"))

    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", '-r ' + cafe_world_uri)])

    # Step 1. Process robot file. 
    robot = IncludeLaunchDescription(join(base_path, "launch","spawn_robot.launch.py"))

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' ],
        output='screen'
        )

    # The controller is setup to put everything in the /krytn frame. We need to construct a static transform to bring it back into the un-namespaced frames. 
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "/base_footprint", "base_footprint"])

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    # Step 5: Enable the ros2 controllers
    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["-c","/controller_manager",
                             'joint_state_broadcaster', 'diff_drive_base_controller'],
                output="screen",
            )

    return LaunchDescription([gazebo_sim, bridge, static_pub, robot, 
                              robot_steering, start_controllers])
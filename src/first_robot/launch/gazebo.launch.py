from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join 
from launch.substitutions import Command
from pathlib import Path
import os

def generate_launch_description():

    # Start a simulation with the cafe world
    cafe_world_uri = join(get_package_share_directory("krytn"), "models", "simple_gamecity_world.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    base_path = join(get_package_share_directory("first_robot"))

    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", '-r ' + cafe_world_uri)])

    # Step 1. Process robot file. 
    robot = IncludeLaunchDescription(join(base_path, "launch","spawn_robot.launch.py"))

    # The controller is setup to put everything in the /krytn frame. We need to construct a static transform to bring it back into the un-namespaced frames. 
    static_pub = Node(package="tf2_ros", 
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "/base_footprint", "base_footprint"])

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )
    
    resources_package = 'first_robot_freecad'

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))

    # Gazebo hint for resources.
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_to_share_dir_clipped

    # Ensure `SDF_PATH` is populated since `sdformat_urdf` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    return LaunchDescription([gazebo_sim, static_pub, robot, 
                              robot_steering])
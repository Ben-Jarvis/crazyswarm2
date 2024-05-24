import os
import yaml
import time
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    print('======================================= START ==========================================')
    robot_prefixes = []
    for robot in crazyflies['robots']: 
        robot_prefixes.append('/'+robot)
    print(robot_prefixes)
    print('====================================== FINISH ===========================================')

    # Get the current timestamp for the bag file
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    
    # server params5
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_contents = yaml.safe_load(ymlfile)
    
    server_params = [crazyflies] + [server_yaml_contents["/crazyflie_server"]["ros__parameters"]]

    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    server_params[1]["robot_description"] = robot_desc
    
    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(get_package_share_directory('crazyflie'),'config','motion_capture.yaml')
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_params["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    # copy relevent settings to server params
    server_params[1]["poses_qos_deadline"] = motion_capture_params["topics"]["poses"]["qos"]["deadline"]

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        Node(
            package='symbiotic_crazyswarm',
            executable='my_vel_mux',
            name='my_vel_mux',
            output='screen',
            parameters=[{'hover_height': 1.5},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefixes': robot_prefixes}]
        ),
        # # Node(
        # #     package='crazyflie',
        # #     executable='crazyflie_server.py',
        # #     name='crazyflie_server',
        # #     output='screen',
        # #     parameters=server_params
        # # ),
        Node(
            package='symbiotic_crazyswarm',
            executable='swarm_utils',
            name='swarm_utils',
            output='screen',
            parameters=[{'robot_prefixes': robot_prefixes}]
        ),
        Node(
            package='symbiotic_crazyswarm',
            executable='swarm_pilot',
            name='swarm_pilot',
            output='screen',
            parameters=[{'robot_prefixes': robot_prefixes}]
        ),
        Node(
            package='symbiotic_crazyswarm',
            executable='rc_pilot',
            name='rc_pilot',
            output='screen',
        ),
        # Record all topics except the camera topics
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a',
                 '--output',os.path.join('src/symbiotic_crazyswarm/data/bags',timestamp)] ,
            output='screen'
        ),
    ])


        # Node(
        #     package='crazyflie',
        #     executable='crazyflie_server',
        #     condition=LaunchConfigurationEquals('backend','cpp'),
        #     name='crazyflie_server',
        #     output='screen',
        #     parameters=server_params,
        #     prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        # ),

        # Node(
        #     package='crazyflie',
        #     executable='crazyflie_server.py',
        #     name='crazyflie_server',
        #     output='screen',
        #     parameters=server_params
        # ),

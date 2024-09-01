import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
import yaml



def generate_launch_description():
    # Path to the world file
    gazebo_world_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'worlds',
        'altair_world.sdf'
    )
    spawn_world = ExecuteProcess(
            cmd=['gz', 'sim', '-v','-r', gazebo_world_path],
            output='screen'
        )
    
    topic_brige_path = os.path.join(
    get_package_share_directory('altair_simulation'),
    'config',
    'bridge.yaml'
    )

    # Path to the robot model file
    # already added in the altair_world.sdf file
    robot_model_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robot.urdf'
    )
    
    robot_urdf_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robot.urdf'
    )
    robot_xacro_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robotis_op3.urdf.xacro'
    )

    convert_urdf_xacro = ExecuteProcess(
        cmd=['xacro', robot_xacro_path, '-o', robot_urdf_path],
        output='screen'
    )

    robot_desc = xacro.process_file(robot_xacro_path)

    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', "Robot",
        # '-file', robot_urdf_path,
        '-topic', 'robot_description',
        '-z', '2'
    ],
    output='screen',
    )

    # joint_state_publisher_gui
    joint_state_publisher_gui = Node(
     package='joint_state_publisher_gui',
     executable='joint_state_publisher_gui',
     name='joint_state_publisher_gui',
     arguments=[robot_model_path],
     output=['screen']
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc.toxml()},
    ]
    )
    # TODO: publish from ros space joint state to gazebo space joint state
    # gz topic -t /model/robotis_op3/joint/head_pan/0/cmd_pos -m gz.msgs.Double -p 'data:0.0'

    robot_count = 1
    robot_name = ["robot1", "robot2", "robot3"]
    joint_name = ["l_hip_yaw", 
                  "l_hip_roll",
                  "l_hip_pitch",
                  "l_knee",
                  "l_ank_roll",
                  "l_ank_pitch",
                  "r_hip_yaw",
                  "r_hip_roll",
                  "r_hip_pitch",
                  "r_knee",
                  "r_ank_roll",
                  "r_ank_pitch",
                  "l_sho_pitch",
                  "l_sho_roll",
                  "l_el",
                  "r_sho_pitch",
                  "r_sho_roll",
                  "r_el",
                  "head_pan",
                  "head_tilt"]
    for i in range(robot_count):
        for j in range(20):
            bridge_list = {
                "ros_topic_name": f"{robot_name[i]}/gazebo/joint_states/{joint_name[j]}", 
                "gz_topic_name": f"model/robotis_op3/joint/{joint_name[j]}/0/cmd_pos", 
                "ros_type_name": "sensor_msgs/msg/JointState", 
                "gz_type_name": "gz.msgs.Model", 
                "direction": "ROS_TO_GZ"
                }
            with open(topic_brige_path, 'r') as file:
                data = yaml.safe_load(file)
            data.append(bridge_list)
            with open(topic_brige_path, 'w') as file:
                yaml.safe_dump(data, file)
        
    topic_brige = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={topic_brige_path}',
        ],
        output='screen',
    )
    
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['gazebo/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        # Launch Gazebo simulator
        spawn_world,
        # convert_urdf_xacro,
        # spawn_robot,
        joint_state_publisher_gui,
        topic_brige,
        camera_bridge,
        robot_state_publisher,

    ])
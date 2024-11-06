import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    motor_controller = Node(
      package='pololu', 
      executable='smc',
      output='screen', 
      emulate_tty=True,
    )
    
    joy_config = LaunchConfiguration('joy_config')
    joy_config_path = LaunchConfiguration('joy_config_path')

    teleop_twist = Node(
      package='teleop_twist_joy',
      executable='teleop_node',
      #parameters=[joy_config_path], # {'config_filepath', joy_config_path}
      output='screen', 
      emulate_tty=True,
    )  

    joystick = Node(
      package='joy',
      executable='joy_node',
      name='joy_node',
      output='screen', 
      emulate_tty=True,
    )              
     
    '''
      remappings=[
          ("/video_source/raw", "/jetbot/camera/image_raw"),
      ],
    '''
              
    return LaunchDescription([
        
        DeclareLaunchArgument('joy_config', default_value='ps3'),
        DeclareLaunchArgument('joy_config_path', default_value=[
            PathJoinSubstitution([
                FindPackageShare('teleop_twist_joy'), 
                'config',
                joy_config,
                '.config.yaml'
            ]),
        ]),
        
        #motor_controller,
        teleop_twist,
        joystick,
    ])

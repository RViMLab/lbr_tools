from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Load robot description
    description_package = LaunchConfiguration('description_package').perform(context)
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare(LaunchConfiguration('description_package')), 'urdf/{}.urdf.xacro'.format(description_package).replace('_description', '')]
            ), " ",
            "robot_name:=", LaunchConfiguration('robot_name')
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Create required nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([FindPackageShare('lbr_tools'), 'config/config.rviz'])
        ]
    )

    return [
        robot_state_publisher_node,
        rviz
    ]

# for reference see
# https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot_description
def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(DeclareLaunchArgument(
        name='description_package',
        default_value='da_vinci_endoscope_description',
        description='Description package.'
    ))

    launch_args.append(DeclareLaunchArgument(
        name='description_file',
        default_value='urdf/da_vinci_endoscope.urdf.xacro',
        description='Path to URDF file, relative to description_package.'
    ))

    launch_args.append(DeclareLaunchArgument(
        name='robot_name',
        default_value='lbr',
        description='Set robot name.'
    ))

    return LaunchDescription(
        launch_args + [
            OpaqueFunction(function=launch_setup)
    ])

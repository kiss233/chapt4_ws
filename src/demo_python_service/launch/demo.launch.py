import launch
import launch_ros

def generate_launch_description():
# 声明一个launch的参数
    action_declare_arg_times_upsample = launch.actions.DeclareLaunchArgument('times_to_upsample',default_value='1')

    action_node_face_detect=launch_ros.actions.Node(
        package='demo_python_service',
        executable ='face_detect_node',
        parameters=[{'number_of_times_to_upsample':launch.substitutions.LaunchConfiguration(
          'times_to_upsample',default='1'
        )}], #将launch中的times_to_upsample参数转换为节点的参数
        output='screen'
    )

    action_node_face_detect_client =launch_ros.actions.Node(
        package='demo_python_service',
        executable ='face_detect_client_node',
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_times_upsample,
        action_node_face_detect,
        action_node_face_detect_client,
    ])
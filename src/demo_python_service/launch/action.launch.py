import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #设置参数用于条件控制action_topic_list的启动
    action_declare_start_topic = launch.actions.DeclareLaunchArgument('startup_topic',default_value='False')
    startup_topic2 = launch.substitutions.LaunchConfiguration('startup_topic')
    
    # 动作1：启动其他的launch
    multisim_launch_path = [get_package_share_directory('turtlesim'),'/launch','/multisim.launch.py'] #turtlesim为功能包名，/multisim.launch.py为launch文件的名字，这里是拼接为了启动文件的地址。注意整体是数组所以要方括号
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )

    # 动作2:打印数据
    action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))

    #动作3：执行进程
    action_topic_list = launch.actions.ExecuteProcess(
        condition = launch.conditions.IfCondition(startup_topic2),
        cmd=['ros2','topic','list',],output='screen' 
    )

    #动作4：组织动作成组，把多个动作放在一组
    action_group = launch.actions.GroupAction([
        #动作5：定时器
        launch.actions.TimerAction(period=2.0,actions=[action_include_launch]), #利用定时器依次启动程序，period是周期，2s启动action_include_launch，4s启动action_topic_list
        launch.actions.TimerAction(period=4.0,actions=[action_topic_list])
    ])

    return launch.LaunchDescription([
        action_declare_start_topic,
        action_log_info,
        action_group
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

# Example ROS launch file
def generate_launch_description():
    
    example_control = Node(
        package='shell_simulation',
        namespace='',
        executable='example_control',
        name='example_control',
        output='screen'
    )
    
    brain_node = Node(
        package='roben_ai',
        namespace='',
        executable='brain.py',
        name='brain',
        output='screen'
    )
    
    return LaunchDescription([  
        # Nodes
        example_control,
        brain_node
    ])
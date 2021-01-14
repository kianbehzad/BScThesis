from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('pack_util'), 'params', 'config_params.yaml')
    
    grsim_node = Node(package='pack_protobuf_wrapper', executable='grsim', name='grsim_node', parameters=[param_path])
    vision_node = Node(package='pack_protobuf_wrapper', executable='vision', name='vision_node', parameters=[param_path])
    worldmodel_node = Node(package='pack_world_model', executable='worldmodel', name='worldmodel_node', parameters=[param_path])
    #interface_node = Node(package='pack_gui', executable='interface', name='interface_node', parameters=[param_path])

    return LaunchDescription([grsim_node, vision_node, worldmodel_node])


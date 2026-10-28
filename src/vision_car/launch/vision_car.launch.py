"""三节点启动入口，统一读取 car_bringup/config/robot.yaml。"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    """合并统一配置和显式启动参数；拒绝模拟检测驱动实体底盘。"""
    config = Path(LaunchConfiguration('config').perform(context))
    data = yaml.safe_load(config.read_text())
    vision = data['vision']
    camera, detector, control = (dict(vision[key]) for key in ('camera','detector','control'))
    camera['simulate'] = LaunchConfiguration('simulate_camera').perform(context).lower() == 'true'
    detector['simulate'] = LaunchConfiguration('simulate_detector').perform(context).lower() == 'true'
    control['driver'] = LaunchConfiguration('driver').perform(context)
    control['hardware_enabled'] = LaunchConfiguration('hardware_enabled').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    if model:
        detector['model'] = model
    camera['video_file'] = LaunchConfiguration('video_file').perform(context)
    if control['driver'] != 'mock' and (camera['simulate'] or detector['simulate'] or camera['video_file']):
        raise ValueError('禁止用模拟图像/模拟检测驱动实体硬件')
    if control['driver'] != 'mock' and not control['hardware_enabled']:
        raise ValueError('实体硬件未显式使能')
    control['max_linear'] = float(data['limits']['max_linear_x'])
    control['max_angular'] = float(data['limits']['max_angular_z'])
    camera.update(image_topic=data['topics']['camera'], frame_id=data['robot']['camera_frame'])
    detector.update(image_topic=data['topics']['camera'], bbox_topic=vision['topics']['bbox'],
                    annotated_topic=vision['topics']['annotated'])
    control.update(bbox_topic=vision['topics']['bbox'], command_topic=vision['topics']['command'],
                   state_topic=vision['topics']['state'])
    # 全部细项均可在 config 文件调整；机器相关路径不写进 launch。
    return [Node(package='vision_car', executable=name, name=name, output='screen', parameters=[params])
            for name,params in [('camera_node',camera),('yolo_detect_node',detector),('control_node',control)]]


def generate_launch_description():
    """声明安全的默认参数：合成相机、模拟检测和模拟电机。"""
    default = str(Path(get_package_share_directory('car_bringup'))/'config'/'robot.yaml')
    args = [('config',default),('simulate_camera','true'),('simulate_detector','true'),
            ('driver','mock'),('hardware_enabled','false'),('model',''),('video_file','')]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name,value in args]
                             + [OpaqueFunction(function=create_nodes)])

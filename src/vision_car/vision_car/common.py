"""ROS 节点公共生命周期工具。"""
import rclpy
from rclpy.executors import ExternalShutdownException


def parameters(node, defaults):
    """声明并读取启动参数。"""
    return {k: node.declare_parameter(k, v).value for k, v in defaults.items()}


def run(factory):
    """启动节点，确保 Ctrl+C 时执行硬件和工作进程清理。"""
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

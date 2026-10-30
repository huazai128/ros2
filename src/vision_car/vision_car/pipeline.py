"""图像链路的队列、时间戳与 QoS 工具。"""

import queue


def put_latest(output, item):
    """单生产者队列满时移除旧帧；feeder 尚未可读时有界等待。"""
    try:
        output.put_nowait(item)
        return True
    except queue.Full:
        pass
    try:
        output.get(timeout=0.005)
    except queue.Empty:
        return False
    try:
        output.put_nowait(item)
        return True
    except queue.Full:
        return False


def stamp_seconds(stamp):
    """将 ROS 原始时间戳转换为秒。"""
    return stamp.sec + stamp.nanosec / 1e9


def latest_image_qos():
    """图像允许丢旧帧，DDS 最多保留一个待取样本。"""
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

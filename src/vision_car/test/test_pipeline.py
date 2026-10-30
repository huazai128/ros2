"""无需 ROS 安装的队列、事件通知和检测时效回归测试。"""

import importlib.util
import json
import multiprocessing as mp
import queue
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from vision_car.pipeline import put_latest


def load_detector():
    """仅替代 ROS/图像依赖，测试真实节点方法，不冒充 ROS 集成。"""
    names = (
        "cv2",
        "cv_bridge",
        "rclpy.node",
        "sensor_msgs.msg",
        "std_msgs.msg",
        "vision_car.common",
    )
    modules = {name: MagicMock() for name in names}
    modules["rclpy.node"].Node = object
    modules["std_msgs.msg"].String = lambda **kwargs: SimpleNamespace(**kwargs)
    source = Path(__file__).parents[1] / "vision_car" / "yolo_detect_node.py"
    spec = importlib.util.spec_from_file_location("vision_car._tested_detector", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module


detector = load_detector()


class PipelineTests(unittest.TestCase):
    """验证旧帧替换以及并发通知关闭行为。"""

    def test_full_queue_replaces_old_frame(self):
        output = queue.Queue(1)
        self.assertTrue(put_latest(output, "old"))
        self.assertTrue(put_latest(output, "new"))
        self.assertEqual(output.get_nowait(), "new")

    def test_multiprocessing_queue_replaces_old_frame(self):
        output = mp.get_context("spawn").Queue(1)
        try:
            self.assertTrue(put_latest(output, "old"))
            # 等待 feeder 提交旧帧，避免把 OS 调度延迟当作替换失败。
            self.assertTrue(output._reader.poll(1))
            self.assertTrue(put_latest(output, "new"))
            self.assertEqual(output.get(timeout=1), "new")
        finally:
            output.close()
            output.join_thread()

    def test_unreadable_feeder_does_not_block_capture(self):
        output = MagicMock()
        output.put_nowait.side_effect = queue.Full
        output.get.side_effect = queue.Empty
        self.assertFalse(put_latest(output, "new"))
        output.get.assert_called_once_with(timeout=0.005)

    def test_receiver_wakes_executor_without_poll_timer(self):
        receive, send = mp.Pipe(duplex=False)
        inbox = queue.Queue()
        stopped = threading.Event()
        notified = threading.Event()
        guard = SimpleNamespace(trigger=notified.set)
        thread = threading.Thread(
            target=detector.receive_results,
            args=(receive, inbox, stopped, guard),
        )
        thread.start()
        try:
            send.send(("result", {"boxes": []}))
            self.assertTrue(notified.wait(1))
            self.assertEqual(inbox.get_nowait()[0], "result")
        finally:
            stopped.set()
            send.close()
            thread.join(1)
            receive.close()
        self.assertFalse(thread.is_alive())
        self.assertTrue(inbox.empty())

    def test_pipe_eof_reports_fault(self):
        receive, send = mp.Pipe(duplex=False)
        inbox = queue.Queue()
        guard = MagicMock()
        send.close()
        try:
            detector.receive_results(receive, inbox, threading.Event(), guard)
            self.assertEqual(inbox.get_nowait()[0], "error")
            guard.trigger.assert_called_once()
        finally:
            receive.close()


class DetectorTimingTests(unittest.TestCase):
    """验证过期输入/结果会被拒绝，预览不参与结果发布。"""

    def setUp(self):
        stamp = SimpleNamespace(sec=99, nanosec=900_000_000)
        self.msg = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp),
            width=640,
            height=480,
        )
        self.node = SimpleNamespace(
            p={
                "image_timeout": 1.0,
                "inference_timeout": 2.0,
                "annotated_hz": 2.0,
                "simulate": True,
            },
            job=None,
            latest=self.msg,
            preview=None,
            ready=True,
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)
            ),
            bridge=MagicMock(),
            requests=queue.Queue(1),
            pub=MagicMock(),
            fault=MagicMock(),
        )

    def test_idle_detector_schedules_on_image_arrival(self):
        self.node.schedule = lambda: detector.YoloDetectNode.schedule(self.node)
        detector.YoloDetectNode.image(self.node, self.msg)
        self.assertIsNotNone(self.node.job)
        self.assertFalse(self.node.requests.empty())

    def test_busy_detector_keeps_only_latest(self):
        self.node.job = ("in-flight",)
        self.node.schedule = lambda: detector.YoloDetectNode.schedule(self.node)
        detector.YoloDetectNode.image(self.node, self.msg)
        newest = SimpleNamespace(header=self.msg.header)
        detector.YoloDetectNode.image(self.node, newest)
        self.assertIs(self.node.latest, newest)
        self.assertTrue(self.node.requests.empty())

    def test_stale_frame_never_enters_worker(self):
        self.msg.header.stamp.sec = 98
        detector.YoloDetectNode.schedule(self.node)
        self.assertTrue(self.node.requests.empty())
        self.node.bridge.imgmsg_to_cv2.assert_not_called()
        self.node.fault.assert_called_once()

    def test_future_frame_never_enters_worker(self):
        self.msg.header.stamp.sec = 101
        detector.YoloDetectNode.schedule(self.node)
        self.node.fault.assert_called_once()
        self.assertTrue(self.node.requests.empty())

    def test_result_publishes_timing_before_preview(self):
        self.node.job = (self.msg, "source-frame", time.monotonic())
        detector.YoloDetectNode.publish_result(
            self.node, {"boxes": [], "inference_ms": 12.0}
        )
        packet = json.loads(self.node.pub.publish.call_args.args[0].data)
        self.assertEqual(packet["inference_ms"], 12.0)
        self.assertAlmostEqual(packet["result_age_ms"], 100.0)
        self.assertTrue(packet["ok"])
        self.assertIsNone(self.node.job)
        self.assertIsNotNone(self.node.preview)
        self.node.bridge.cv2_to_imgmsg.assert_not_called()

    def test_late_result_fails_closed_even_before_watchdog(self):
        self.node.job = (self.msg, "frame", time.monotonic() - 3)
        detector.YoloDetectNode.publish_result(
            self.node, {"boxes": [], "inference_ms": 3000.0}
        )
        self.node.pub.publish.assert_not_called()
        self.node.fault.assert_called_once()

    def test_stale_result_is_not_published(self):
        self.msg.header.stamp.sec = 98
        self.node.job = (self.msg, "frame", time.monotonic())
        detector.YoloDetectNode.publish_result(
            self.node, {"boxes": [], "inference_ms": 12.0}
        )
        self.node.pub.publish.assert_not_called()
        self.node.fault.assert_called_once()

    def test_preview_disabled_does_not_retain_frame(self):
        self.node.p["annotated_hz"] = 0.0
        self.node.job = (self.msg, "frame", time.monotonic())
        detector.YoloDetectNode.publish_result(
            self.node, {"boxes": [], "inference_ms": 12.0}
        )
        self.assertIsNone(self.node.preview)

    def test_preview_without_subscribers_skips_work(self):
        self.node.preview = (self.msg, MagicMock(), [])
        self.node.annotated = MagicMock()
        self.node.annotated.get_subscription_count.return_value = 0
        detector.YoloDetectNode.publish_preview(self.node)
        self.node.bridge.cv2_to_imgmsg.assert_not_called()
        self.assertIsNone(self.node.preview)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""验证实际 OpenCV 卡尔曼滤波：不规则采样、外推、预测状态隔离和预热。"""

import unittest

import cv2
import numpy as np

from mac_yolo_tracker import MotionTrack, draw_track


class KalmanTests(unittest.TestCase):
    def test_irregular_timestamps_and_future_position(self):
        track = MotionTrack(cv2, np, (100, 200), 0.0, 20)
        stamp = 0.0
        for index in range(60):
            stamp += (0.03, 0.07, 0.12)[index % 3]
            track.update((100 + 80 * stamp, 200 - 25 * stamp), stamp)
        x, y, vx, vy, fx, fy = track.state(stamp, 0.5)
        self.assertAlmostEqual(vx, 80, delta=2)
        self.assertAlmostEqual(vy, -25, delta=2)
        self.assertAlmostEqual(fx, 100 + 80 * (stamp + 0.5), delta=3)
        self.assertAlmostEqual(fy, 200 - 25 * (stamp + 0.5), delta=3)
        self.assertEqual(len(track.history), 20)
        self.assertTrue(track.ready())

    def test_forecast_does_not_mutate_filter(self):
        track = MotionTrack(cv2, np, (10, 20), 0.0, 10)
        for index in range(1, 15):
            track.update((10 + index * 5, 20), index * 0.1)
        before = track.filter.statePost.copy()
        x, y, vx, vy, fx, fy = track.state(1.7, 0.5)
        np.testing.assert_array_equal(before, track.filter.statePost)
        self.assertAlmostEqual(x, before[0, 0] + vx * 0.3)
        self.assertAlmostEqual(fx, x + vx * 0.5)
        self.assertEqual(track.count, 15)
        self.assertAlmostEqual(track.last_seen, 1.4)

    def test_warmup_and_invalid_time(self):
        track = MotionTrack(cv2, np, (10, 20), 1.0, 10)
        self.assertFalse(track.ready())
        self.assertEqual(track.state(1.0, 0.5)[2:4], (0.0, 0.0))
        with self.assertRaises(ValueError):
            track.update((11, 20), 1.0)

    def test_reversal_and_stop(self):
        track = MotionTrack(cv2, np, (200, 100), 0.0, 10)
        x = 200
        for index in range(1, 91):
            speed = 80 if index <= 30 else -80 if index <= 60 else 0
            x += speed * 0.1
            track.update((x, 100), index * 0.1)
            if index == 30:
                self.assertGreater(track.state(index * 0.1, 0.5)[2], 70)
            if index == 60:
                self.assertLess(track.state(index * 0.1, 0.5)[2], -70)
        self.assertAlmostEqual(track.state(9.0, 0.5)[2], 0, delta=3)

    def test_crossing_exit_and_lost_markers(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        for origin, expected in ((230, "CROSS CENTER"), (540, "EXIT FRAME")):
            track = MotionTrack(cv2, np, (origin, 200), 0.0, 20)
            for index in range(1, 11):
                track.update((origin + index * 8, 200), index * 0.1)
            values = draw_track(cv2, np, image, 1, track, 1.0, 0.5, 20, "bottle", True)
            self.assertEqual(values[-2:], ("RIGHT", expected))
            lost = draw_track(cv2, np, image, 1, track, 1.1, 0.5, 20, "bottle", False)
            self.assertEqual(lost[-2], "LOST / PREDICTED")
        self.assertTrue(image.any())


if __name__ == "__main__":
    unittest.main()

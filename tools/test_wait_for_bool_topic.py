"""Exercise delayed/mismatching state delivery without a ROS master."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch


class WaitTests(unittest.TestCase):
    def run_wait(self, deliveries, expected, timeout):
        timers = []
        removed = []

        def subscribe(topic, message_type, callback, queue_size):
            for delay, value in deliveries:
                timer = threading.Timer(delay, callback,
                                        args=[types.SimpleNamespace(data=value)])
                timers.append(timer)
                timer.start()
            return types.SimpleNamespace(unregister=lambda: removed.append(True))

        modules = {"rospy": types.SimpleNamespace(Subscriber=subscribe),
                   "std_msgs": types.ModuleType("std_msgs"),
                   "std_msgs.msg": types.SimpleNamespace(Bool=object)}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                "waiter", Path(__file__).with_name("wait_for_bool_topic.py"))
            waiter = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(waiter)
            try:
                result = waiter.wait_for_bool("/test", expected, timeout)
            finally:
                for timer in timers:
                    timer.cancel()
                    timer.join()
        self.assertEqual(removed, [True])
        return result

    def test_delivery_after_old_one_second_limit(self):
        self.assertTrue(self.run_wait([(1.2, True)], True, 2.0))

    def test_false_then_true(self):
        self.assertTrue(self.run_wait([(0.01, False), (0.05, True)], True, 0.3))

    def test_missing_state_times_out(self):
        self.assertFalse(self.run_wait([], True, 0.05))

    def test_wrong_state_times_out(self):
        self.assertFalse(self.run_wait([(0.01, False)], True, 0.05))

    def test_disable_wait_accepts_false(self):
        self.assertTrue(self.run_wait([(0.01, False)], False, 0.3))


if __name__ == "__main__":
    unittest.main()

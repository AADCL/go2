#!/usr/bin/env python3
"""Wait for a matching ROS Bool using one subscription and a wall-time deadline."""
import sys
import threading

import rospy
from std_msgs.msg import Bool


def wait_for_bool(topic, expected, timeout):
    matched = threading.Event()

    def receive(message):
        if message.data == expected:
            matched.set()

    subscriber = rospy.Subscriber(topic, Bool, receive, queue_size=1)
    try:
        return matched.wait(timeout)
    finally:
        subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("go2_wait_bool", anonymous=True, disable_signals=True)
    try:
        success = wait_for_bool(sys.argv[1], sys.argv[2].lower() == "true",
                                float(sys.argv[3]))
    finally:
        rospy.signal_shutdown("State check completed")
    sys.exit(0 if success else 1)

#!/usr/bin/env python3
"""
CP5a helper — Joy Mapping Sniffer

Subscribes to /joy and prints ONLY the axes/buttons whose value changed
since the previous message, instead of the full raw message every time.

HOW TO USE:
  1. In one terminal: ros2 run joy joy_node
  2. In another (sourced ROS2 terminal): python3 joy_mapping_sniffer.py
  3. Move ONE control at a time (one stick direction, one trigger, one
     button, one D-pad press). Only the line(s) for what you moved should
     print -- that tells you the index and its value range directly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# How much a value has to move before we print it. Without this, tiny
# stick jitter near center would spam the terminal even when you think
# you're not touching anything.
CHANGE_THRESHOLD = 0.05


class JoyMappingSniffer(Node):
    def __init__(self):
        super().__init__('joy_mapping_sniffer')
        self.prev_axes = None
        self.prev_buttons = None
        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)
        self.get_logger().info("Listening on /joy — move ONE control at a time.")

    def callback(self, msg):
        # First message: just record it as the baseline, nothing to
        # compare against yet.
        if self.prev_axes is None:
            self.prev_axes = list(msg.axes)
            self.prev_buttons = list(msg.buttons)
            return

        for i, (old, new) in enumerate(zip(self.prev_axes, msg.axes)):
            if abs(new - old) > CHANGE_THRESHOLD:
                print(f"axes[{i}]    changed:  {old:+.3f} -> {new:+.3f}")

        for i, (old, new) in enumerate(zip(self.prev_buttons, msg.buttons)):
            if old != new:
                state = "PRESSED" if new == 1 else "released"
                print(f"buttons[{i}] {state}")

        self.prev_axes = list(msg.axes)
        self.prev_buttons = list(msg.buttons)


def main():
    rclpy.init()
    node = JoyMappingSniffer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

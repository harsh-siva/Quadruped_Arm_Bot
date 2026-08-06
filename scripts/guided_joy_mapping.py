#!/usr/bin/env python3
"""
CP5a helper — Guided Joy Mapping Sniffer

Walks through each control ONE AT A TIME. For each, press Enter to start
capturing, move/press ONLY that control, then press Enter again to stop.
The script reports exactly which axis/button index changed and its
resting vs. extreme value for THAT control -- no manual matching needed.
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

STEPS = [
    "Left stick — push LEFT, hold, release",
    "Left stick — push RIGHT, hold, release",
    "Left stick — push UP, hold, release",
    "Left stick — push DOWN, hold, release",
    "Right stick — push LEFT, hold, release",
    "Right stick — push RIGHT, hold, release",
    "Right stick — push UP, hold, release",
    "Right stick — push DOWN, hold, release",
    "Left Trigger (LT) — pull fully, hold, release",
    "Right Trigger (RT) — pull fully, hold, release",
    "D-pad — press LEFT",
    "D-pad — press RIGHT",
    "Button A",
    "Button B",
    "Button Y",
    "Left Bumper (LB)",
    "Right Bumper (RB)",
]


class GuidedSniffer(Node):
    def __init__(self):
        super().__init__('guided_joy_sniffer')
        self.latest_axes = None
        self.latest_buttons = None
        self.min_axes = None
        self.max_axes = None
        self.pressed_buttons = set()
        self.recording = False
        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)

    def callback(self, msg):
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)
        if not self.recording:
            return
        if self.min_axes is None:
            self.min_axes = list(msg.axes)
            self.max_axes = list(msg.axes)
        for i, v in enumerate(msg.axes):
            self.min_axes[i] = min(self.min_axes[i], v)
            self.max_axes[i] = max(self.max_axes[i], v)
        for i, v in enumerate(msg.buttons):
            if v == 1:
                self.pressed_buttons.add(i)

    def run_step(self, label):
        input(f"\n>>> {label}\n    Press Enter, THEN perform the action, "
              f"THEN press Enter again when done.")
        self.min_axes = None
        self.max_axes = None
        self.pressed_buttons = set()
        self.recording = True
        input("    Recording... press Enter to STOP.")
        self.recording = False

        if self.min_axes is None:
            print("    (no data captured -- did joy_node see the input?)")
            return

        for i in range(len(self.min_axes)):
            lo, hi = self.min_axes[i], self.max_axes[i]
            if hi - lo > 0.05:
                print(f"    -> axes[{i}] moved: {lo:+.3f} to {hi:+.3f}")
        for i in self.pressed_buttons:
            print(f"    -> buttons[{i}] pressed")


def main():
    rclpy.init()
    node = GuidedSniffer()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("=== Guided Joy Mapping ===")
    for step in STEPS:
        node.run_step(step)

    print("\n=== Done. Scroll up to see the full mapping. ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

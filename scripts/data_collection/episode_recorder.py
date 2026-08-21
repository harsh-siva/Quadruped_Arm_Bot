#!/usr/bin/env python3
"""
episode_recorder.py — CP5 synchronized data collection (v2).
Camera-driven: on every /rgb frame, samples the latest full /joint_states
message and stamps it. Episode start/stop is triggered by pressing Enter
in the terminal (temporary — swap for a joy button once confirmed safe).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import h5py
import numpy as np
import os
import re
import glob
import threading
from datetime import datetime

CAMERA_TOPIC = "/rgb"
JOINT_STATES_TOPIC = "/joint_states"
DATA_DIR = os.path.expanduser("~/project/Quadruped_Arm_bot/data/episodes")


def slugify(text, max_len=30):
    text = text.strip().lower()
    text = re.sub(r'[^a-z0-9]+', '_', text)
    text = text.strip('_')
    return text[:max_len] or "episode"


def next_episode_index():
    existing = glob.glob(os.path.join(DATA_DIR, "ep[0-9][0-9][0-9]_*.h5"))
    nums = []
    for p in existing:
        m = re.match(r'ep(\d+)_', os.path.basename(p))
        if m:
            nums.append(int(m.group(1)))
    return (max(nums) + 1) if nums else 1


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__('episode_recorder')
        self.bridge = CvBridge()
        self.recording = False
        self.episode_file = None
        self.frame_idx = 0

        self.latest_joint_names = None
        self.latest_joint_positions = None
        self.latest_image_shape = None  # (H, W, 3), captured from first real frame
        self.lock = threading.Lock()

        os.makedirs(DATA_DIR, exist_ok=True)

        self.create_subscription(Image, CAMERA_TOPIC, self.camera_cb, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.joint_cb, 10)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        self.get_logger().info(
            f"Recorder ready. Subscribed to {CAMERA_TOPIC} and {JOINT_STATES_TOPIC}. "
            "Press Enter in this terminal to start/stop an episode."
        )

    def keyboard_loop(self):
        while rclpy.ok():
            input()
            self.toggle_recording()

    def joint_cb(self, msg: JointState):
        self.latest_joint_names = list(msg.name)
        self.latest_joint_positions = np.array(msg.position, dtype=np.float32)

    def toggle_recording(self):
        if not self.recording:
            self.start_episode()
        else:
            self.stop_episode()

    def start_episode(self):
        if self.latest_joint_positions is None:
            self.get_logger().warn("No /joint_states message received yet — can't start. Is sim playing?")
            return
        if self.latest_image_shape is None:
            self.get_logger().warn("No /rgb frame received yet — can't start. Is the camera publishing?")
            return

        instruction = input("Instruction for this episode: ")
        num_joints = len(self.latest_joint_positions)
        h, w, c = self.latest_image_shape
        idx = next_episode_index()
        fname = f"ep{idx:03d}_{slugify(instruction)}.h5"
        path = os.path.join(DATA_DIR, fname)

        ep_file = h5py.File(path, 'w')
        ep_file.attrs['instruction'] = instruction
        ep_file.attrs['episode_index'] = idx
        ep_file.attrs['recorded_at'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        ep_file.attrs['joint_names'] = np.array(self.latest_joint_names, dtype='S')
        ep_file.create_dataset('images', shape=(0, h, w, c),
                                maxshape=(None, h, w, c),
                                dtype='uint8', chunks=True)
        ep_file.create_dataset('joint_positions', shape=(0, num_joints),
                                maxshape=(None, num_joints),
                                dtype='float32', chunks=True)
        ep_file.create_dataset('timestamps', shape=(0,),
                                maxshape=(None,), dtype='float64', chunks=True)

        with self.lock:
            self.episode_file = ep_file
            self.frame_idx = 0
            self.recording = True
        self.get_logger().info(f"Recording started ({num_joints} joints): {path}")

    def stop_episode(self):
        with self.lock:
            self.recording = False
            ep_file = self.episode_file
            n = self.frame_idx
            self.episode_file = None
        if ep_file:
            ep_file.close()
            self.get_logger().info(f"Recording stopped. {n} frames saved.")

    def camera_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image_shape = img.shape  # keep this fresh even when not recording

        with self.lock:
            if not self.recording or self.episode_file is None:
                return
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            n = self.frame_idx + 1
            for name, val in [('images', img),
                               ('joint_positions', self.latest_joint_positions),
                               ('timestamps', ts)]:
                ds = self.episode_file[name]
                ds.resize((n,) + ds.shape[1:])
                ds[self.frame_idx] = val
            self.frame_idx = n


def main():
    rclpy.init()
    node = EpisodeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_episode()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

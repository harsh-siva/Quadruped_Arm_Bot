#!/usr/bin/env python3
"""
visualize_episode.py — quick sanity-check visualizer for a CP5 episode .h5 file.

Outputs, next to the input file:
  - <name>_contact_sheet.png  (grid of sampled frames, fast visual check)
  - <name>_joint_plot.png     (joint positions over time)
  - <name>_playback.mp4       (actual video of the recorded frames)

Usage:
  python3 visualize_episode.py data/episodes/episode_20260806_132503.h5
"""
import sys
import os
import h5py
import numpy as np
import matplotlib
matplotlib.use('Agg')  # no display needed, just save files
import matplotlib.pyplot as plt
import cv2


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 visualize_episode.py <path_to_episode.h5>")
        sys.exit(1)

    path = sys.argv[1]
    base = os.path.splitext(path)[0]

    with h5py.File(path, 'r') as f:
        instruction = f.attrs['instruction']
        joint_names = [n.decode() if isinstance(n, bytes) else n for n in f.attrs['joint_names']]
        images = f['images'][:]
        joint_positions = f['joint_positions'][:]
        timestamps = f['timestamps'][:]

    n_frames = images.shape[0]
    print(f"instruction: {instruction}")
    print(f"frames: {n_frames}, image shape: {images.shape[1:]}")
    print(f"joints ({len(joint_names)}): {joint_names}")
    print(f"duration: {timestamps[-1] - timestamps[0]:.2f}s")

    # 1) Contact sheet: 8 evenly sampled frames
    n_sample = min(8, n_frames)
    idxs = np.linspace(0, n_frames - 1, n_sample, dtype=int)
    fig, axes = plt.subplots(1, n_sample, figsize=(3 * n_sample, 3))
    if n_sample == 1:
        axes = [axes]
    for ax, i in zip(axes, idxs):
        ax.imshow(images[i])
        ax.set_title(f"frame {i}")
        ax.axis('off')
    fig.suptitle(f"instruction: {instruction}")
    fig.tight_layout()
    fig.savefig(f"{base}_contact_sheet.png", dpi=100)
    plt.close(fig)
    print(f"saved {base}_contact_sheet.png")

    # 2) Joint positions over time
    rel_t = timestamps - timestamps[0]
    fig, ax = plt.subplots(figsize=(10, 5))
    for j, name in enumerate(joint_names):
        ax.plot(rel_t, joint_positions[:, j], label=name, linewidth=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint position (rad)")
    ax.set_title(f"Joint positions — {instruction}")
    ax.legend(fontsize=6, ncol=2, loc='upper right')
    fig.tight_layout()
    fig.savefig(f"{base}_joint_plot.png", dpi=120)
    plt.close(fig)
    print(f"saved {base}_joint_plot.png")

    # 3) Playable video
    h, w = images.shape[1:3]
    fps = n_frames / (timestamps[-1] - timestamps[0]) if timestamps[-1] > timestamps[0] else 10
    writer = cv2.VideoWriter(f"{base}_playback.mp4", cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    for i in range(n_frames):
        frame_bgr = cv2.cvtColor(images[i], cv2.COLOR_RGB2BGR)
        writer.write(frame_bgr)
    writer.release()
    print(f"saved {base}_playback.mp4 ({fps:.1f} fps)")


if __name__ == '__main__':
    main()

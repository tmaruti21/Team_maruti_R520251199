#!/usr/bin/env python3
"""
collect_data.py  –  Labelled image collector for custom_follow
==============================================================
Use this script to build your training dataset.  It shows the live
Astra camera feed (via ROS2) and lets you press a key to instantly
save the current frame into the correct class folder.

Folder layout (created automatically):
    dataset/
        straight/
        left/
        right/
        intersection/
        dead_end/

Controls
--------
  1  –  save as  straight
  2  –  save as  left
  3  –  save as  right
  4  –  save as  intersection
  5  –  save as  dead_end
  [  –  shrink preview window
  ]  –  enlarge preview window
  Q  –  quit

Usage
-----
  # Activate venv first
  source /home/chetan-satpute/lane_following/.venv/bin/activate

  # In a separate terminal, start the camera driver:
  source /home/chetan-satpute/lane_following/install/setup.bash
  ros2 launch orbbec_camera astra_pro_plus.launch.py

  # Then run this collector:
  source /home/chetan-satpute/lane_following/install/setup.bash
  python3 collect_data.py

  # Or from a static image (no ROS2 needed):
  python3 collect_data.py --image /path/to/photo.jpg

How many images to collect
--------------------------
  Class          Minimum    Recommended
  straight         80          200
  left             80          200
  right            80          200
  intersection     60          150
  dead_end         60          150
  ─────────────────────────────────
  TOTAL           360          900

  Vary: lighting, approach distance, slight camera angles.
  More variety = better generalisation across environments.

After collecting, run:  python3 train.py
"""

import argparse
import os
import threading
import time

import cv2
import numpy as np

# ── ROS2 / camera imports (optional – skip in static-image mode) ──────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _ROS2_OK = True
except ImportError:
    _ROS2_OK = False

# ── Configuration ─────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(SCRIPT_DIR, 'dataset')

CLASSES = {
    ord('1'): 'straight',
    ord('2'): 'left',
    ord('3'): 'right',
    ord('4'): 'intersection',
    ord('5'): 'dead_end',
    ord('6'): 'right_lane_ending',
    ord('7'): 'left_lane_ending',
    # optional "miscellaneous" class for outliers
}

# Ensure folders exist
for cls in CLASSES.values():
    os.makedirs(os.path.join(DATASET_DIR, cls), exist_ok=True)


# ── Helpers ───────────────────────────────────────────────────────────────────

def count_images(cls_name):
    folder = os.path.join(DATASET_DIR, cls_name)
    return len([f for f in os.listdir(folder)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))])


def next_index(cls_name):
    folder = os.path.join(DATASET_DIR, cls_name)
    existing = [f for f in os.listdir(folder)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    indices = []
    for f in existing:
        try:
            indices.append(int(os.path.splitext(f)[0].split('_')[-1]))
        except ValueError:
            pass
    return max(indices, default=-1) + 1


def save_image(frame, cls_name):
    idx  = next_index(cls_name)
    path = os.path.join(DATASET_DIR, cls_name, f'{cls_name}_{idx:05d}.jpg')
    cv2.imwrite(path, frame)
    return path


def draw_hud(frame, display_scale, total_counts):
    """Overlay class counts and key hints on preview frame."""
    out = frame.copy()
    h, w = out.shape[:2]

    # Dark band at top
    cv2.rectangle(out, (0, 0), (w, 100), (30, 30, 30), -1)

    # Class counts
    labels = [
        f'[1] straight     : {total_counts["straight"]}',
        f'[2] left         : {total_counts["left"]}',
        f'[3] right        : {total_counts["right"]}',
        f'[4] intersection : {total_counts["intersection"]}',
        f'[5] dead_end     : {total_counts["dead_end"]}',
        f'[6] right_lane_ending : {total_counts["right_lane_ending"]}',
        f'[7] left_lane_ending  : {total_counts["left_lane_ending"]}',
    ]
    for i, txt in enumerate(labels):
        cv2.putText(out, txt, (10, 18 + i * 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # Bottom hint
    cv2.putText(out, 'Press 1-5 to save  |  [ ] = zoom  |  Q = quit',
                (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

    # Scale
    if display_scale != 1.0:
        new_w, new_h = int(w * display_scale), int(h * display_scale)
        out = cv2.resize(out, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return out


# ── ROS2 camera subscriber ────────────────────────────────────────────────────

class AstraNode(Node):
    def __init__(self):
        super().__init__('collect_data_node')
        self.bridge  = CvBridge()
        self.latest  = None
        self._lock   = threading.Lock()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, '/camera/color/image_raw', self._cb, qos)
        self.get_logger().info('Subscribed to /camera/color/image_raw')

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self.latest = frame
        except Exception as e:
            self.get_logger().error(str(e))

    def get_frame(self):
        with self._lock:
            return None if self.latest is None else self.latest.copy()


# ── Main ──────────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser()
parser.add_argument('--image', type=str, default=None,
                    help='Path to a static image file (skips ROS2 live feed)')
args = parser.parse_args()


def main_loop(get_frame_fn, is_live=True):
    display_scale = 0.6
    flash_msg     = ''
    flash_until   = 0.0

    print('\n── collect_data ──────────────────────────────────────')
    print('  1=straight  2=left  3=right  4=intersection  5=dead_end')
    print('  [ = shrink window   ] = enlarge   Q = quit')
    print(f'  Saving to: {DATASET_DIR}')
    print('─────────────────────────────────────────────────────\n')

    while True:
        frame = get_frame_fn()

        if frame is None:
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for camera…',
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
            cv2.imshow('collect_data', placeholder)
            if cv2.waitKey(100) & 0xFF in (ord('q'), 27):
                break
            continue

        counts = {cls: count_images(cls) for cls in CLASSES.values()}
        vis    = draw_hud(frame, display_scale, counts)

        # Flash saved-message overlay
        if time.time() < flash_until:
            h_v, w_v = vis.shape[:2]
            cv2.putText(vis, flash_msg,
                        (w_v // 2 - 100, h_v // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow('collect_data', vis)
        key = cv2.waitKey(1 if is_live else 0) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key in CLASSES:
            cls_name = CLASSES[key]
            path = save_image(frame, cls_name)
            flash_msg   = f'Saved → {cls_name} ({counts[cls_name] + 1})'
            flash_until = time.time() + 0.8
            print(f'  ✓  {path}')
        elif key == ord('['):
            display_scale = max(0.2, display_scale - 0.05)
        elif key == ord(']'):
            display_scale = min(2.0, display_scale + 0.05)

    cv2.destroyAllWindows()

    print('\n── Final counts ──────────────────────────────────────')
    total = 0
    for cls in CLASSES.values():
        n = count_images(cls)
        total += n
        bar = '█' * (n // 5)
        status = '✓' if n >= 80 else '✗ need more'
        print(f'  {cls:<14} {n:>4}  {bar}  {status}')
    print(f'  {"TOTAL":<14} {total:>4}')
    print('─────────────────────────────────────────────────────')
    print('  Next step: python3 train.py')


if args.image:
    # Static image mode — useful for manually adding a single image
    img = cv2.imread(args.image)
    if img is None:
        print(f'Cannot read {args.image}')
        raise SystemExit(1)
    main_loop(lambda: img, is_live=False)

elif _ROS2_OK:
    rclpy.init()
    node        = AstraNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    main_loop(node.get_frame, is_live=True)
    node.destroy_node()
    rclpy.shutdown()

else:
    print('ERROR: ROS2 not available and no --image flag given.')
    print('  Either start the Astra driver and source setup.bash,')
    print('  or pass --image /path/to/photo.jpg')
    raise SystemExit(1)

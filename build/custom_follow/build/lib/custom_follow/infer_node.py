#!/usr/bin/env python3
"""
infer_node.py  –  CNN lane inference + navigation (ROS2 node)
=============================================================
Loads the trained MobileNetV2 model (model/lane_model.pt) and runs
inference on every frame from the Astra camera.  Publishes /cmd_vel
Twist messages using the same turn-delay state machine as kayro.py.

State machine
-------------
  DRIVE_STRAIGHT  →  no turn detected, keep going forward
  TURN_PENDING    →  turn detected but lanes still visible, buffer direction
  EXECUTING_TURN  →  at the junction (no lanes), execute buffered turn
  DEAD_END_SPIN   →  dead end confirmed, spin 360°

Usage
-----
  # Build the package first:
  cd /home/chetan-satpute/lane_following
  colcon build --packages-select custom_follow
  source install/setup.bash

  # Run (model must exist at src/custom_follow/model/lane_model.pt):
  ros2 run custom_follow infer_node

  # Override parameters:
  ros2 run custom_follow infer_node --ros-args \
      -p linear_speed:=0.18 \
      -p angular_speed:=0.6 \
      -p spin_frames:=240 \
      -p dead_end_confirm:=10
"""

import collections
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import torch
import torch.nn as nn
from torchvision import models, transforms
from torchvision.models import MobileNet_V2_Weights
from PIL import Image as PILImage

# ── Paths ─────────────────────────────────────────────────────────────────────
# Search order:
#   1. ROS2 package share directory  (installed via colcon)
#   2. Source tree next to setup.py   (during development)
#   3. HOME fallback
def _find_model_default():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('custom_follow')
        candidates.append(os.path.join(share, 'model', 'lane_model.pt'))
    except Exception:
        pass
    # Source-tree location
    _src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', '..', 'src', 'custom_follow', 'model', 'lane_model.pt')
    candidates.append(os.path.normpath(_src))
    candidates.append(os.path.expanduser('~/lane_following/src/custom_follow/model/lane_model.pt'))
    for p in candidates:
        if os.path.exists(p):
            return p
    return candidates[0]  # return first so error message is useful

_DEFAULT_MODEL_PATH = _find_model_default()

# ── Inference preprocessing ───────────────────────────────────────────────────
_INFER_TF = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std =[0.229, 0.224, 0.225]),
])


def load_model(model_path):
    checkpoint = torch.load(model_path, map_location='cpu')
    classes    = checkpoint['classes']
    img_size   = checkpoint.get('img_size', 224)

    model = models.mobilenet_v2(weights=None)
    model.classifier = nn.Sequential(
        nn.Dropout(0.2),
        nn.Linear(model.classifier[1].in_features, len(classes)),
    )
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    return model, classes


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class InferNode(Node):

    # Navigation states
    DRIVE_STRAIGHT  = 'DRIVE_STRAIGHT'
    TURN_PENDING    = 'TURN_PENDING'
    EXECUTING_TURN  = 'EXECUTING_TURN'
    DEAD_END_SPIN   = 'DEAD_END_SPIN'

    def __init__(self):
        super().__init__('lane_infer_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',      0.18)
        self.declare_parameter('angular_speed',     0.60)
        self.declare_parameter('spin_frames',       240)
        self.declare_parameter('dead_end_confirm',  10)
        self.declare_parameter('show_debug',        True)
        self.declare_parameter('model_path',        _DEFAULT_MODEL_PATH)
        # Smoothing: only accept a label whose confidence >= this threshold
        self.declare_parameter('confidence_threshold', 0.65)
        # Smoothing: majority vote over this many recent frames
        self.declare_parameter('smooth_window',      7)

        p = self.get_parameter
        self._lin_spd      = p('linear_speed').value
        self._ang_spd      = p('angular_speed').value
        self._spin_total   = p('spin_frames').value
        self._de_thresh    = p('dead_end_confirm').value
        self._show_debug   = p('show_debug').value
        self._conf_thresh  = p('confidence_threshold').value
        self._smooth_win   = p('smooth_window').value

        # ── Load model ───────────────────────────────────────────────────────
        model_path = os.path.abspath(p('model_path').value)
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Model not found at {model_path}\n'
                f'  Train first:  python3 src/custom_follow/train.py\n'
                f'  Or specify:   --ros-args -p model_path:=/full/path/to/lane_model.pt')
            raise RuntimeError('Model file missing')

        self._model, self._classes = load_model(model_path)
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._model  = self._model.to(self._device)
        self.get_logger().info(f'Model loaded: classes={self._classes}  device={self._device}')

        # ── Navigation state machine ─────────────────────────────────────────
        self._nav_state        = self.DRIVE_STRAIGHT
        self._pending_turn     = None   # 'left' or 'right'
        self._executing_turn   = None
        self._de_count         = 0
        self._spin_remaining   = 0
        self._last_label       = None   # track label changes for WASD output
        # Rolling window for majority-vote smoothing
        self._label_buf        = collections.deque(maxlen=self._smooth_win)

        # ── ROS2 pub / sub ───────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._image_cb, sensor_qos)
        self.get_logger().info('Subscribed to /camera/color/image_raw')
        self.get_logger().info('Publishing to /cmd_vel')

    # ── Camera callback ───────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        with self._frame_lock:
            self._latest_frame = frame

        label, confidence = self._classify(frame)

        # ── Smoothing: confidence gate + majority vote ────────────────────
        if confidence >= self._conf_thresh:
            self._label_buf.append(label)
        # Use the majority label in the rolling window as the effective label
        if self._label_buf:
            smooth_label = collections.Counter(self._label_buf).most_common(1)[0][0]
        else:
            smooth_label = label

        twist = self._navigate(smooth_label)
        self._cmd_pub.publish(twist)

        if smooth_label != self._last_label:
            self._last_label = smooth_label
            _LABEL_KEY = {
                'straight':           'w',
                'left':               'a',
                'left_lane_endings':  'a',
                'right':              'd',
                'right_lane_endings': 'd',
                'dead_end':           's',
            }
            key = _LABEL_KEY.get(smooth_label)
            if key:
                print(key, flush=True)

        if self._show_debug:
            self._show(frame, smooth_label, confidence)

    # ── CNN inference ─────────────────────────────────────────────────────────

    def _classify(self, bgr_frame):
        """Returns (label_str, confidence_float)."""
        rgb    = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        pil    = PILImage.fromarray(rgb)
        tensor = _INFER_TF(pil).unsqueeze(0).to(self._device)

        with torch.no_grad():
            logits = self._model(tensor)
            probs  = torch.softmax(logits, dim=1)[0]
            idx    = probs.argmax().item()

        return self._classes[idx], probs[idx].item()

    # ── State machine ─────────────────────────────────────────────────────────

    def _navigate(self, label):
        twist = Twist()

        # ── Dead-end spin (highest priority) ─────────────────────────────────
        if self._nav_state == self.DEAD_END_SPIN:
            if self._spin_remaining > 0:
                twist.linear.x  = 0.0
                twist.angular.z = self._ang_spd
                self._spin_remaining -= 1
                return twist
            else:
                self._nav_state = self.DRIVE_STRAIGHT
                self.get_logger().info('NAV: spin complete → DRIVE_STRAIGHT')

        # ── Dead-end detection ────────────────────────────────────────────────
        if label == 'dead_end':
            self._de_count += 1
            if self._de_count >= self._de_thresh:
                self._de_count       = 0
                self._nav_state      = self.DEAD_END_SPIN
                self._spin_remaining = self._spin_total
                self.get_logger().warn('NAV: DEAD END confirmed → spinning 360°')
                return twist   # stop this frame, spin starts next frame
        else:
            self._de_count = 0

        # ── Turn-delay state machine ──────────────────────────────────────────
        if self._nav_state == self.DRIVE_STRAIGHT:
            if label in ('left', 'right'):
                self._pending_turn = label
                self._nav_state    = self.TURN_PENDING
                self.get_logger().info(f'NAV: {label} detected → TURN_PENDING')
            # keep driving straight regardless
            twist.linear.x  = self._lin_spd
            twist.angular.z = 0.0

        elif self._nav_state == self.TURN_PENDING:
            if label in ('straight', 'intersection'):
                # Lanes disappeared = we are AT the junction → execute turn
                self._executing_turn = self._pending_turn
                self._nav_state      = self.EXECUTING_TURN
                self.get_logger().info(
                    f'NAV: lanes gone → EXECUTING_TURN {self._executing_turn}')
            elif label in ('left', 'right'):
                # Still approaching — update direction if it changed
                self._pending_turn = label
            # Still go straight while pending
            twist.linear.x  = self._lin_spd
            twist.angular.z = 0.0

        elif self._nav_state == self.EXECUTING_TURN:
            if label == 'straight':
                # Turn complete
                self._nav_state      = self.DRIVE_STRAIGHT
                self._executing_turn = None
                self.get_logger().info('NAV: turn complete → DRIVE_STRAIGHT')
                twist.linear.x  = self._lin_spd
                twist.angular.z = 0.0
            else:
                # Keep turning
                ang = self._ang_spd if self._executing_turn == 'left' else -self._ang_spd
                twist.linear.x  = self._lin_spd * 0.5
                twist.angular.z = ang

        return twist

    # ── Debug overlay ─────────────────────────────────────────────────────────

    def _show(self, frame, label, confidence):
        vis = frame.copy()
        h, w = vis.shape[:2]

        state_colour = {
            self.DRIVE_STRAIGHT: (0, 255,   0),
            self.TURN_PENDING:   (0, 165, 255),
            self.EXECUTING_TURN: (0,   0, 255),
            self.DEAD_END_SPIN:  (255, 0,   0),
        }
        col = state_colour.get(self._nav_state, (200, 200, 200))

        cv2.rectangle(vis, (0, 0), (w, 50), (30, 30, 30), -1)
        cv2.putText(vis,
                    f'{label} ({confidence:.0%})   state: {self._nav_state}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)

        # Scale down for display
        disp = cv2.resize(vis, (int(w * 0.6), int(h * 0.6)))
        cv2.imshow('Lane Inference', disp)
        cv2.waitKey(1)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = InferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._cmd_pub.publish(stop)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

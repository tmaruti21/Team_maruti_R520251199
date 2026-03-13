#!/usr/bin/env python3
"""
infer_node.py  –  CNN lane inference + navigation (ROS2 node)
=============================================================
Loads the trained MobileNetV2 model (model/lane_model.pt) and runs
inference on every frame from the Astra camera.  Publishes /cmd_vel
Twist messages using a direct policy-based controller.

Navigation policy
-----------------
    straight                  → go forward
    left_lane_ending          → turn left only if confidence in [90%, 100%]
    right_lane_ending         → turn right only if confidence in [90%, 100%]
    dead_end                  → stop (no reverse for now)
    left / right              → soft turn toward indicated side
    intersection / unknown    → default forward

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
      -p turn_linear_speed:=0.0 \
      -p angular_speed:=0.6 \
      -p lane_ending_conf_min:=0.90 \
      -p lane_ending_conf_max:=1.00
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

    # Debug/navigation modes
    NAV_IDLE           = 'IDLE'
    NAV_STRAIGHT       = 'STRAIGHT'
    NAV_TURN_LEFT      = 'TURN_LEFT'
    NAV_TURN_RIGHT     = 'TURN_RIGHT'
    NAV_DEAD_END_STOP  = 'DEAD_END_STOP'
    NAV_INTERSECTION   = 'INTERSECTION'

    def __init__(self):
        super().__init__('lane_infer_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',      0.14)
        self.declare_parameter('turn_linear_speed', 0.0)
        self.declare_parameter('angular_speed',     0.60)
        # Kept for compatibility with previous launch commands.
        self.declare_parameter('reverse_speed',     0.12)
        self.declare_parameter('reverse_frames',    18)
        self.declare_parameter('min_turn_frames',   10)
        self.declare_parameter('max_turn_frames',   80)
        self.declare_parameter('dead_end_confirm',  10)
        self.declare_parameter('dead_end_rearm_frames', 12)
        self.declare_parameter('lane_ending_conf_min', 0.90)
        self.declare_parameter('lane_ending_conf_max', 1.00)
        self.declare_parameter('soft_turn_scale',   0.60)
        self.declare_parameter('show_debug',        True)
        self.declare_parameter('model_path',        _DEFAULT_MODEL_PATH)
        # Smoothing: only accept a label whose confidence >= this threshold
        self.declare_parameter('confidence_threshold', 0.65)
        # Smoothing: majority vote over this many recent frames
        self.declare_parameter('smooth_window',      7)

        p = self.get_parameter
        self._lin_spd      = p('linear_speed').value
        self._turn_lin_spd = p('turn_linear_speed').value
        self._ang_spd      = abs(p('angular_speed').value)
        self._rev_spd      = abs(p('reverse_speed').value)
        self._reverse_total = int(p('reverse_frames').value)
        self._min_turn_frames = max(1, int(p('min_turn_frames').value))
        self._max_turn_frames = max(self._min_turn_frames, int(p('max_turn_frames').value))
        self._de_thresh    = p('dead_end_confirm').value
        self._de_rearm_frames = max(1, int(p('dead_end_rearm_frames').value))
        self._lane_end_conf_min = float(p('lane_ending_conf_min').value)
        self._lane_end_conf_max = float(p('lane_ending_conf_max').value)
        self._soft_turn_scale = max(0.0, min(1.0, float(p('soft_turn_scale').value)))
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
        self._classes = [self._normalise_label(c) for c in self._classes]
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._model  = self._model.to(self._device)
        self.get_logger().info(f'Model loaded: classes={self._classes}  device={self._device}')

        # ── Navigation status ───────────────────────────────────────────────
        self._nav_state        = self.NAV_IDLE
        self._last_label       = None   # track label changes for WASD output
        self._last_nav_message = None
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

        label, confidence, scores = self._classify(frame)
        label = self._normalise_label(label)

        # ── Smoothing: confidence gate + majority vote ────────────────────
        if confidence >= self._conf_thresh:
            self._label_buf.append(label)
        # Use the majority label in the rolling window as the effective label
        if self._label_buf:
            smooth_label = collections.Counter(self._label_buf).most_common(1)[0][0]
        else:
            smooth_label = label

        smooth_conf = self._class_confidence(smooth_label, scores)

        twist = self._navigate(smooth_label, smooth_conf)
        self._cmd_pub.publish(twist)

        if smooth_label != self._last_label:
            self._last_label = smooth_label
            _LABEL_KEY = {
                'straight':           'w',
                'left':               'a',
                'left_lane_ending':   'a',
                'left_lane_endings':  'a',
                'right':              'd',
                'right_lane_ending':  'd',
                'right_lane_endings': 'd',
                'dead_end':           's',
            }
            key = _LABEL_KEY.get(smooth_label)
            if key:
                print(key, flush=True)

        if self._show_debug:
            self._show(frame, smooth_label, smooth_conf)

    # ── CNN inference ─────────────────────────────────────────────────────────

    def _classify(self, bgr_frame):
        """Returns (label_str, confidence_float, class_score_dict)."""
        rgb    = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        pil    = PILImage.fromarray(rgb)
        tensor = _INFER_TF(pil).unsqueeze(0).to(self._device)

        with torch.no_grad():
            logits = self._model(tensor)
            probs  = torch.softmax(logits, dim=1)[0]
            idx    = probs.argmax().item()

        score_map = {
            self._normalise_label(self._classes[i]): float(probs[i].item())
            for i in range(len(self._classes))
        }

        return self._normalise_label(self._classes[idx]), probs[idx].item(), score_map

    def _normalise_label(self, label):
        if not isinstance(label, str):
            return label
        label = label.strip().lower()
        aliases = {
            'left_lane_endings': 'left_lane_ending',
            'right_lane_endings': 'right_lane_ending',
        }
        return aliases.get(label, label)

    def _class_confidence(self, label, score_map):
        key = self._normalise_label(label)
        return float(score_map.get(key, 0.0))

    def _set_turn_twist(self, twist, turn_left):
        ang = self._ang_spd if turn_left else -self._ang_spd
        twist.linear.x = self._turn_lin_spd
        twist.angular.z = ang
        return twist

    def _set_mode(self, mode, message):
        if mode != self._nav_state or message != self._last_nav_message:
            self._nav_state = mode
            self._last_nav_message = message
            self.get_logger().info(message)

    # ── Policy controller ────────────────────────────────────────────────────

    def _navigate(self, label, confidence):
        label = self._normalise_label(label)
        twist = Twist()
        conf = max(0.0, min(1.0, float(confidence)))

        # 1) Dead end: stop immediately (user requested stop-only behavior).
        if label == 'dead_end':
            self._set_mode(self.NAV_DEAD_END_STOP, 'NAV: dead_end → STOP')
            return twist

        # 2) Lane-ending turns with strict confidence gate [90%, 100%].
        if label == 'left_lane_ending' and self._lane_end_conf_min <= conf <= self._lane_end_conf_max:
            self._set_mode(self.NAV_TURN_LEFT, f'NAV: left_lane_ending ({conf:.0%}) → TURN_LEFT')
            return self._set_turn_twist(twist, turn_left=True)

        if label == 'right_lane_ending' and self._lane_end_conf_min <= conf <= self._lane_end_conf_max:
            self._set_mode(self.NAV_TURN_RIGHT, f'NAV: right_lane_ending ({conf:.0%}) → TURN_RIGHT')
            return self._set_turn_twist(twist, turn_left=False)

        # 3) Straight state.
        if label == 'straight':
            twist.linear.x = self._lin_spd
            twist.angular.z = 0.0
            self._set_mode(self.NAV_STRAIGHT, 'NAV: straight → FORWARD')
            return twist

        # 4) Soft turning for left/right labels.
        if label == 'left':
            twist.linear.x = self._lin_spd * self._soft_turn_scale
            twist.angular.z = self._ang_spd * self._soft_turn_scale
            self._set_mode(self.NAV_TURN_LEFT, f'NAV: left ({conf:.0%}) → SOFT TURN_LEFT')
            return twist

        if label == 'right':
            twist.linear.x = self._lin_spd * self._soft_turn_scale
            twist.angular.z = -self._ang_spd * self._soft_turn_scale
            self._set_mode(self.NAV_TURN_RIGHT, f'NAV: right ({conf:.0%}) → SOFT TURN_RIGHT')
            return twist

        # 5) Intersection and unknown fallback.
        if label == 'intersection':
            twist.linear.x = self._lin_spd
            twist.angular.z = 0.0
            self._set_mode(self.NAV_INTERSECTION, 'NAV: intersection → FORWARD')
            return twist

        twist.linear.x = self._lin_spd
        twist.angular.z = 0.0
        self._set_mode(self.NAV_IDLE, f'NAV: {label} ({conf:.0%}) → default FORWARD')

        return twist

    # ── Debug overlay ─────────────────────────────────────────────────────────

    def _show(self, frame, label, confidence):
        vis = frame.copy()
        h, w = vis.shape[:2]

        state_colour = {
            self.NAV_IDLE:          (180, 180, 180),
            self.NAV_STRAIGHT:      (0, 255, 0),
            self.NAV_TURN_LEFT:     (0, 165, 255),
            self.NAV_TURN_RIGHT:    (0, 120, 255),
            self.NAV_DEAD_END_STOP: (0, 0, 255),
            self.NAV_INTERSECTION:  (255, 255, 0),
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

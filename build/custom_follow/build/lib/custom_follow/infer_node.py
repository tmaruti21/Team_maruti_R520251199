#!/usr/bin/env python3
"""
infer_node.py  –  Lane inference + navigation (ROS2 node)
=========================================================
Supports two model formats transparently:

  v2_seg  (new)  –  LaneSegNet (EfficientNet-B4 + FPN decoder)
                    trained with train_v2.py  →  model/lane_model_v2.pt
                    Provides BOTH classification logits AND a lane
                    segmentation mask visualised in the debug window.

  v1      (legacy)  –  MobileNetV2 classification head
                        trained with train.py  →  model/lane_model.pt

The node auto-detects the format from the checkpoint's 'model_type' key.

Navigation policy
-----------------
    straight / left / right   → FORWARD command
    left_lane_ending          → LEFT command  (only if confidence >= 90%)
    right_lane_ending         → RIGHT command (only if confidence >= 90%)
    dead_end / dead_zone      → STOP command, then reverse maneuver
    all other states          → STOP command

Usage
-----
  # Train the new model first:
  cd src/custom_follow && python3 train_v2.py

  # Build and source:
  cd /home/chetan-satpute/lane_following
  colcon build --packages-select custom_follow && source install/setup.bash

  # Run with v2 model:
  ros2 run custom_follow infer_node \\
      --ros-args -p model_path:=$(pwd)/src/custom_follow/model/lane_model_v2.pt

  # Run with legacy v1 model (unchanged behaviour):
  ros2 run custom_follow infer_node

  # Additional tuning parameters:
  ros2 run custom_follow infer_node --ros-args \\
      -p linear_speed:=0.18 \\
      -p angular_speed:=0.60 \\
      -p lane_ending_conf_min:=0.88 \\
      -p seg_alpha:=0.40          # segmentation overlay opacity (v2 only)
      -p use_tta:=true            # test-time augmentation for robustness
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
import torch.nn.functional as F
from torchvision import models, transforms
from PIL import Image as PILImage

# ── Model architecture (v2_seg) ───────────────────────────────────────────────
# Imported lazily inside load_model() so the node starts even if
# torchvision.models.feature_extraction is unavailable (old envs).
_LaneSegNet = None

def _get_lane_seg_net():
    global _LaneSegNet
    if _LaneSegNet is None:
        try:
            from custom_follow.model_arch import LaneSegNet as _Cls
            _LaneSegNet = _Cls
        except Exception as exc:
            raise RuntimeError(
                f'Cannot import LaneSegNet from custom_follow.model_arch: {exc}\n'
                f'Make sure model_arch.py is present and the package is built.') from exc
    return _LaneSegNet

# ── Paths ─────────────────────────────────────────────────────────────────────
# Preference order:
#   1. Installed share directory (colcon install)
#   2. lane_model_v2.pt in source tree  (new EfficientNet model)
#   3. lane_model.pt in source tree     (legacy MobileNetV2)
#   4. HOME fallback
def _find_model_default():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('custom_follow')
        # Prefer v2 model if present
        candidates.append(os.path.join(share, 'model', 'lane_model_v2.pt'))
        candidates.append(os.path.join(share, 'model', 'lane_model.pt'))
    except Exception:
        pass
    _base = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', 'src', 'custom_follow', 'model'))
    candidates.append(os.path.join(_base, 'lane_model_v2.pt'))
    candidates.append(os.path.join(_base, 'lane_model.pt'))
    candidates.append(os.path.expanduser(
        '~/lane_following/src/custom_follow/model/lane_model_v2.pt'))
    candidates.append(os.path.expanduser(
        '~/lane_following/src/custom_follow/model/lane_model.pt'))
    for p in candidates:
        if os.path.exists(p):
            return p
    return candidates[0]   # will raise a clear error later

_DEFAULT_MODEL_PATH = _find_model_default()

# ── Preprocessing transforms ──────────────────────────────────────────────────
_NORM_MEAN = [0.485, 0.456, 0.406]
_NORM_STD  = [0.229, 0.224, 0.225]

def _make_infer_tf(img_size: int):
    return transforms.Compose([
        transforms.Resize((img_size, img_size), antialias=True),
        transforms.ToTensor(),
        transforms.Normalize(mean=_NORM_MEAN, std=_NORM_STD),
    ])

# Test-time augmentation transforms (mild spatial + colour perturbations)
def _make_tta_tfs(img_size: int):
    """Return a list of PIL→tensor transforms for TTA ensemble."""
    base = _make_infer_tf(img_size)
    bright_up   = transforms.Compose([
        transforms.Resize((img_size, img_size), antialias=True),
        transforms.ColorJitter(brightness=(1.2, 1.2)),
        transforms.ToTensor(),
        transforms.Normalize(_NORM_MEAN, _NORM_STD),
    ])
    bright_down = transforms.Compose([
        transforms.Resize((img_size, img_size), antialias=True),
        transforms.ColorJitter(brightness=(0.8, 0.8)),
        transforms.ToTensor(),
        transforms.Normalize(_NORM_MEAN, _NORM_STD),
    ])
    return [base, bright_up, bright_down]


# ── Model loading ─────────────────────────────────────────────────────────────

def load_model(model_path: str):
    """
    Load a checkpoint and return (model, classes, img_size, model_type).

    Handles both:
      • 'v2_seg'  – LaneSegNet (EfficientNet-B4 + segmentation decoder)
      • legacy    – MobileNetV2 classification head
    """
    checkpoint = torch.load(model_path, map_location='cpu',
                            weights_only=False)
    classes    = checkpoint['classes']
    img_size   = checkpoint.get('img_size', 224)
    mtype      = checkpoint.get('model_type', 'v1')

    if mtype == 'v2_seg':
        LaneSegNet = _get_lane_seg_net()
        model = LaneSegNet(num_classes=len(classes), pretrained=False)
        model.load_state_dict(checkpoint['model_state_dict'])
    else:
        # Legacy MobileNetV2 checkpoint (train.py output)
        model = models.mobilenet_v2(weights=None)
        model.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(model.classifier[1].in_features, len(classes)),
        )
        model.load_state_dict(checkpoint['model_state_dict'])

    model.eval()
    return model, classes, img_size, mtype


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class InferNode(Node):

    # Debug/navigation modes
    NAV_IDLE           = 'IDLE'
    NAV_STRAIGHT       = 'STRAIGHT'
    NAV_TURN_LEFT      = 'TURN_LEFT'
    NAV_TURN_RIGHT     = 'TURN_RIGHT'
    NAV_DEAD_END_STOP  = 'DEAD_END_STOP'
    NAV_REVERSE        = 'REVERSE'
    NAV_INTERSECTION   = 'INTERSECTION'

    # Command log values (mapped to WASD keys)
    CMD_FORWARD = 'FORWARD'
    CMD_LEFT    = 'LEFT'
    CMD_RIGHT   = 'RIGHT'
    CMD_STOP    = 'STOP'

    def __init__(self):
        super().__init__('lane_infer_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',      0.14)
        self.declare_parameter('turn_linear_speed', 0.0)
        self.declare_parameter('angular_speed',     0.60)
        # Kept for compatibility with previous launch commands.
        self.declare_parameter('reverse_speed',     0.12)
        self.declare_parameter('reverse_frames',    18)
        self.declare_parameter('stop_before_reverse_frames', 2)
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
        # v2_seg: segmentation overlay opacity (0 = off, 1 = full)
        self.declare_parameter('seg_alpha',          0.35)
        # TTA: average over 3 brightness-varied passes (slower but more stable)
        self.declare_parameter('use_tta',            False)

        p = self.get_parameter
        self._lin_spd      = p('linear_speed').value
        self._turn_lin_spd = p('turn_linear_speed').value
        self._ang_spd      = abs(p('angular_speed').value)
        self._rev_spd      = abs(p('reverse_speed').value)
        self._reverse_total = int(p('reverse_frames').value)
        self._stop_before_reverse_frames = max(1, int(p('stop_before_reverse_frames').value))
        self._min_turn_frames = max(1, int(p('min_turn_frames').value))
        self._max_turn_frames = max(self._min_turn_frames, int(p('max_turn_frames').value))
        self._de_thresh    = max(1, int(p('dead_end_confirm').value))
        self._de_rearm_frames = max(1, int(p('dead_end_rearm_frames').value))
        self._lane_end_conf_min = float(p('lane_ending_conf_min').value)
        self._lane_end_conf_max = float(p('lane_ending_conf_max').value)
        self._soft_turn_scale = max(0.0, min(1.0, float(p('soft_turn_scale').value)))
        self._show_debug   = p('show_debug').value
        self._conf_thresh  = p('confidence_threshold').value
        self._smooth_win   = p('smooth_window').value
        self._seg_alpha    = max(0.0, min(1.0, float(p('seg_alpha').value)))
        self._use_tta      = bool(p('use_tta').value)

        # ── Load model ───────────────────────────────────────────────────────
        model_path = os.path.abspath(p('model_path').value)
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Model not found at {model_path}\n'
                f'  Train new model:  cd src/custom_follow && python3 train_v2.py\n'
                f'  Or legacy model:  python3 train.py\n'
                f'  Or specify:       --ros-args -p model_path:=/full/path/to/model.pt')
            raise RuntimeError('Model file missing')

        self._model, self._classes, self._img_size, self._model_type = \
            load_model(model_path)
        self._classes = [self._normalise_label(c) for c in self._classes]
        self._device  = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._model   = self._model.to(self._device)

        # Build preprocessing transform(s)
        self._infer_tf  = _make_infer_tf(self._img_size)
        self._tta_tfs   = _make_tta_tfs(self._img_size) if self._use_tta else None
        self._latest_seg_mask: np.ndarray | None = None   # HxW float32 in [0,1]

        self.get_logger().info(
            f'Model loaded: type={self._model_type}  '
            f'classes={self._classes}  '
            f'img_size={self._img_size}  device={self._device}'
            + ('  TTA=ON' if self._use_tta else ''))

        # ── Navigation status ───────────────────────────────────────────────
        self._nav_state        = self.NAV_IDLE
        self._last_cmd_name    = None
        self._last_cmd_detail  = None
        self._last_nav_message = None
        # Rolling window for majority-vote smoothing
        self._label_buf        = collections.deque(maxlen=self._smooth_win)

        # Dead-zone stop-then-reverse state
        self._dead_zone_hits = 0
        self._dead_zone_rearm_left = 0
        self._stop_frames_left = 0
        self._reverse_frames_left = 0

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

        twist, cmd_name, cmd_detail = self._navigate(smooth_label, smooth_conf)
        self._cmd_pub.publish(twist)
        self._emit_command_log(cmd_name, cmd_detail, smooth_label, smooth_conf)

        if self._show_debug:
            self._show(frame, smooth_label, smooth_conf)

    # ── CNN inference ─────────────────────────────────────────────────────────

    def _classify(self, bgr_frame):
        """
        Run inference on a BGR numpy frame.
        Returns (label_str, confidence_float, class_score_dict).

        For v2_seg models, also stores the lane segmentation mask in
        self._latest_seg_mask (H×W float32 in [0, 1]).
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)

        if self._use_tta and self._tta_tfs is not None:
            # Average probabilities over multiple preprocessing variants
            all_probs = []
            for tf in self._tta_tfs:
                t = tf(pil).unsqueeze(0).to(self._device)
                with torch.no_grad():
                    out = self._model(t)
                    logits = out[0] if isinstance(out, (tuple, list)) else out
                    all_probs.append(torch.softmax(logits, dim=1)[0])
            probs = torch.stack(all_probs).mean(dim=0)
            # For seg mask, use the base transform pass
            if self._model_type == 'v2_seg':
                t = self._infer_tf(pil).unsqueeze(0).to(self._device)
                with torch.no_grad():
                    _, seg = self._model(t)
                self._latest_seg_mask = seg[0, 0].cpu().numpy()
        else:
            tensor = self._infer_tf(pil).unsqueeze(0).to(self._device)
            with torch.no_grad():
                out = self._model(tensor)
                if isinstance(out, (tuple, list)):
                    logits, seg = out[0], out[1]
                    self._latest_seg_mask = seg[0, 0].cpu().numpy()
                else:
                    logits = out
                    self._latest_seg_mask = None
            probs = torch.softmax(logits, dim=1)[0]

        idx       = probs.argmax().item()
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

    def _emit_command_log(self, cmd_name, cmd_detail, label, confidence):
        key_map = {
            self.CMD_FORWARD: 'w',
            self.CMD_LEFT: 'a',
            self.CMD_RIGHT: 'd',
            self.CMD_STOP: 's',
        }
        key = key_map.get(cmd_name, '-')
        # Log only on command/detail transitions to keep logs readable.
        if cmd_name != self._last_cmd_name or cmd_detail != self._last_cmd_detail:
            self.get_logger().info(
                f'CMD: {cmd_name} ({key}) | label={label} conf={confidence:.0%} | {cmd_detail}')
            if key != '-':
                print(key, flush=True)
            self._last_cmd_name = cmd_name
            self._last_cmd_detail = cmd_detail

    # ── Policy controller ────────────────────────────────────────────────────

    def _navigate(self, label, confidence):
        label = self._normalise_label(label)
        twist = Twist()
        conf = max(0.0, min(1.0, float(confidence)))

        # Decrement dead-zone rearm counter every frame.
        if self._dead_zone_rearm_left > 0:
            self._dead_zone_rearm_left -= 1

        # 1) Active dead-zone maneuver: STOP phase, then REVERSE phase.
        if self._stop_frames_left > 0:
            self._stop_frames_left -= 1
            self._set_mode(self.NAV_DEAD_END_STOP, 'NAV: dead_zone → STOP (pre-reverse)')
            return twist, self.CMD_STOP, 'dead_zone stop phase'

        if self._reverse_frames_left > 0:
            self._reverse_frames_left -= 1
            twist.linear.x = -self._rev_spd
            twist.angular.z = 0.0
            self._set_mode(self.NAV_REVERSE, 'NAV: dead_zone → REVERSE')
            return twist, self.CMD_STOP, 'dead_zone reverse phase'

        # 2) Dead-zone trigger with confirmation threshold.
        is_dead_zone = label in {'dead_end', 'dead_zone'}
        if is_dead_zone:
            self._dead_zone_hits += 1
        else:
            self._dead_zone_hits = 0

        if is_dead_zone and self._dead_zone_rearm_left == 0 and self._dead_zone_hits >= self._de_thresh:
            self._dead_zone_hits = 0
            self._dead_zone_rearm_left = self._de_rearm_frames
            self._stop_frames_left = self._stop_before_reverse_frames
            self._reverse_frames_left = self._reverse_total
            self._set_mode(self.NAV_DEAD_END_STOP, 'NAV: dead_zone confirmed → STOP then REVERSE')
            return twist, self.CMD_STOP, 'dead_zone confirmed'

        # 3) LEFT command only when left_lane_ending confidence >= 90%.
        if label == 'left_lane_ending' and self._lane_end_conf_min <= conf <= self._lane_end_conf_max:
            self._set_mode(self.NAV_TURN_LEFT, f'NAV: left_lane_ending ({conf:.0%}) → LEFT')
            return self._set_turn_twist(twist, turn_left=True), self.CMD_LEFT, 'left_lane_ending high confidence'

        # 4) RIGHT command only when right_lane_ending confidence >= 90%.
        if label == 'right_lane_ending' and self._lane_end_conf_min <= conf <= self._lane_end_conf_max:
            self._set_mode(self.NAV_TURN_RIGHT, f'NAV: right_lane_ending ({conf:.0%}) → RIGHT')
            return self._set_turn_twist(twist, turn_left=False), self.CMD_RIGHT, 'right_lane_ending high confidence'

        # 5) FORWARD command only for straight/left/right states.
        if label in {'straight', 'left', 'right'}:
            twist.linear.x = self._lin_spd
            twist.angular.z = 0.0
            self._set_mode(self.NAV_STRAIGHT, f'NAV: {label} → FORWARD')
            return twist, self.CMD_FORWARD, f'{label} state'

        # 6) All other states: STOP.
        self._set_mode(self.NAV_IDLE, f'NAV: {label} ({conf:.0%}) → STOP (unmapped/low confidence)')
        return twist, self.CMD_STOP, 'unmapped or low-confidence state'

    # ── Debug overlay ─────────────────────────────────────────────────────────

    def _show(self, frame, label, confidence):
        vis = frame.copy()
        h, w = vis.shape[:2]

        # ── Segmentation overlay (v2_seg only) ───────────────────────────
        seg = self._latest_seg_mask
        if seg is not None and self._seg_alpha > 0.01:
            # Resize mask to frame resolution
            seg_resized = cv2.resize(seg, (w, h), interpolation=cv2.INTER_LINEAR)
            # Map to 0-255 uint8 and tint green
            seg_u8 = (np.clip(seg_resized, 0.0, 1.0) * 255).astype(np.uint8)
            overlay = np.zeros_like(vis)
            overlay[:, :, 1] = seg_u8    # green channel = lane mask

            # Hard edge tint on confident pixels (> 0.5)
            hard = (seg_resized > 0.50).astype(np.uint8)
            edge_overlay = np.zeros_like(vis)
            edge_overlay[:, :, 1] = (hard * 200).astype(np.uint8)
            edge_overlay[:, :, 2] = (hard * 80).astype(np.uint8)

            vis = cv2.addWeighted(vis, 1.0 - self._seg_alpha,
                                  overlay, self._seg_alpha, 0)
            vis = cv2.addWeighted(vis, 1.0,
                                  edge_overlay, self._seg_alpha * 0.5, 0)

        # ── Status bar ───────────────────────────────────────────────────
        state_colour = {
            self.NAV_IDLE:          (180, 180, 180),
            self.NAV_STRAIGHT:      (0, 255, 0),
            self.NAV_TURN_LEFT:     (0, 165, 255),
            self.NAV_TURN_RIGHT:    (0, 120, 255),
            self.NAV_DEAD_END_STOP: (0, 0, 255),
            self.NAV_REVERSE:       (255, 80, 80),
            self.NAV_INTERSECTION:  (255, 255, 0),
        }
        col = state_colour.get(self._nav_state, (200, 200, 200))

        cv2.rectangle(vis, (0, 0), (w, 54), (20, 20, 20), -1)
        model_tag = f'[{self._model_type}]' if self._model_type else ''
        cv2.putText(vis,
                    f'{label} ({confidence:.0%})   {self._nav_state} {model_tag}',
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)

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

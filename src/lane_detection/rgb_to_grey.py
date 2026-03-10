#!/usr/bin/env python3
"""
Lane Detection Pipeline Viewer  (Astra Pro Plus / ROS2)
========================================================
Shows every stage of the lane-detection pipeline in real time AND runs
the shape classifier live so you can tune parameters on the spot.

Grid layout (3 columns × 2 rows):
  ┌─────────────┬─────────────┬─────────────┐
  │ 1. Original │ 2. Grey     │ 3. Blur     │
  │    + ROI    │             │             │
  ├─────────────┼─────────────┼─────────────┤
  │ 4. Canny    │ 5. Masked   │ 6. Shape    │
  │    edges    │    binary   │    debug    │
  └─────────────┴─────────────┴─────────────┘

Panel 6 colour key:
  BLUE    – near-horizontal (wall / road edge)
  GREEN   – diagonal-left   (outer left  boundary)
  RED     – diagonal-right  (outer right boundary)
  MAGENTA – near-vertical left
  CYAN    – near-vertical right
  GREY    – excluded (too short or below outer-zone cutoff)

Controls
--------
  S        – save original + all pipeline images to ./lane_snapshots/
  Q        – quit
  +  / =   – increase outer-boundary zone  (include more of frame)
  -        – decrease outer-boundary zone  (far-field only)
  W / N    – widen / narrow ROI top
  H / L    – raise / lower ROI height
  P        – print current parameter values

Usage
-----
  source /home/chetan-satpute/lane_following/install/setup.bash
  python3 rgb_to_grey.py                    # Astra live feed
  python3 rgb_to_grey.py --image photo.jpg  # static image (no ROS needed)
"""

# import argparse
# import os
# import threading
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# # ── Tunable parameters (same defaults as kayro.py) ────────────────────────────
# params = dict(
#     kernel_size      = 9,     # Gaussian blur kernel (must be odd)
#     canny_low        = 60,
#     canny_high       = 150,
#     roi_top_width    = 0.20,  # fraction of half-frame-width at top of trapezoid
#     roi_height       = 0.55,  # fraction of frame height where ROI top sits
#     outer_zone_frac  = 0.75,  # lines with centre y > frac*H are excluded from shape detection
# )

# # ── Shape-debug colour map ─────────────────────────────────────────────────────
# COLOUR_MAP = {
#     'horiz':      (255,   0,   0),
#     'diag_left':  (  0, 255,   0),
#     'diag_right': (  0,   0, 255),
#     'vert_left':  (255,   0, 255),
#     'vert_right': (  0, 255, 255),
#     'excluded':   (100, 100, 100),
# }
# SHAPE_COLOUR = {
#     'STRAIGHT':     (200, 200, 200),
#     'TURN_LEFT':    (  0, 165, 255),
#     'TURN_RIGHT':   (  0, 165, 255),
#     'INTERSECTION': (  0, 255, 255),
#     'DEAD_END':     (  0,   0, 255),
# }


# # ── Pipeline helpers ───────────────────────────────────────────────────────────

# def build_roi_mask(shape, roi_top_width, roi_height):
#     h, w = shape[:2]
#     cx = w / 2
#     tl_x = int(cx - w * roi_top_width)
#     tr_x = int(cx + w * roi_top_width)
#     top_y = int(h * roi_height)
#     verts = np.array([[(0, h), (tl_x, top_y), (tr_x, top_y), (w, h)]], dtype=np.int32)
#     mask = np.zeros((h, w), dtype=np.uint8)
#     cv2.fillPoly(mask, verts, 255)
#     return mask, verts


# def pipeline(frame, p):
#     grey   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     blur   = cv2.GaussianBlur(grey, (p['kernel_size'], p['kernel_size']), 0)
#     edges  = cv2.Canny(blur, p['canny_low'], p['canny_high'])
#     mask, verts = build_roi_mask(frame.shape, p['roi_top_width'], p['roi_height'])
#     masked = cv2.bitwise_and(edges, mask)
#     lines  = cv2.HoughLinesP(masked, 2, np.pi/180, 45,
#                               np.array([]), minLineLength=40, maxLineGap=100)
#     return grey, blur, edges, masked, verts, lines


# def classify_road_shape(lines, imshape, outer_zone_y):
#     debug = {k: [] for k in COLOUR_MAP}
#     if lines is None or len(lines) == 0:
#         return 'STRAIGHT', debug
#     h, w = imshape[:2]
#     mid_x = w / 2.0
#     has_horiz = has_diag_left = has_diag_right = has_vert_left = has_vert_right = False
#     for line in lines:
#         for x1, y1, x2, y2 in line:
#             seg = (x1, y1, x2, y2)
#             if ((x2-x1)**2 + (y2-y1)**2)**0.5 < 25:
#                 debug['excluded'].append(seg); continue
#             if (y1+y2)/2.0 > outer_zone_y:
#                 debug['excluded'].append(seg); continue
#             dx = x2 - x1
#             s  = abs((y2-y1)/dx) if dx != 0 else 999.0
#             cx = (x1+x2)/2.0
#             if s < 0.36:
#                 has_horiz = True;         debug['horiz'].append(seg)
#             elif s > 2.75:
#                 if cx < mid_x: has_vert_left  = True; debug['vert_left'].append(seg)
#                 else:          has_vert_right = True; debug['vert_right'].append(seg)
#             else:
#                 if cx < mid_x: has_diag_left  = True; debug['diag_left'].append(seg)
#                 else:          has_diag_right = True; debug['diag_right'].append(seg)
#     if has_horiz and not has_diag_left and not has_diag_right:
#         return 'DEAD_END', debug
#     if has_horiz and (has_diag_left or has_diag_right):
#         return 'INTERSECTION', debug
#     if (has_horiz or has_vert_right) and not has_diag_right:
#         if has_diag_left or has_vert_left:
#             return 'TURN_RIGHT', debug
#     if (has_horiz or has_vert_left) and not has_diag_left:
#         if has_diag_right or has_vert_right:
#             return 'TURN_LEFT', debug
#     return 'STRAIGHT', debug


# def make_shape_panel(frame, lines, p):
#     """Render colour-coded Hough lines + shape result on a copy of the frame."""
#     out = frame.copy()
#     h, w = out.shape[:2]
#     outer_zone_y = h * p['outer_zone_frac']
#     road_shape, debug = classify_road_shape(lines, frame.shape, outer_zone_y)

#     for cat, segs in debug.items():
#         col = COLOUR_MAP[cat]
#         th  = 1 if cat == 'excluded' else 3
#         for (x1, y1, x2, y2) in segs:
#             cv2.line(out, (x1, y1), (x2, y2), col, th)

#     # Outer-zone cutoff line (dashed)
#     zone_yi = int(outer_zone_y)
#     for sx in range(0, w, 18):
#         cv2.line(out, (sx, zone_yi), (min(sx+10, w), zone_yi), (0, 220, 220), 2)
#     cv2.putText(out, f'zone={p["outer_zone_frac"]:.2f}',
#                 (8, zone_yi - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,220,220), 1)

#     # Mid vertical
#     cv2.line(out, (w//2, 0), (w//2, h), (160,160,0), 1)

#     # Legend
#     for i, (cat, col) in enumerate(COLOUR_MAP.items()):
#         y = 18 + i*20
#         cv2.rectangle(out, (8, y-11), (22, y+3), col, -1)
#         cv2.putText(out, f'{cat}({len(debug[cat])})',
#                     (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,255,255), 1)

#     # Shape result at bottom
#     sc = SHAPE_COLOUR.get(road_shape, (200,200,200))
#     cv2.putText(out, road_shape,
#                 (w//2 - 60, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, sc, 2)

#     return out, road_shape


# def annotate(img_bgr, label):
#     out = img_bgr.copy()
#     cv2.putText(out, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
#     return out


# def make_grid(frame, grey, blur, edges, masked, verts, lines, p):
#     """3×2 grid: original | grey | blur // canny | masked | shape-debug"""
#     def to_bgr(img):
#         return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if len(img.shape) == 2 else img

#     orig = frame.copy()
#     cv2.polylines(orig, verts, True, (0,255,255), 2)

#     shape_panel, road_shape = make_shape_panel(frame, lines, p)

#     c1 = annotate(orig,              '1. Original + ROI')
#     c2 = annotate(to_bgr(grey),      '2. Greyscale')
#     c3 = annotate(to_bgr(blur),      '3. Blur')
#     c4 = annotate(to_bgr(edges),     '4. Canny edges')
#     c5 = annotate(to_bgr(masked),    '5. Masked binary')
#     c6 = annotate(shape_panel,       '6. Shape debug')

#     top = np.hstack([c1, c2, c3])
#     bot = np.hstack([c4, c5, c6])
#     grid = np.vstack([top, bot])

#     h, w = grid.shape[:2]
#     cv2.putText(grid,
#                 f'Canny {p["canny_low"]}/{p["canny_high"]}  '
#                 f'ROI w={p["roi_top_width"]:.2f} h={p["roi_height"]:.2f}  '
#                 f'zone={p["outer_zone_frac"]:.2f}  '
#                 f'S=save Q=quit +/-=zone W/N=width H/L=height P=params',
#                 (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)
#     return grid


# # ── Keyboard handler ───────────────────────────────────────────────────────────

# def handle_key(key, frame, snap_count, save_dir):
#     """Returns updated snap_count, or -1 to quit."""
#     if key in (ord('q'), 27):
#         return -1
#     elif key == ord('s'):
#         grey, blur, edges, masked, _, lines = pipeline(frame, params)
#         orig_path = os.path.join(save_dir, f'snap_{snap_count:04d}_original.jpg')
#         cv2.imwrite(orig_path, frame)
#         print(f'Saved → {orig_path}  ← put this in straight/ left/ right/ etc.')
#         for name, img in [('grey', grey), ('blur', blur),
#                            ('edges', edges), ('masked', masked)]:
#             cv2.imwrite(os.path.join(save_dir, f'snap_{snap_count:04d}_{name}.png'), img)
#         print(f'       + grey / blur / edges / masked saved to {save_dir}/')
#         return snap_count + 1
#     elif key in (ord('+'), ord('=')):
#         params['outer_zone_frac'] = min(1.0,  params['outer_zone_frac'] + 0.02)
#         print(f"outer_zone_frac → {params['outer_zone_frac']:.2f}")
#     elif key == ord('-'):
#         params['outer_zone_frac'] = max(0.10, params['outer_zone_frac'] - 0.02)
#         print(f"outer_zone_frac → {params['outer_zone_frac']:.2f}")
#     elif key == ord('w'):
#         params['roi_top_width'] = min(0.45, params['roi_top_width'] + 0.02)
#     elif key == ord('n'):
#         params['roi_top_width'] = max(0.05, params['roi_top_width'] - 0.02)
#     elif key == ord('h'):
#         params['roi_height'] = max(0.20, params['roi_height'] - 0.02)
#     elif key == ord('l'):
#         params['roi_height'] = min(0.90, params['roi_height'] + 0.02)
#     elif key == ord('p'):
#         print('\n── Current parameters ──')
#         for k, v in params.items():
#             print(f'  {k:<18} = {v}')
#     return snap_count


# # ── ROS2 subscriber node ───────────────────────────────────────────────────────

# class AstraPipelineNode(Node):
#     def __init__(self):
#         super().__init__('lane_pipeline_viewer')
#         self.bridge = CvBridge()
#         self.latest = None
#         self._lock  = threading.Lock()
#         sensor_qos  = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1,
#         )
#         self.create_subscription(Image, '/camera/color/image_raw', self._cb, sensor_qos)
#         self.get_logger().info('Subscribed to /camera/color/image_raw')

#     def _cb(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             with self._lock:
#                 self.latest = frame
#         except Exception as e:
#             self.get_logger().error(str(e))

#     def get_frame(self):
#         with self._lock:
#             return None if self.latest is None else self.latest.copy()


# # ── Entry point ────────────────────────────────────────────────────────────────

# parser = argparse.ArgumentParser()
# parser.add_argument('--image', type=str, default=None,
#                     help='Path to a static image (skips ROS2 live feed)')
# args = parser.parse_args()

# save_dir   = 'lane_snapshots'
# os.makedirs(save_dir, exist_ok=True)
# snap_count = 0

# if args.image:
#     frame = cv2.imread(args.image)
#     if frame is None:
#         print(f'Cannot read {args.image}'); raise SystemExit(1)
#     print('Static image mode.  S=save  Q=quit  +/-=zone  W/N=ROI width  H/L=ROI height  P=params')
#     while True:
#         grey, blur, edges, masked, verts, lines = pipeline(frame, params)
#         grid = make_grid(frame, grey, blur, edges, masked, verts, lines, params)
#         cv2.imshow('Lane Pipeline – Astra', grid)
#         key = cv2.waitKey(0) & 0xFF
#         snap_count = handle_key(key, frame, snap_count, save_dir)
#         if snap_count == -1:
#             break
#     cv2.destroyAllWindows()

# else:
#     rclpy.init()
#     node = AstraPipelineNode()
#     spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     spin_thread.start()

#     print('Waiting for Astra frame on /camera/color/image_raw …')
#     print('S=save  Q=quit  +/-=zone  W/N=ROI width  H/L=ROI height  P=params')

#     while rclpy.ok():
#         frame = node.get_frame()
#         if frame is None:
#             placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
#             cv2.putText(placeholder, 'Waiting for Astra camera…',
#                         (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,200), 1)
#             cv2.imshow('Lane Pipeline – Astra', placeholder)
#             if cv2.waitKey(100) & 0xFF in (ord('q'), 27):
#                 break
#             continue

#         grey, blur, edges, masked, verts, lines = pipeline(frame, params)
#         grid = make_grid(frame, grey, blur, edges, masked, verts, lines, params)
#         cv2.imshow('Lane Pipeline – Astra', grid)

#         key = cv2.waitKey(1) & 0xFF
#         snap_count = handle_key(key, frame, snap_count, save_dir)
#         if snap_count == -1:
#             break

#     cv2.destroyAllWindows()
#     node.destroy_node()
#     rclpy.shutdown()


# The masked binary image (white lane edges on black) is exactly what
# feeds into the Hough line detector.

# Controls
# --------
#   S        – save all four pipeline images to ./lane_snapshots/
#   Q        – quit
#   +  / =   – increase Canny high threshold
#   -        – decrease Canny high threshold
#   W / N    – widen / narrow ROI top
#   H / L    – raise / lower ROI height
#   P        – print current parameter values

# Usage
# -----
#   # Make sure the Astra driver is running first:
#   #   ros2 launch orbbec_camera astra_pro.launch.py
  
#   source /home/chetan-satpute/lane_following/install/setup.bash
#   python3 rgb_to_grey.py                    # Astra live feed
#   python3 rgb_to_grey.py --image photo.jpg  # static image (no ROS needed)


import argparse
import os
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ── Tunable parameters (same defaults as kayro.py) ────────────────────────────
params = dict(
    kernel_size   = 9,      # Gaussian blur kernel (must be odd)
    canny_low     = 60,
    canny_high    = 150,
    roi_top_width = 0.20,   # fraction of half-frame-width at top of trapezoid
    roi_height    = 0.55,   # fraction of frame height where ROI top sits
)


def build_roi_mask(shape, roi_top_width, roi_height):
    h, w = shape[:2]
    cx = w / 2
    tl_x = int(cx - w * roi_top_width)
    tr_x = int(cx + w * roi_top_width)
    top_y = int(h * roi_height)
    verts = np.array([[(0, h), (tl_x, top_y), (tr_x, top_y), (w, h)]], dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, verts, 255)
    return mask, verts


def pipeline(frame, p):
    grey   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur   = cv2.GaussianBlur(grey, (p['kernel_size'], p['kernel_size']), 0)
    edges  = cv2.Canny(blur, p['canny_low'], p['canny_high'])
    mask, verts = build_roi_mask(frame.shape, p['roi_top_width'], p['roi_height'])
    masked = cv2.bitwise_and(edges, mask)
    return grey, blur, edges, masked, verts


def annotate(img_bgr, label):
    out = img_bgr.copy()
    cv2.putText(out, label, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out


def make_grid(frame, grey, blur, edges, masked, verts, p):
    """2×2 grid: original | greyscale | Canny edges | masked binary"""
    def to_bgr(img):
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if len(img.shape) == 2 else img

    orig = frame.copy()
    cv2.polylines(orig, verts, True, (0, 255, 255), 2)   # show ROI on original

    tl = annotate(orig,          '1. Original + ROI')
    tr = annotate(to_bgr(grey),  '2. Greyscale')
    bl = annotate(to_bgr(edges), '3. Canny edges')
    br = annotate(to_bgr(masked),'4. Masked binary  ← Hough input')

    top = np.hstack([tl, tr])
    bot = np.hstack([bl, br])
    grid = np.vstack([top, bot])

    h, w = grid.shape[:2]
    cv2.putText(grid,
                f'Canny {p["canny_low"]}/{p["canny_high"]}  '
                f'ROI w={p["roi_top_width"]:.2f} h={p["roi_height"]:.2f}  '
                f'  S=save  Q=quit  +/-=canny  W/N=width  H/L=height',
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return grid


def handle_key(key, frame, snap_count, save_dir):
    """Shared key handler. Returns updated snap_count, or -1 to quit."""
    if key in (ord('q'), 27):
        return -1
    elif key == ord('s'):
        grey, blur, edges, masked, _ = pipeline(frame, params)
        # Save original colour frame — this is what calibrate_shapes.py needs
        orig_path = os.path.join(save_dir, f'snap_{snap_count:04d}_original.jpg')
        cv2.imwrite(orig_path, frame)
        print(f'Saved → {orig_path}  ← copy this into straight/ left/ right/ etc.')
        for name, img in [('grey', grey), ('blur', blur),
                           ('edges', edges), ('masked', masked)]:
            path = os.path.join(save_dir, f'snap_{snap_count:04d}_{name}.png')
            cv2.imwrite(path, img)
            print(f'Saved → {path}')
        return snap_count + 1
    elif key in (ord('+'), ord('=')):
        params['canny_high'] = min(300, params['canny_high'] + 10)
    elif key == ord('-'):
        params['canny_high'] = max(params['canny_low'] + 10, params['canny_high'] - 10)
    elif key == ord('w'):
        params['roi_top_width'] = min(0.45, params['roi_top_width'] + 0.02)
    elif key == ord('n'):
        params['roi_top_width'] = max(0.05, params['roi_top_width'] - 0.02)
    elif key == ord('h'):
        params['roi_height'] = max(0.20, params['roi_height'] - 0.02)
    elif key == ord('l'):
        params['roi_height'] = min(0.90, params['roi_height'] + 0.02)
    elif key == ord('p'):
        print('\n── Current parameters ──')
        for k, v in params.items():
            print(f'  {k:<16} = {v}')
    return snap_count


# ── ROS2 subscriber node ───────────────────────────────────────────────────────

class AstraPipelineNode(Node):
    def __init__(self):
        super().__init__('lane_pipeline_viewer')
        self.bridge  = CvBridge()
        self.latest  = None          # most-recent frame (BGR)
        self._lock   = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._cb,
            sensor_qos,
        )
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


# ── Entry point ────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser()
parser.add_argument('--image', type=str, default=None,
                    help='Path to a static image (skips ROS2 live feed)')
args = parser.parse_args()

save_dir   = 'lane_snapshots'
os.makedirs(save_dir, exist_ok=True)
snap_count = 0

if args.image:
    # ── Static image mode (no ROS needed) ─────────────────────────────────────
    frame = cv2.imread(args.image)
    if frame is None:
        print(f'Cannot read {args.image}')
        raise SystemExit(1)

    print('Static image mode.  S=save  Q=quit  +/-=Canny  W/N=ROI width  H/L=ROI height  P=params')
    while True:
        grey, blur, edges, masked, verts = pipeline(frame, params)
        grid = make_grid(frame, grey, blur, edges, masked, verts, params)
        cv2.imshow('Lane Pipeline – Astra', grid)
        key = cv2.waitKey(0) & 0xFF
        snap_count = handle_key(key, frame, snap_count, save_dir)
        if snap_count == -1:
            break
    cv2.destroyAllWindows()

else:
    # ── Live Astra Pro Plus mode (ROS2) ───────────────────────────────────────
    rclpy.init()
    node = AstraPipelineNode()

    # Spin ROS2 in a background thread so the main thread can run the GUI
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Waiting for Astra frame on /camera/color/image_raw …')
    print('S=save  Q=quit  +/-=Canny  W/N=ROI width  H/L=ROI height  P=params')

    while rclpy.ok():
        frame = node.get_frame()
        if frame is None:
            # No frame yet — show a placeholder
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for Astra camera…',
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
            cv2.imshow('Lane Pipeline – Astra', placeholder)
            if cv2.waitKey(100) & 0xFF in (ord('q'), 27):
                break
            continue

        grey, blur, edges, masked, verts = pipeline(frame, params)
        grid = make_grid(frame, grey, blur, edges, masked, verts, params)
        cv2.imshow('Lane Pipeline – Astra', grid)

        key = cv2.waitKey(1) & 0xFF
        snap_count = handle_key(key, frame, snap_count, save_dir)
        if snap_count == -1:
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3
"""
Shape-Calibration Tool
======================
Run the full lane-detection pipeline on static images so you can tune
parameters BEFORE deploying on the bot.

Usage
-----
1. Create a folder with sub-folders named after each scenario:
       images/
           straight/     *.jpg / *.png   (straight lane)
           left_turn/
           right_turn/
           intersection/
           dead_end/

2. Run:
       python3 calibrate_shapes.py
   or point it at a different root folder:
       python3 calibrate_shapes.py --images_dir /path/to/your/folder

Keyboard controls (per image)
------------------------------
  Arrow Right / Space  – next image
  Arrow Left           – previous image
  g                    – cycle display mode:
                            [0] colour + lines overlay
                            [1] greyscale
                            [2] Gaussian blur
                            [3] Canny edges
                            [4] ROI mask applied to edges
  +  / =               – widen outer-boundary zone (include more frame)
  -                    – shrink outer-boundary zone (far-field only)
  w                    – widen ROI top
  n                    – narrow ROI top
  h                    – raise ROI (move up)
  l                    – lower ROI (move down)
  p                    – print current parameter values to terminal
  q  / Escape          – quit
"""

import os
import sys
import argparse
import glob
import cv2
import numpy as np


# ── Default pipeline parameters (mirror kayro.py defaults) ────────────────────
DEFAULTS = dict(
    roi_top_width    = 0.20,   # fraction of half-width at top of ROI trapezoid
    roi_height       = 0.55,   # y fraction where ROI top edge sits (0=top,1=bottom)
    outer_zone_frac  = 0.75,   # only lines with centre y < frac*H feed shape classifier
    kernel_size      = 9,      # Gaussian kernel size (must be odd)
    canny_low        = 60,
    canny_high       = 150,
    hough_threshold  = 45,
    hough_min_len    = 40,
    hough_max_gap    = 100,
)

# Actual folder names inside lane_detection/
CATEGORIES = ['straight', 'left', 'right', 'lane_endings', 'deadend']

COLOUR_MAP = {
    'horiz':      (255,   0,   0),   # blue
    'diag_left':  (  0, 255,   0),   # green
    'diag_right': (  0,   0, 255),   # red
    'vert_left':  (255,   0, 255),   # magenta
    'vert_right': (  0, 255, 255),   # cyan
    'excluded':   (100, 100, 100),   # grey
}
THICKNESS_MAP = {k: (1 if k == 'excluded' else 3) for k in COLOUR_MAP}

SHAPE_COLOUR = {
    'STRAIGHT':     (200, 200, 200),
    'TURN_LEFT':    (  0, 165, 255),
    'TURN_RIGHT':   (  0, 165, 255),
    'INTERSECTION': (  0, 255, 255),
    'DEAD_END':     (  0,   0, 255),
}

DISPLAY_MODES = ['overlay', 'grey', 'blur', 'edges', 'masked_edges']


# ── Pipeline functions ─────────────────────────────────────────────────────────

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


def process_image(im, p):
    """Return intermediate images and Hough lines."""
    gray    = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    blur    = cv2.GaussianBlur(gray, (p['kernel_size'], p['kernel_size']), 0)
    edges   = cv2.Canny(blur, p['canny_low'], p['canny_high'])
    mask, v = build_roi_mask(im.shape, p['roi_top_width'], p['roi_height'])
    masked  = cv2.bitwise_and(edges, mask)
    lines   = cv2.HoughLinesP(
        masked,
        rho=2, theta=np.pi/180,
        threshold=p['hough_threshold'],
        lines=np.array([]),
        minLineLength=p['hough_min_len'],
        maxLineGap=p['hough_max_gap'],
    )
    return gray, blur, edges, masked, lines, v


def classify_road_shape(lines, imshape, outer_zone_y):
    """
    Identical logic to kayro.py – returns (shape_str, debug_dict).
    Lines whose centre y > outer_zone_y are excluded from classification.
    """
    debug = {k: [] for k in COLOUR_MAP}

    if lines is None or len(lines) == 0:
        return 'STRAIGHT', debug

    h, w = imshape[:2]
    mid_x = w / 2.0

    has_horiz = has_diag_left = has_diag_right = has_vert_left = has_vert_right = False

    for line in lines:
        for x1, y1, x2, y2 in line:
            seg = (x1, y1, x2, y2)
            length = ((x2-x1)**2 + (y2-y1)**2)**0.5
            if length < 25:
                debug['excluded'].append(seg)
                continue
            cy = (y1 + y2) / 2.0
            if cy > outer_zone_y:
                debug['excluded'].append(seg)
                continue
            dx = x2 - x1
            abs_slope = abs((y2-y1)/dx) if dx != 0 else 999.0
            cx = (x1 + x2) / 2.0
            if abs_slope < 0.36:
                has_horiz = True;         debug['horiz'].append(seg)
            elif abs_slope > 2.75:
                if cx < mid_x:
                    has_vert_left = True; debug['vert_left'].append(seg)
                else:
                    has_vert_right = True;debug['vert_right'].append(seg)
            else:
                if cx < mid_x:
                    has_diag_left = True; debug['diag_left'].append(seg)
                else:
                    has_diag_right = True;debug['diag_right'].append(seg)

    if has_horiz and not has_diag_left and not has_diag_right:
        return 'DEAD_END', debug
    if has_horiz and (has_diag_left or has_diag_right):
        return 'INTERSECTION', debug
    if (has_horiz or has_vert_right) and not has_diag_right:
        if has_diag_left or has_vert_left:
            return 'TURN_RIGHT', debug
    if (has_horiz or has_vert_left) and not has_diag_left:
        if has_diag_right or has_vert_right:
            return 'TURN_LEFT', debug
    return 'STRAIGHT', debug


def render_overlay(im, debug_lines, roi_verts, outer_zone_y, road_shape, label, p):
    """Colour-coded line overlay + annotations."""
    out = im.copy()
    h, w = out.shape[:2]

    # Draw colour-coded Hough lines
    for cat, segs in debug_lines.items():
        col = COLOUR_MAP[cat]
        th  = THICKNESS_MAP[cat]
        for (x1, y1, x2, y2) in segs:
            cv2.line(out, (x1, y1), (x2, y2), col, th)

    # ROI trapezoid outline
    cv2.polylines(out, roi_verts, True, (0, 255, 255), 1)

    # Outer-boundary zone cutoff (dashed cyan)
    zone_yi = int(outer_zone_y)
    for sx in range(0, w, 20):
        cv2.line(out, (sx, zone_yi), (min(sx+12, w), zone_yi), (0, 220, 220), 2)
    cv2.putText(out, f'zone y<{zone_yi}  frac={p["outer_zone_frac"]:.2f}',
                (10, zone_yi - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 220), 1)

    # Vertical mid-line
    cv2.line(out, (w//2, 0), (w//2, h), (180, 180, 0), 1)

    # Legend
    lx = 10
    for i, (cat, col) in enumerate(COLOUR_MAP.items()):
        ly = 22 + i * 22
        cv2.rectangle(out, (lx, ly-12), (lx+14, ly+2), col, -1)
        cv2.putText(out, f'{cat.upper()} ({len(debug_lines[cat])})',
                    (lx+18, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,255), 1)

    # Shape result banner
    sc = SHAPE_COLOUR.get(road_shape, (200,200,200))
    cv2.putText(out, f'DETECTED: {road_shape}',
                (w//2 - 100, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, sc, 2)

    # Ground-truth label (folder name)
    cv2.putText(out, f'Label: {label}',
                (w//2 - 80, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 1)

    # Correct / wrong indicator
    expected = {
        'straight':     'STRAIGHT',
        'left':         'TURN_LEFT',
        'right':        'TURN_RIGHT',
        'lane_endings': 'INTERSECTION',
        'deadend':      'DEAD_END',
    }.get(label)
    if expected is not None:
        ok = road_shape == expected
        cv2.putText(out, '✓ CORRECT' if ok else f'✗ WANT {expected}',
                    (w - 180, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0,220,0) if ok else (0,0,255), 2)

    return out


def bgr_to_3ch(gray_or_1ch):
    """Ensure an image has 3 channels for display."""
    if len(gray_or_1ch.shape) == 2:
        return cv2.cvtColor(gray_or_1ch, cv2.COLOR_GRAY2BGR)
    return gray_or_1ch


# ── Image loader ───────────────────────────────────────────────────────────────

def load_dataset(root):
    """
    Returns list of (filepath, label) sorted by label then filename.
    Accepts both organised sub-folders and a flat folder of images.
    """
    exts = ('*.jpg', '*.jpeg', '*.png', '*.bmp', '*.webp')
    entries = []

    # Try sub-folder structure first
    for cat in CATEGORIES:
        cat_dir = os.path.join(root, cat)
        if os.path.isdir(cat_dir):
            for ext in exts:
                for fp in sorted(glob.glob(os.path.join(cat_dir, ext))):
                    entries.append((fp, cat))

    # Fall back: flat folder, label = 'unknown'
    if not entries:
        for ext in exts:
            for fp in sorted(glob.glob(os.path.join(root, ext))):
                entries.append((fp, 'unknown'))

    return entries


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Lane shape calibration tool')
    # Default: the folder containing this script (lane_detection/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--images_dir', default=script_dir,
                        help='Root folder containing category sub-folders '
                             '(default: same folder as this script)')
    args = parser.parse_args()

    dataset = load_dataset(args.images_dir)
    if not dataset:
        print(f'[ERROR] No images found under "{args.images_dir}"')
        print('  Expected sub-folders (with images inside):')
        for cat in CATEGORIES:
            print(f'    {args.images_dir}/{cat}/*.jpg')
        sys.exit(1)

    print(f'Loaded {len(dataset)} images from "{args.images_dir}"')
    print('  Keyboard: Space/→=next  ←=prev  g=display mode  +/-=zone  w/n=ROI width  h/l=ROI height  p=print params  q=quit')

    p = dict(DEFAULTS)
    idx = 0
    display_mode = 0   # 0=overlay 1=grey 2=blur 3=edges 4=masked
    display_scale = 0.5  # window size; press [ to shrink, ] to enlarge

    while True:
        filepath, label = dataset[idx]
        im = cv2.imread(filepath)
        if im is None:
            print(f'[WARN] Could not read {filepath}, skipping')
            idx = (idx + 1) % len(dataset)
            continue

        # ── Run pipeline ──────────────────────────────────────────────────────
        gray, blur_im, edges, masked, lines, roi_verts = process_image(im, p)
        outer_zone_y = im.shape[0] * p['outer_zone_frac']
        road_shape, debug_lines = classify_road_shape(lines, im.shape, outer_zone_y)

        # ── Render chosen display mode ─────────────────────────────────────────
        if display_mode == 0:
            vis = render_overlay(im, debug_lines, roi_verts, outer_zone_y, road_shape, label, p)
        elif display_mode == 1:
            vis = bgr_to_3ch(gray)
        elif display_mode == 2:
            vis = bgr_to_3ch(blur_im)
        elif display_mode == 3:
            vis = bgr_to_3ch(edges)
        else:
            vis = bgr_to_3ch(masked)

        # Progress bar at the top
        h_v, w_v = vis.shape[:2]
        progress_x = int(w_v * (idx / max(1, len(dataset)-1)))
        cv2.rectangle(vis, (0, 0), (w_v, 6), (50, 50, 50), -1)
        cv2.rectangle(vis, (0, 0), (progress_x, 6), (0, 200, 0), -1)

        # Filename + index
        fname = os.path.basename(filepath)
        cv2.putText(vis, f'[{idx+1}/{len(dataset)}] {label}/{fname}  mode={DISPLAY_MODES[display_mode]}',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 0), 1)

        if display_scale != 1.0:
            dh, dw = vis.shape[:2]
            vis = cv2.resize(vis, (int(dw * display_scale), int(dh * display_scale)),
                             interpolation=cv2.INTER_AREA)
        cv2.imshow('Shape Calibration', vis)

        # ── Keyboard ───────────────────────────────────────────────────────────
        key = cv2.waitKey(0) & 0xFF

        if key in (ord('q'), 27):      # q / Escape
            break
        elif key == ord('['):
            display_scale = max(0.2, display_scale - 0.05)
            print(f'display_scale → {display_scale:.2f}')
            continue
        elif key == ord(']'):
            display_scale = min(2.0, display_scale + 0.05)
            print(f'display_scale → {display_scale:.2f}')
            continue
        elif key in (ord(' '), 83):    # Space or Right-arrow
            idx = (idx + 1) % len(dataset)
        elif key == 81:                # Left-arrow
            idx = (idx - 1) % len(dataset)
        elif key == ord('g'):          # cycle display mode
            display_mode = (display_mode + 1) % len(DISPLAY_MODES)
        elif key in (ord('+'), ord('=')):
            p['outer_zone_frac'] = min(1.0, p['outer_zone_frac'] + 0.02)
            print(f"outer_zone_frac → {p['outer_zone_frac']:.2f}")
        elif key == ord('-'):
            p['outer_zone_frac'] = max(0.10, p['outer_zone_frac'] - 0.02)
            print(f"outer_zone_frac → {p['outer_zone_frac']:.2f}")
        elif key == ord('w'):
            p['roi_top_width'] = min(0.45, p['roi_top_width'] + 0.02)
            print(f"roi_top_width   → {p['roi_top_width']:.2f}")
        elif key == ord('n'):
            p['roi_top_width'] = max(0.05, p['roi_top_width'] - 0.02)
            print(f"roi_top_width   → {p['roi_top_width']:.2f}")
        elif key == ord('h'):
            p['roi_height'] = max(0.20, p['roi_height'] - 0.02)
            print(f"roi_height      → {p['roi_height']:.2f}")
        elif key == ord('l'):
            p['roi_height'] = min(0.90, p['roi_height'] + 0.02)
            print(f"roi_height      → {p['roi_height']:.2f}")
        elif key == ord('p'):
            print('\n── Current parameters ──────────────────────────')
            for k, v in p.items():
                print(f'  {k:<20} = {v}')
            print('  Copy these into kayro.py __init__ to apply.\n')

    cv2.destroyAllWindows()
    print('\nFinal parameters:')
    for k, v in p.items():
        print(f'  {k:<20} = {v}')


if __name__ == '__main__':
    main()

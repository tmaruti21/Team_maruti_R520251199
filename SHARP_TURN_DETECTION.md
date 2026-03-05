# Sharp Turn Detection Guide

## Overview

The lane detection system now includes **sharp turn detection** that specifically identifies when the vehicle is approaching a sharp turn (not gradual curves).

## How It Works

### Curvature Calculation

The system calculates curvature using two key metrics:

1. **Slope Asymmetry**: Difference between left and right lane slopes
   - Straight lanes: Both slopes are similar in magnitude
   - Sharp turns: One lane has much steeper slope than the other

2. **Width Ratio**: Change in lane width from bottom to top of ROI
   - Straight lanes: Width stays consistent (ratio ≈ 1.0)
   - Sharp turns: Width changes significantly (ratio far from 1.0)

**Formula**: `Curvature = Slope_Asymmetry × |1 - Width_Ratio|`

### Detection Threshold

- **sharp_turn_threshold**: `0.0008` (default)
- Higher threshold = Only very sharp turns detected
- Lower threshold = More sensitive, detects gentler curves too

### Turn Direction

- **RIGHT TURN**: Right lane has steeper slope
- **LEFT TURN**: Left lane has steeper slope

## Visual Indicators

When a sharp turn is detected, you'll see:

1. **Red warning box** at top center: "SHARP TURN LEFT!" or "SHARP TURN RIGHT!"
2. **Yellow arrow** in the middle: Points in turn direction
3. **Curvature value** at bottom: Real-time curvature measurement

## Tuning Parameters

Edit these values in [LaneDetect_AstraCam.py](src/Lane-Detection/LaneDetect_AstraCam.py):

```python
# Line ~37
self.sharp_turn_threshold = 0.0008  # Adjust this value
self.history_size = 5  # Smoothing frames
```

### Tuning Tips

| Issue | Solution |
|-------|----------|
| Detects too many false turns | Increase `sharp_turn_threshold` (try 0.001, 0.0012) |
| Misses sharp turns | Decrease `sharp_turn_threshold` (try 0.0006, 0.0005) |
| Turn detection flickers | Increase `history_size` (try 7, 10) |
| Slow to detect turns | Decrease `history_size` (try 3) |

### Finding the Right Threshold

1. Run the system and watch the **curvature value** at bottom of screen

2. Drive through your track:
   - On straight sections, note the curvature (should be < 0.0003)
   - On sharp turns, note the curvature (might be 0.0008 - 0.003)
   - On gentle curves, note the curvature (might be 0.0004 - 0.0007)

3. Set threshold between straight and sharp turn values

**Example:**
- Straight: 0.0002
- Gentle curve: 0.0005
- Sharp turn: 0.0012
- **Good threshold**: 0.0008 (catches sharp turns, ignores straight/gentle)

## Testing

### Run the detection:
```bash
cd ~/depth_cam
./run_astra_lane_detection.sh
```

### What to look for:

✅ **Good Detection:**
- Sharp turns trigger warning and arrow
- Straight sections show no warning
- Gentle curves don't trigger (if that's desired)

❌ **Needs Tuning:**
- Warning appears on straight sections → Increase threshold
- No warning on sharp turns → Decrease threshold
- Warning flickers on/off rapidly → Increase history_size

## Integration with Rover

When integrating with motor control:

```python
if self.turn_detected and self.turn_direction == 'LEFT':
    # Sharp left turn ahead
    # Slow down rover
    # Prepare for left turn maneuver
    
elif self.turn_detected and self.turn_direction == 'RIGHT':
    # Sharp right turn ahead
    # Slow down rover
    # Prepare for right turn maneuver
    
else:
    # Straight or gentle curve
    # Normal speed
```

## Debugging

The system logs sharp turns:
```
🔄 SHARP TURN DETECTED: LEFT (Curvature: 0.001234)
```

Check terminal output for these messages to verify detection is working.

## Advanced: Understanding the Math

### Why This Method?

Traditional lane detection uses:
- **Radius of curvature**: Requires fitting polynomial curves
- **Complex**: Computationally expensive

Our method uses:
- **Slope asymmetry**: Direct from Hough lines
- **Width ratio**: Simple calculation
- **Fast**: Real-time performance

### Curvature Components

```python
# Slope asymmetry (how different are the lane angles?)
slope_asymmetry = abs(abs(left_slope) - abs(right_slope))

# Width change (do lanes converge/diverge?)
width_ratio = top_width / bottom_width

# Combined curvature metric
curvature = slope_asymmetry × abs(1 - width_ratio)
```

**Example Values:**

Straight lane:
- left_slope = -0.8, right_slope = 0.8
- slope_asymmetry = 0.0
- width_ratio = 1.0
- **curvature = 0.0 × 0.0 = 0.0**

Sharp left turn:
- left_slope = -0.5, right_slope = 1.2
- slope_asymmetry = 0.7
- width_ratio = 0.6
- **curvature = 0.7 × 0.4 = 0.28** ← Definitely a sharp turn!

## Next Steps

1. **Test on your track** - Run through different sections
2. **Note curvature values** - Watch the bottom display
3. **Tune threshold** - Adjust to match your definition of "sharp"
4. **Integrate with control** - Use turn detection in your rover logic

---

**Quick Reference:**
- Default threshold: `0.0008`
- Increase if too sensitive
- Decrease if missing turns
- Watch real-time curvature at bottom of screen

 #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Real-time Lane Detection from Astra Pro Plus Depth Camera
# Uses ROS2 to subscribe to camera feed
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import numpy as np
# import cv2


# class LaneDetectorNode(Node):
#     def __init__(self):
#         super().__init__('lane_detector_astra')
        
#         # Create CV Bridge for converting ROS images to OpenCV
#         self.bridge = CvBridge()
        
#         # Initialize previous line positions for smoothing
#         self.x1Prev = None
#         self.x2Prev = None
#         self.y1Prev = None
#         self.y2Prev = None
#         self.x1NPrev = None
#         self.x2NPrev = None
#         self.y1NPrev = None
        
#         self.frameNum = 0
#         self.recording = False
#         self.videoOut = None
#         self.fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        
#         # Turn state (used by visualisation overlay)
#         self.turn_detected = False
#         self.turn_direction = None  # 'LEFT' or 'RIGHT'

#         # Autonomous navigation parameters
#         self.forward_speed     = 0.25   # m/s nominal forward speed
#         self.max_angular_speed = 0.6    # rad/s maximum turning rate
#         self.turn_threshold    = 0.15   # normalised lateral error that classifies as turn
#         # Look-ahead fraction: evaluate lane centre this far ahead (fraction of frame height)
#         self.look_ahead_fraction = 0.20

#         # Dead-end / 360° spin state
#         self.dead_end_confirm_count = 0
#         self.dead_end_confirm_thresh = 10   # frames of DEAD_END before committing to spin
#         self.is_spinning = False
#         self.spin_frames_remaining = 0
#         self.spin_frames_total = 240        # ~12 s at 20 fps → ~360° at 0.6 rad/s

#         # Adjustable ROI parameters
#         self.roi_top_width = 0.20  # Width at top (0.2 = 20% from center on each side)
#         self.roi_height = 0.55  # Height position (0.55 = 55% down from top)
#         self.show_roi = True  # Toggle ROI visualization

#         # /cmd_vel publisher
#         self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Subscribe to color image topic – use BEST_EFFORT to match camera driver QoS
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.image_callback,
#             sensor_qos)
        
#         self.get_logger().info('='*60)
#         self.get_logger().info('Lane Detection Node for Astra Pro Plus Started!')
#         self.get_logger().info('='*60)
#         self.get_logger().info('Subscribing to /camera/color/image_raw')
#         self.get_logger().info('Controls:')
#         self.get_logger().info('  q - Quit')
#         self.get_logger().info('  s - Start/stop recording')
#         self.get_logger().info('  w - Widen ROI | n - Narrow ROI')
#         self.get_logger().info('  h - Raise ROI height | l - Lower ROI height')
#         self.get_logger().info('  r - Toggle ROI visualization')
#         self.get_logger().info('='*60)
    
#     def image_callback(self, msg):
#         """Callback function that processes incoming images from the camera"""
#         try:
#             # Convert ROS Image message to OpenCV format
#             im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f'Failed to convert image: {str(e)}')
#             return
        
#         self.frameNum += 1
#         nav_published = False
#         if self.frameNum % 30 == 0:
#             self.get_logger().info(f'Processing frame {self.frameNum}...')
        
#         imshape = im.shape
        
#         # -------------GREYSCALE IMAGE---------------
#         grayIm = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        
#         #------------GAUSSIAN SMOOTHING-----------------
#         kernel_size = 9
#         smoothedIm = cv2.GaussianBlur(grayIm, (kernel_size, kernel_size), 0)
        
#         #-------------EDGE DETECTION---------------------
#         minVal = 60
#         maxVal = 150
#         edgesIm = cv2.Canny(smoothedIm, minVal, maxVal)
        
#         #-------------------------CREATE MASK--------------------------------
#         # Calculate ROI vertices based on adjustable parameters
#         center_x = imshape[1] / 2
#         top_left_x = int(center_x - (imshape[1] * self.roi_top_width))
#         top_right_x = int(center_x + (imshape[1] * self.roi_top_width))
#         top_y = int(imshape[0] * self.roi_height)
        
#         # Trapezoid ROI: wider at bottom, narrower at top
#         vertices = np.array([[(0, imshape[0]),
#                               (top_left_x, top_y), 
#                               (top_right_x, top_y), 
#                               (imshape[1], imshape[0])]], dtype=np.int32)
        
#         mask = np.zeros_like(edgesIm)   
#         color = 255
#         cv2.fillPoly(mask, vertices, color)
        
#         #----------------------APPLY MASK TO IMAGE-------------------------------
#         maskedIm = cv2.bitwise_and(edgesIm, mask)
        
#         #-----------------------HOUGH LINES------------------------------------
#         rho = 2
#         theta = np.pi/180
#         threshold = 45
#         min_line_len = 40
#         max_line_gap = 100
#         lines = cv2.HoughLinesP(maskedIm, rho, theta, threshold, np.array([]),
#                                     minLineLength=min_line_len, maxLineGap=max_line_gap)

#         # Classify structural road shape from raw Hough lines
#         road_shape = self.classify_road_shape(lines, imshape)

#         # Default output image
#         outputIm = im.copy()

#         # ── 360° SPIN STATE (highest priority) ──────────────────────────────────
#         if self.is_spinning:
#             self.spin_frames_remaining -= 1
#             twist = Twist()
#             twist.angular.z = float(self.max_angular_speed)
#             self.cmd_publisher.publish(twist)
#             nav_published = True
#             if self.spin_frames_remaining <= 0:
#                 self.is_spinning = False
#                 self.dead_end_confirm_count = 0
#                 self.get_logger().info('360° spin complete – resuming lane following')
#             pct = int(100 * (1 - self.spin_frames_remaining / max(1, self.spin_frames_total)))
#             cv2.putText(outputIm, f'DEAD END – SPINNING 360 ({pct}%)',
#                        (10, imshape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
#         else:
#             # Accumulate evidence for dead-end before committing to spin
#             if road_shape == 'DEAD_END':
#                 self.dead_end_confirm_count += 1
#                 if self.dead_end_confirm_count >= self.dead_end_confirm_thresh:
#                     self.is_spinning = True
#                     self.spin_frames_remaining = self.spin_frames_total
#                     self.get_logger().info('DEAD END confirmed – starting 360° spin')
#             else:
#                 self.dead_end_confirm_count = 0

#         # Check if we got more than 1 line (skip lane processing while spinning)
#         if not self.is_spinning and lines is not None and len(lines) > 2:
            
#             #-----------------------Separate Lines Into Positive/Negative Slope--------------------------
#             slopePositiveLines = []
#             slopeNegativeLines = []
#             yValues = []
            
#             addedPos = False
#             addedNeg = False
#             for currentLine in lines:   
#                 for x1,y1,x2,y2 in currentLine:
#                     lineLength = ((x2-x1)**2 + (y2-y1)**2)**.5
#                     if lineLength > 30:
#                         if x2 != x1:
#                             slope = (y2-y1)/(x2-x1)
#                             if slope > 0: 
#                                 tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
#                                 ang = np.arctan(tanTheta)*180/np.pi
#                                 if abs(ang) < 85 and abs(ang) > 20:
#                                     slopeNegativeLines.append([x1,y1,x2,y2,-slope])
#                                     yValues.append(y1)
#                                     yValues.append(y2)
#                                     addedPos = True
#                             if slope < 0:
#                                 tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
#                                 ang = np.arctan(tanTheta)*180/np.pi
#                                 if abs(ang) < 85 and abs(ang) > 20:
#                                     slopePositiveLines.append([x1,y1,x2,y2,-slope])
#                                     yValues.append(y1)
#                                     yValues.append(y2)
#                                     addedNeg = True
                   
#             # If we didn't get any positive lines         
#             if not addedPos:
#                 for currentLine in lines:
#                     for x1,y1,x2,y2 in currentLine:
#                         if x2 != x1:
#                             slope = (y2-y1)/(x2-x1)
#                             if slope > 0:
#                                 tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
#                                 ang = np.arctan(tanTheta)*180/np.pi
#                                 if abs(ang) < 80 and abs(ang) > 15:
#                                     slopeNegativeLines.append([x1,y1,x2,y2,-slope])
#                                     yValues.append(y1)
#                                     yValues.append(y2)
            
#             # If we didn't get any negative lines
#             if not addedNeg:
#                 for currentLine in lines:
#                     for x1,y1,x2,y2 in currentLine:
#                         if x2 != x1:
#                             slope = (y2-y1)/(x2-x1)
#                             if slope < 0:
#                                 tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
#                                 ang = np.arctan(tanTheta)*180/np.pi
#                                 if abs(ang) < 85 and abs(ang) > 15:
#                                     slopePositiveLines.append([x1,y1,x2,y2,-slope])           
#                                     yValues.append(y1)
#                                     yValues.append(y2)
            
#             #------------------------Get Positive/Negative Slope Averages-----------------------------------
#             if len(slopePositiveLines) > 0 and len(slopeNegativeLines) > 0:
#                 positiveSlopes = np.asarray(slopePositiveLines)[:,4]
#                 posSlopeMedian = np.median(positiveSlopes)
#                 posSlopesGood = []
#                 for slope in positiveSlopes:
#                     if abs(slope-posSlopeMedian) < posSlopeMedian*.2:
#                         posSlopesGood.append(slope)
                
#                 if len(posSlopesGood) > 0:
#                     posSlopeMean = np.mean(np.asarray(posSlopesGood))
#                 else:
#                     posSlopeMean = posSlopeMedian
                        
#                 negativeSlopes = np.asarray(slopeNegativeLines)[:,4]
#                 negSlopeMedian = np.median(negativeSlopes)
#                 negSlopesGood = []
#                 for slope in negativeSlopes:
#                     if abs(slope-negSlopeMedian) < .9:
#                         negSlopesGood.append(slope)
                
#                 if len(negSlopesGood) > 0:
#                     negSlopeMean = np.mean(np.asarray(negSlopesGood))
#                 else:
#                     negSlopeMean = negSlopeMedian
                    
#                 #--------------------------Get Average x Coord When y Coord Of Line = 0----------------------------
#                 # Positive Lines
#                 xInterceptPos = []
#                 for line in slopePositiveLines:
#                         x1 = line[0]
#                         y1 = im.shape[0]-line[1]
#                         slope = line[4]
#                         yIntercept = y1-slope*x1
#                         xIntercept = -yIntercept/slope
#                         if xIntercept == xIntercept:
#                             xInterceptPos.append(xIntercept)
                        
#                 if len(xInterceptPos) > 0:
#                     xIntPosMed = np.median(xInterceptPos)
#                     xIntPosGood = []
#                     for line in slopePositiveLines:
#                             x1 = line[0]
#                             y1 = im.shape[0]-line[1]
#                             slope = line[4]
#                             yIntercept = y1-slope*x1
#                             xIntercept = -yIntercept/slope
#                             if abs(xIntercept-xIntPosMed) < .35*xIntPosMed:
#                                 xIntPosGood.append(xIntercept)
                                
#                     if len(xIntPosGood) > 0:
#                         xInterceptPosMean = np.mean(np.asarray(xIntPosGood))
#                     else:
#                         xInterceptPosMean = xIntPosMed
                
#                     # Negative Lines 
#                     xInterceptNeg = []
#                     for line in slopeNegativeLines:
#                         x1 = line[0]
#                         y1 = im.shape[0]-line[1]
#                         slope = line[4]
#                         yIntercept = y1-slope*x1
#                         xIntercept = -yIntercept/slope
#                         if xIntercept == xIntercept:
#                                 xInterceptNeg.append(xIntercept)
                            
#                     if len(xInterceptNeg) > 0:
#                         xIntNegMed = np.median(xInterceptNeg)
#                         xIntNegGood = []
#                         for line in slopeNegativeLines:
#                             x1 = line[0]
#                             y1 = im.shape[0]-line[1]
#                             slope = line[4]
#                             yIntercept = y1-slope*x1
#                             xIntercept = -yIntercept/slope
#                             if abs(xIntercept-xIntNegMed)< .35*xIntNegMed: 
#                                     xIntNegGood.append(xIntercept)
                            
#                         if len(xIntNegGood) > 0:
#                             xInterceptNegMean = np.mean(np.asarray(xIntNegGood))
#                         else:
#                             xInterceptNegMean = xIntNegMed
                    
#                         # ----------------------PLOT LANE LINES------------------------------
#                         colorLines = im.copy()
                        
#                         # Positive Slope Line
#                         slope = posSlopeMean
#                         x1 = xInterceptPosMean
#                         y1 = 0
#                         y2 = imshape[0] - (imshape[0]-imshape[0]*.35)
#                         x2 = (y2-y1)/slope + x1
                        
#                         x1 = int(round(x1))
#                         x2 = int(round(x2))
#                         y1 = int(round(y1))
#                         y2 = int(round(y2))
                        
#                         # Smoothing filter
#                         jumpThresh = 50
#                         if self.x1Prev is not None:
#                             if abs(x1-self.x1Prev) > 3 and abs(x1-self.x1Prev) < jumpThresh:
#                                 x1 = self.x1Prev + int(np.sign(x1-self.x1Prev)*1)
                                
#                             if abs(x2-self.x2Prev) > 3 and abs(x2-self.x2Prev) < jumpThresh:
#                                 x2 = self.x2Prev + int(np.sign(x2-self.x2Prev)*1)
                        
#                         cv2.line(colorLines,(x1,im.shape[0]-y1),(x2,imshape[0]-y2),(0,255,0),4)
                        
#                         # Negative Slope Line
#                         slope = negSlopeMean
#                         x1N = xInterceptNegMean
#                         y1N = 0
#                         x2N = (y2-y1N)/slope + x1N
                        
#                         x1N = int(round(x1N))
#                         x2N = int(round(x2N))
#                         y1N = int(round(y1N))
                        
#                         if self.x1NPrev is not None:
#                             if abs(x1N-self.x1NPrev) > 3 and abs(x1N-self.x1NPrev) < jumpThresh:
#                                 x1N = self.x1NPrev + int(np.sign(x1N-self.x1NPrev)*1)
                                
#                             if abs(x2N-self.x2NPrev) > 3 and abs(x2N-self.x2NPrev) < jumpThresh:
#                                 x2N = self.x2NPrev + int(np.sign(x2N-self.x2NPrev)*1)
                        
#                         cv2.line(colorLines,(x1N,im.shape[0]-y1N),(x2N,imshape[0]-y2),(0,255,0),4)
                        
#                         # Store previous values
#                         self.x1Prev = x1
#                         self.x2Prev = x2
#                         self.y1Prev = y1
#                         self.y2Prev = y2
#                         self.x1NPrev = x1N
#                         self.x2NPrev = x2N
#                         self.y1NPrev = y1N
                        
#                         #-------------------------------------SHAPE-PRIORITY NAVIGATION-----------------------------------------
#                         # Priority 1: structural shape features trigger only when rover is AT the junction.
#                         # Priority 2: proportional near look-ahead controller for straight / gentle curves.

#                         if road_shape == 'TURN_LEFT':
#                             # _| pattern: right side blocked, path opens left
#                             twist = Twist()
#                             twist.linear.x  = float(self.forward_speed * 0.4)
#                             twist.angular.z = float(self.max_angular_speed)
#                             self.turn_detected = True
#                             self.turn_direction = 'LEFT'
#                             error = 0.0

#                         elif road_shape == 'TURN_RIGHT':
#                             # |_ pattern: left side blocked, path opens right
#                             twist = Twist()
#                             twist.linear.x  = float(self.forward_speed * 0.4)
#                             twist.angular.z = float(-self.max_angular_speed)
#                             self.turn_detected = True
#                             self.turn_direction = 'RIGHT'
#                             error = 0.0

#                         elif road_shape == 'INTERSECTION':
#                             # Cross / T-junction: go forward slowly, gentle centering
#                             look_ahead_px = imshape[0] * self.look_ahead_fraction
#                             x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
#                             x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
#                             lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
#                             error = (lane_center_ahead - imshape[1] / 2.0) / (imshape[1] / 2.0)
#                             error = max(-1.0, min(1.0, error))
#                             twist = Twist()
#                             twist.linear.x  = float(self.forward_speed * 0.5)
#                             twist.angular.z = float(-self.max_angular_speed * 0.3 * error)
#                             self.turn_detected = False
#                             self.turn_direction = None

#                         else:  # STRAIGHT – proportional near look-ahead controller
#                             look_ahead_px = imshape[0] * self.look_ahead_fraction
#                             x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
#                             x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
#                             lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
#                             error = (lane_center_ahead - imshape[1] / 2.0) / (imshape[1] / 2.0)
#                             error = max(-1.0, min(1.0, error))
#                             twist = Twist()
#                             twist.linear.x  = float(self.forward_speed * (1.0 - 0.5 * abs(error)))
#                             twist.angular.z = float(-self.max_angular_speed * error)
#                             if abs(error) > self.turn_threshold:
#                                 self.turn_detected = True
#                                 self.turn_direction = 'RIGHT' if error > 0 else 'LEFT'
#                             else:
#                                 self.turn_detected = False
#                                 self.turn_direction = None

#                         self.cmd_publisher.publish(twist)
#                         nav_published = True
                        
#                         #-------------------------------------Blend Image-----------------------------------------
#                         laneFill = im.copy()
#                         vertices = np.array([[(x1,im.shape[0]-y1),(x2,im.shape[0]-y2),  (x2N,imshape[0]-y2),
#                                                               (x1N,imshape[0]-y1N)]], dtype=np.int32)
#                         color = [241,255,1]
#                         cv2.fillPoly(laneFill, vertices, color)
#                         opacity = .25
#                         blendedIm = cv2.addWeighted(laneFill,opacity,im,1-opacity,0,im)
#                         cv2.line(blendedIm,(x1,im.shape[0]-y1),(x2,imshape[0]-y2),(0,255,0),4)
#                         cv2.line(blendedIm,(x1N,im.shape[0]-y1N),(x2N,imshape[0]-y2),(0,255,0),4)
                        
#                         #-------------------------------------TURN VISUALIZATION-----------------------------------------
#                         if self.turn_detected and self.turn_direction:
#                             src = 'SHAPE' if road_shape in ('TURN_LEFT', 'TURN_RIGHT') else 'LANE'
#                             warning_text = f'[{src}] TURN {self.turn_direction}!'
#                             font = cv2.FONT_HERSHEY_SIMPLEX
#                             font_scale = 1.2
#                             thickness = 3
                            
#                             # Get text size for background box
#                             (text_width, text_height), baseline = cv2.getTextSize(warning_text, font, font_scale, thickness)
                            
#                             # Position at top center
#                             text_x = (imshape[1] - text_width) // 2
#                             text_y = 60
                            
#                             # Draw red background box
#                             box_coords = ((text_x - 10, text_y - text_height - 10), 
#                                         (text_x + text_width + 10, text_y + baseline + 10))
#                             cv2.rectangle(blendedIm, box_coords[0], box_coords[1], (0, 0, 255), -1)
                            
#                             # Draw white text
#                             cv2.putText(blendedIm, warning_text, (text_x, text_y), 
#                                       font, font_scale, (255, 255, 255), thickness)
                            
#                             # Draw direction arrow
#                             arrow_y = imshape[0] // 2
#                             center_x = imshape[1] // 2
#                             arrow_length = 80
                            
#                             if self.turn_direction == 'LEFT':
#                                 # Left arrow
#                                 start_point = (center_x + 30, arrow_y)
#                                 end_point = (center_x - arrow_length + 30, arrow_y)
#                             else:
#                                 # Right arrow
#                                 start_point = (center_x - 30, arrow_y)
#                                 end_point = (center_x + arrow_length - 30, arrow_y)
                            
#                             # Draw thick yellow arrow
#                             cv2.arrowedLine(blendedIm, start_point, end_point, 
#                                           (0, 255, 255), 8, tipLength=0.4)
                            
#                             # Log sharp turn detection
#                             if self.frameNum % 30 == 0:
#                                 self.get_logger().info(f'TURNING {self.turn_direction}  shape={road_shape}  error={error:+.2f}  ang={twist.angular.z:+.2f}')
                        
#                         # Display navigation info
#                         nav_info = (f'Shape:{road_shape}  Err:{error:+.2f}'
#                                     f'  Lin:{twist.linear.x:.2f}  Ang:{twist.angular.z:+.2f}')
#                         cv2.putText(blendedIm, nav_info, (10, imshape[0] - 20),
#                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
#                         outputIm = blendedIm
        
#         #-------------------------------------ROI VISUALIZATION-----------------------------------------
#         if self.show_roi:
#             # Draw ROI overlay on output image
#             roi_overlay = outputIm.copy()
            
#             # Calculate ROI vertices (same as mask)
#             center_x = imshape[1] / 2
#             top_left_x = int(center_x - (imshape[1] * self.roi_top_width))
#             top_right_x = int(center_x + (imshape[1] * self.roi_top_width))
#             top_y = int(imshape[0] * self.roi_height)
            
#             roi_vertices = np.array([[(0, imshape[0]),
#                                       (top_left_x, top_y), 
#                                       (top_right_x, top_y), 
#                                       (imshape[1], imshape[0])]], dtype=np.int32)
            
#             # Draw semi-transparent cyan overlay
#             cv2.fillPoly(roi_overlay, roi_vertices, (255, 255, 0))
#             outputIm = cv2.addWeighted(outputIm, 0.9, roi_overlay, 0.1, 0)
            
#             # Draw ROI border lines
#             cv2.polylines(outputIm, roi_vertices, True, (0, 255, 255), 2)
            
#             # Draw corner markers
#             marker_size = 15
#             # Top-left corner
#             cv2.line(outputIm, (top_left_x, top_y), (top_left_x + marker_size, top_y), (0, 255, 255), 3)
#             cv2.line(outputIm, (top_left_x, top_y), (top_left_x, top_y + marker_size), (0, 255, 255), 3)
#             # Top-right corner
#             cv2.line(outputIm, (top_right_x, top_y), (top_right_x - marker_size, top_y), (0, 255, 255), 3)
#             cv2.line(outputIm, (top_right_x, top_y), (top_right_x, top_y + marker_size), (0, 255, 255), 3)
            
#             # Display ROI settings
#             roi_info = f"ROI: Width={self.roi_top_width:.2f} Height={self.roi_height:.2f}"
#             cv2.putText(outputIm, roi_info, (10, 30), 
#                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
#         # Fallback: lane geometry unavailable – use shape classifier alone
#         if not nav_published:
#             twist = Twist()
#             if road_shape == 'TURN_LEFT':
#                 twist.linear.x  = float(self.forward_speed * 0.4)
#                 twist.angular.z = float(self.max_angular_speed)
#                 self.turn_detected = True
#                 self.turn_direction = 'LEFT'
#                 cv2.putText(outputIm, 'Shape: TURN LEFT', (10, outputIm.shape[0] - 20),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
#             elif road_shape == 'TURN_RIGHT':
#                 twist.linear.x  = float(self.forward_speed * 0.4)
#                 twist.angular.z = float(-self.max_angular_speed)
#                 self.turn_detected = True
#                 self.turn_direction = 'RIGHT'
#                 cv2.putText(outputIm, 'Shape: TURN RIGHT', (10, outputIm.shape[0] - 20),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
#             elif road_shape == 'INTERSECTION':
#                 twist.linear.x = float(self.forward_speed * 0.5)
#                 cv2.putText(outputIm, 'Shape: INTERSECTION – going forward', (10, outputIm.shape[0] - 20),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
#             elif road_shape == 'DEAD_END':
#                 cv2.putText(outputIm, f'DEAD END – confirming ({self.dead_end_confirm_count}/{self.dead_end_confirm_thresh})',
#                            (10, outputIm.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
#             else:
#                 cv2.putText(outputIm, 'No lanes detected', (10, outputIm.shape[0] - 20),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
#             self.cmd_publisher.publish(twist)

#         # Display the resulting frame
#         cv2.imshow('Lane Detection - Astra Pro Plus', outputIm)
        
#         # Write to video if recording
#         if self.recording and self.videoOut is not None:
#             self.videoOut.write(outputIm)
        
#         # Keyboard controls
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             self.get_logger().info("Quitting...")
#             self.cleanup()
#             rclpy.shutdown()
#         elif key == ord('s'):
#             if not self.recording:
#                 # Start recording
#                 self.videoOut = cv2.VideoWriter('output_astra.avi', self.fourcc, 20.0, 
#                                           (outputIm.shape[1], outputIm.shape[0]))
#                 self.recording = True
#                 self.get_logger().info("Recording started...")
#             else:
#                 # Stop recording
#                 self.recording = False
#                 if self.videoOut is not None:
#                     self.videoOut.release()
#                     self.videoOut = None
#                 self.get_logger().info("Recording stopped. Saved to output_astra.avi")
#         elif key == ord('w'):
#             # Widen ROI
#             self.roi_top_width = min(0.45, self.roi_top_width + 0.02)
#             self.get_logger().info(f"ROI Width: {self.roi_top_width:.2f}")
#         elif key == ord('n'):
#             # Narrow ROI
#             self.roi_top_width = max(0.05, self.roi_top_width - 0.02)
#             self.get_logger().info(f"ROI Width: {self.roi_top_width:.2f}")
#         elif key == ord('h'):
#             # Raise ROI height (move detection zone up)
#             self.roi_height = max(0.3, self.roi_height - 0.02)
#             self.get_logger().info(f"ROI Height: {self.roi_height:.2f}")
#         elif key == ord('l'):
#             # Lower ROI height (move detection zone down)
#             self.roi_height = min(0.8, self.roi_height + 0.02)
#             self.get_logger().info(f"ROI Height: {self.roi_height:.2f}")
#         elif key == ord('r'):
#             # Toggle ROI visualization
#             self.show_roi = not self.show_roi
#             self.get_logger().info(f"ROI Visualization: {'ON' if self.show_roi else 'OFF'}")
    
#     def cleanup(self):
#         """Clean up resources"""
#         stop = Twist()
#         self.cmd_publisher.publish(stop)
#         if self.videoOut is not None:
#             self.videoOut.release()
#         cv2.destroyAllWindows()
#         self.get_logger().info('Cleanup complete!')

#     def classify_road_shape(self, lines, imshape):
#         """
#         Detect structural junction shapes from Hough lines.

#         Patterns (camera looks forward, image bottom = closest to rover):

#           _|   right wall/horizontal, left diagonal open  → TURN_LEFT
#           |_   left wall/horizontal, right diagonal open  → TURN_RIGHT
#           =    horizontal + both lane diagonals present   → INTERSECTION
#           =    horizontal + NO diagonals                  → DEAD_END  (→ 360° spin)

#         Returns: 'STRAIGHT' | 'TURN_LEFT' | 'TURN_RIGHT' | 'INTERSECTION' | 'DEAD_END'
#         """
#         if lines is None or len(lines) == 0:
#             return 'STRAIGHT'

#         h, w = imshape[:2]
#         mid_x = w / 2.0

#         has_horiz      = False   # near-horizontal line   → cross-road / wall edge
#         has_diag_left  = False   # diagonal in left  half → left  lane boundary
#         has_diag_right = False   # diagonal in right half → right lane boundary
#         has_vert_left  = False   # near-vertical on left  → |_ left wall
#         has_vert_right = False   # near-vertical on right → _| right wall

#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
#                 if length < 25:
#                     continue
#                 dx = x2 - x1
#                 abs_slope = abs((y2 - y1) / dx) if dx != 0 else 999.0
#                 cx = (x1 + x2) / 2.0

#                 if abs_slope < 0.36:        # angle < ~20° → near-horizontal
#                     has_horiz = True
#                 elif abs_slope > 2.75:      # angle > ~70° → near-vertical
#                     if cx < mid_x:
#                         has_vert_left = True
#                     else:
#                         has_vert_right = True
#                 else:                       # 20–70° diagonal → lane line
#                     if cx < mid_x:
#                         has_diag_left = True
#                     else:
#                         has_diag_right = True

#         # DEAD_END: horizontal blocker present, no diagonal lane lines at all
#         if has_horiz and not has_diag_left and not has_diag_right:
#             return 'DEAD_END'

#         # INTERSECTION: horizontal + both lane diagonals still visible
#         if has_horiz and has_diag_left and has_diag_right:
#             return 'INTERSECTION'

#         # TURN_LEFT (_|): right side blocked, left diagonal open
#         if (has_horiz or has_vert_right) and not has_diag_right and has_diag_left:
#             return 'TURN_LEFT'

#         # TURN_RIGHT (|_): left side blocked, right diagonal open
#         if (has_horiz or has_vert_left) and not has_diag_left and has_diag_right:
#             return 'TURN_RIGHT'

#         return 'STRAIGHT'


# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         lane_detector = LaneDetectorNode()
#         rclpy.spin(lane_detector)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             lane_detector.cleanup()
#             lane_detector.destroy_node()
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()

"""
Real-time Lane Detection from Astra Pro Plus Depth Camera
Uses ROS2 to subscribe to camera feed
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_astra')
        
        # Create CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Initialize previous line positions for smoothing
        self.x1Prev = None
        self.x2Prev = None
        self.y1Prev = None
        self.y2Prev = None
        self.x1NPrev = None
        self.x2NPrev = None
        self.y1NPrev = None
        
        self.frameNum = 0
        self.recording = False
        self.videoOut = None
        self.fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        
        # Turn state (used by the visualisation overlay)
        self.turn_detected = False
        self.turn_direction = None  # 'LEFT' or 'RIGHT'

        # Autonomous navigation parameters
        self.forward_speed    = 0.25   # m/s nominal forward speed
        self.max_angular_speed = 0.6   # rad/s maximum turning rate
        # Normalised lateral error (0-1) that classifies motion as a turn
        self.turn_threshold   = 0.15
        # Look-ahead fraction: evaluate lane centre this far ahead (fraction of
        # frame height, measured from the bottom = closest to rover).
        # 0.20 = ~20 % of frame height → reacts only when the curve is close.
        self.look_ahead_fraction = 0.20

        # ── Turn-delay state machine ───────────────────────────────────────────
        # Prevents premature turning when a junction is spotted from a distance.
        # Strategy:
        #   1. TURN_LEFT / TURN_RIGHT detected while lanes are still visible
        #      → remember the direction, keep driving STRAIGHT.
        #   2. Lanes disappear (no-lane-detected) while TURN_PENDING
        #      → now the bot is physically AT the junction → execute buffered turn.
        #   3. DEAD_END confirmed for N consecutive frames
        #      → spin 360° to find a new path.
        #
        # States: DRIVE_STRAIGHT | TURN_PENDING | EXECUTING_TURN | DEAD_END_SPIN
        self.nav_state          = 'DRIVE_STRAIGHT'
        self.pending_turn_dir   = None   # 'LEFT' or 'RIGHT' – seen, not yet executed
        self.executing_turn_dir = None   # 'LEFT' or 'RIGHT' – currently being executed

        # Dead-end / 360° spin state
        self.dead_end_confirm_count  = 0
        self.dead_end_confirm_thresh = 10   # consecutive DEAD_END frames before committing
        self.spin_frames_remaining   = 0
        self.spin_frames_total       = 240  # ~12 s @ 20 fps → ~360° @ 0.6 rad/s

        # /cmd_vel publisher
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Adjustable ROI parameters
        self.roi_top_width = 0.19  # Width at top (calibrated)
        self.roi_height = 0.20  # Height position (calibrated)
        self.show_roi = True  # Toggle ROI visualization

        # ── Outer-boundary zone for shape classification ───────────────────────
        # When the camera is low, near-field lines (bottom of frame) overwhelm
        # the shape classifier.  We restrict shape detection to the OUTER BOUNDARY
        # zone — lines whose average y is in the UPPER portion of the frame
        # (= farthest from the rover).
        #
        # outer_zone_frac = fraction of full frame height below which a line's
        # centre-point must fall to be considered in shape detection.
        # E.g. 0.75 → only lines in the top 75 % of the image are classified.
        # Raise  with '+' key  (include more of the frame)
        # Lower  with '-' key  (restrict to far-field only)
        self.outer_zone_frac = 0.75   # calibrated: top 75 % of frame height

        # ── Debug visualization ────────────────────────────────────────────────
        # Press 'd' to toggle a second window that shows every Hough line
        # color-coded by its shape-classifier category:
        #   BLUE    – near-horizontal  (wall / cross-road edge)
        #   GREEN   – diagonal left    (left  lane outer boundary)
        #   RED     – diagonal right   (right lane outer boundary)
        #   MAGENTA – near-vertical left
        #   CYAN    – near-vertical right
        #   GREY    – excluded (outside outer-boundary zone or too short)
        self.show_shape_debug = False
        
        # Subscribe to color image topic from Astra Pro Plus
        # Use BEST_EFFORT QoS to match the camera driver's publisher profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            sensor_qos)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Lane Detection Node for Astra Pro Plus Started!')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribing to /camera/color/image_raw')
        self.get_logger().info('Controls:')
        self.get_logger().info('  q - Quit')
        self.get_logger().info('  s - Start/stop recording')
        self.get_logger().info('  w - Widen ROI | n - Narrow ROI')
        self.get_logger().info('  h - Raise ROI height | l - Lower ROI height')
        self.get_logger().info('  r - Toggle ROI visualization')
        self.get_logger().info('  d - Toggle shape-debug window')
        self.get_logger().info('  + - Expand outer-boundary zone | - - Shrink outer-boundary zone')
        self.get_logger().info('='*60)
    
    def image_callback(self, msg):
        """Callback function that processes incoming images from the camera"""
        try:
            # Convert ROS Image message to OpenCV format
            im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return
        
        self.frameNum += 1
        nav_published = False
        if self.frameNum % 30 == 0:
            self.get_logger().info(f'Processing frame {self.frameNum}...')
        
        imshape = im.shape
        
        # -------------GREYSCALE IMAGE---------------
        grayIm = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        
        #------------GAUSSIAN SMOOTHING-----------------
        kernel_size = 9
        smoothedIm = cv2.GaussianBlur(grayIm, (kernel_size, kernel_size), 0)
        
        #-------------EDGE DETECTION---------------------
        minVal = 60
        maxVal = 150
        edgesIm = cv2.Canny(smoothedIm, minVal, maxVal)
        
        #-------------------------CREATE MASK--------------------------------
        # Calculate ROI vertices based on adjustable parameters
        center_x = imshape[1] / 2
        top_left_x = int(center_x - (imshape[1] * self.roi_top_width))
        top_right_x = int(center_x + (imshape[1] * self.roi_top_width))
        top_y = int(imshape[0] * self.roi_height)
        
        # Trapezoid ROI: wider at bottom, narrower at top
        vertices = np.array([[(0, imshape[0]),
                              (top_left_x, top_y), 
                              (top_right_x, top_y), 
                              (imshape[1], imshape[0])]], dtype=np.int32)
        
        mask = np.zeros_like(edgesIm)   
        color = 255
        cv2.fillPoly(mask, vertices, color)
        
        #----------------------APPLY MASK TO IMAGE-------------------------------
        maskedIm = cv2.bitwise_and(edgesIm, mask)
        
        #-----------------------HOUGH LINES------------------------------------
        rho = 2
        theta = np.pi/180
        threshold = 45
        min_line_len = 40
        max_line_gap = 100
        lines = cv2.HoughLinesP(maskedIm, rho, theta, threshold, np.array([]), 
                                    minLineLength=min_line_len, maxLineGap=max_line_gap)
        
        # Classify road shape from raw structural line features.
        # Only lines in the OUTER BOUNDARY ZONE (top outer_zone_frac of the frame)
        # are considered — this prevents near-field lines from masking real junctions.
        outer_zone_y = imshape[0] * self.outer_zone_frac
        road_shape, shape_debug = self.classify_road_shape(lines, imshape, outer_zone_y)

        # ── Shape debug window ────────────────────────────────────────────────
        if self.show_shape_debug and lines is not None:
            self._draw_shape_debug(im, shape_debug, outer_zone_y, road_shape)

        # ── Dead-end frame confirmation (run every frame regardless of lane quality) ──
        if road_shape == 'DEAD_END':
            self.dead_end_confirm_count += 1
        else:
            self.dead_end_confirm_count = 0

        if (self.dead_end_confirm_count >= self.dead_end_confirm_thresh
                and self.nav_state != 'DEAD_END_SPIN'):
            self.nav_state             = 'DEAD_END_SPIN'
            self.spin_frames_remaining = self.spin_frames_total
            self.dead_end_confirm_count = 0
            self.get_logger().info('DEAD END confirmed – starting 360° spin')

        # Default output image
        outputIm = im.copy()

        # ── HIGH PRIORITY: 360° dead-end spin ──────────────────────────────────
        if self.nav_state == 'DEAD_END_SPIN':
            self.spin_frames_remaining -= 1
            spin_twist = Twist()
            spin_twist.angular.z = float(self.max_angular_speed)
            self.cmd_publisher.publish(spin_twist)
            nav_published = True
            if self.spin_frames_remaining <= 0:
                self.nav_state          = 'DRIVE_STRAIGHT'
                self.pending_turn_dir   = None
                self.executing_turn_dir = None
                self.get_logger().info('360° spin complete – resuming lane following')
            pct = int(100 * (1 - self.spin_frames_remaining / max(1, self.spin_frames_total)))
            cv2.putText(outputIm, f'DEAD END – SPINNING 360 ({pct}%)',
                       (10, imshape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        # Check if we got more than 1 line
        if lines is not None and len(lines) > 2:
            
            #-----------------------Separate Lines Into Positive/Negative Slope--------------------------
            slopePositiveLines = []
            slopeNegativeLines = []
            yValues = []
            
            addedPos = False
            addedNeg = False
            for currentLine in lines:   
                for x1,y1,x2,y2 in currentLine:
                    lineLength = ((x2-x1)**2 + (y2-y1)**2)**.5
                    if lineLength > 30:
                        if x2 != x1:
                            slope = (y2-y1)/(x2-x1)
                            if slope > 0: 
                                tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
                                ang = np.arctan(tanTheta)*180/np.pi
                                if abs(ang) < 85 and abs(ang) > 20:
                                    slopeNegativeLines.append([x1,y1,x2,y2,-slope])
                                    yValues.append(y1)
                                    yValues.append(y2)
                                    addedPos = True
                            if slope < 0:
                                tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
                                ang = np.arctan(tanTheta)*180/np.pi
                                if abs(ang) < 85 and abs(ang) > 20:
                                    slopePositiveLines.append([x1,y1,x2,y2,-slope])
                                    yValues.append(y1)
                                    yValues.append(y2)
                                    addedNeg = True
                   
            # If we didn't get any positive lines         
            if not addedPos:
                for currentLine in lines:
                    for x1,y1,x2,y2 in currentLine:
                        if x2 != x1:
                            slope = (y2-y1)/(x2-x1)
                            if slope > 0:
                                tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
                                ang = np.arctan(tanTheta)*180/np.pi
                                if abs(ang) < 80 and abs(ang) > 15:
                                    slopeNegativeLines.append([x1,y1,x2,y2,-slope])
                                    yValues.append(y1)
                                    yValues.append(y2)
            
            # If we didn't get any negative lines
            if not addedNeg:
                for currentLine in lines:
                    for x1,y1,x2,y2 in currentLine:
                        if x2 != x1:
                            slope = (y2-y1)/(x2-x1)
                            if slope < 0:
                                tanTheta = np.tan((abs(y2-y1))/(abs(x2-x1)))
                                ang = np.arctan(tanTheta)*180/np.pi
                                if abs(ang) < 85 and abs(ang) > 15:
                                    slopePositiveLines.append([x1,y1,x2,y2,-slope])           
                                    yValues.append(y1)
                                    yValues.append(y2)
            
            #------------------------Get Positive/Negative Slope Averages-----------------------------------
            if len(slopePositiveLines) > 0 and len(slopeNegativeLines) > 0:
                positiveSlopes = np.asarray(slopePositiveLines)[:,4]
                posSlopeMedian = np.median(positiveSlopes)
                posSlopesGood = []
                for slope in positiveSlopes:
                    if abs(slope-posSlopeMedian) < posSlopeMedian*.2:
                        posSlopesGood.append(slope)
                
                if len(posSlopesGood) > 0:
                    posSlopeMean = np.mean(np.asarray(posSlopesGood))
                else:
                    posSlopeMean = posSlopeMedian
                        
                negativeSlopes = np.asarray(slopeNegativeLines)[:,4]
                negSlopeMedian = np.median(negativeSlopes)
                negSlopesGood = []
                for slope in negativeSlopes:
                    if abs(slope-negSlopeMedian) < .9:
                        negSlopesGood.append(slope)
                
                if len(negSlopesGood) > 0:
                    negSlopeMean = np.mean(np.asarray(negSlopesGood))
                else:
                    negSlopeMean = negSlopeMedian
                    
                #--------------------------Get Average x Coord When y Coord Of Line = 0----------------------------
                # Positive Lines
                xInterceptPos = []
                for line in slopePositiveLines:
                        x1 = line[0]
                        y1 = im.shape[0]-line[1]
                        slope = line[4]
                        yIntercept = y1-slope*x1
                        xIntercept = -yIntercept/slope
                        if xIntercept == xIntercept:
                            xInterceptPos.append(xIntercept)
                        
                if len(xInterceptPos) > 0:
                    xIntPosMed = np.median(xInterceptPos)
                    xIntPosGood = []
                    for line in slopePositiveLines:
                            x1 = line[0]
                            y1 = im.shape[0]-line[1]
                            slope = line[4]
                            yIntercept = y1-slope*x1
                            xIntercept = -yIntercept/slope
                            if abs(xIntercept-xIntPosMed) < .35*xIntPosMed:
                                xIntPosGood.append(xIntercept)
                                
                    if len(xIntPosGood) > 0:
                        xInterceptPosMean = np.mean(np.asarray(xIntPosGood))
                    else:
                        xInterceptPosMean = xIntPosMed
                
                    # Negative Lines 
                    xInterceptNeg = []
                    for line in slopeNegativeLines:
                        x1 = line[0]
                        y1 = im.shape[0]-line[1]
                        slope = line[4]
                        yIntercept = y1-slope*x1
                        xIntercept = -yIntercept/slope
                        if xIntercept == xIntercept:
                                xInterceptNeg.append(xIntercept)
                            
                    if len(xInterceptNeg) > 0:
                        xIntNegMed = np.median(xInterceptNeg)
                        xIntNegGood = []
                        for line in slopeNegativeLines:
                            x1 = line[0]
                            y1 = im.shape[0]-line[1]
                            slope = line[4]
                            yIntercept = y1-slope*x1
                            xIntercept = -yIntercept/slope
                            if abs(xIntercept-xIntNegMed)< .35*xIntNegMed: 
                                    xIntNegGood.append(xIntercept)
                            
                        if len(xIntNegGood) > 0:
                            xInterceptNegMean = np.mean(np.asarray(xIntNegGood))
                        else:
                            xInterceptNegMean = xIntNegMed
                    
                        # ----------------------PLOT LANE LINES------------------------------
                        colorLines = im.copy()
                        
                        # Positive Slope Line
                        slope = posSlopeMean
                        x1 = xInterceptPosMean
                        y1 = 0
                        y2 = imshape[0] - (imshape[0]-imshape[0]*.35)
                        x2 = (y2-y1)/slope + x1
                        
                        x1 = int(round(x1))
                        x2 = int(round(x2))
                        y1 = int(round(y1))
                        y2 = int(round(y2))
                        
                        # Smoothing filter
                        jumpThresh = 50
                        if self.x1Prev is not None:
                            if abs(x1-self.x1Prev) > 3 and abs(x1-self.x1Prev) < jumpThresh:
                                x1 = self.x1Prev + int(np.sign(x1-self.x1Prev)*1)
                                
                            if abs(x2-self.x2Prev) > 3 and abs(x2-self.x2Prev) < jumpThresh:
                                x2 = self.x2Prev + int(np.sign(x2-self.x2Prev)*1)
                        
                        cv2.line(colorLines,(x1,im.shape[0]-y1),(x2,imshape[0]-y2),(0,255,0),4)
                        
                        # Negative Slope Line
                        slope = negSlopeMean
                        x1N = xInterceptNegMean
                        y1N = 0
                        x2N = (y2-y1N)/slope + x1N
                        
                        x1N = int(round(x1N))
                        x2N = int(round(x2N))
                        y1N = int(round(y1N))
                        
                        if self.x1NPrev is not None:
                            if abs(x1N-self.x1NPrev) > 3 and abs(x1N-self.x1NPrev) < jumpThresh:
                                x1N = self.x1NPrev + int(np.sign(x1N-self.x1NPrev)*1)
                                
                            if abs(x2N-self.x2NPrev) > 3 and abs(x2N-self.x2NPrev) < jumpThresh:
                                x2N = self.x2NPrev + int(np.sign(x2N-self.x2NPrev)*1)
                        
                        cv2.line(colorLines,(x1N,im.shape[0]-y1N),(x2N,imshape[0]-y2),(0,255,0),4)
                        
                        # Store previous values
                        self.x1Prev = x1
                        self.x2Prev = x2
                        self.y1Prev = y1
                        self.y2Prev = y2
                        self.x1NPrev = x1N
                        self.x2NPrev = x2N
                        self.y1NPrev = y1N
                        
                        #-------------------------------------STATE-MACHINE NAVIGATION-----------------------------------------
                        # The state machine prevents premature turning:
                        #   • When TURN_LEFT/RIGHT is detected while lanes are still visible
                        #     → store the direction, keep going straight.
                        #   • Lanes will disappear once the bot is AT the junction
                        #     → the fallback block below will execute the buffered turn.
                        #   • DEAD_END spin is handled at the top of the callback.

                        # Safe defaults so display code always has valid values
                        error = 0.0
                        twist = Twist()

                        if self.nav_state == 'DEAD_END_SPIN':
                            # Spin command already published above; just skip nav here.
                            pass

                        elif road_shape in ('TURN_LEFT', 'TURN_RIGHT'):
                            # Junction visible but lanes still present → approaching, not there yet.
                            new_dir = 'LEFT' if road_shape == 'TURN_LEFT' else 'RIGHT'
                            if self.nav_state in ('DRIVE_STRAIGHT', 'TURN_PENDING'):
                                if self.nav_state == 'DRIVE_STRAIGHT' or self.pending_turn_dir != new_dir:
                                    self.get_logger().info(
                                        f'[NAV] {road_shape} detected – buffering turn {new_dir}, going straight')
                                self.nav_state        = 'TURN_PENDING'
                                self.pending_turn_dir = new_dir
                            # Keep going straight with mild look-ahead correction
                            look_ahead_px = imshape[0] * self.look_ahead_fraction
                            x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
                            x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
                            lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
                            error = (lane_center_ahead - imshape[1] / 2.0) / (imshape[1] / 2.0)
                            error = max(-1.0, min(1.0, error))
                            twist = Twist()
                            twist.linear.x  = float(self.forward_speed)
                            twist.angular.z = float(-self.max_angular_speed * 0.4 * error)
                            self.turn_detected  = True
                            self.turn_direction = new_dir
                            self.cmd_publisher.publish(twist)
                            nav_published = True

                        elif self.nav_state == 'EXECUTING_TURN':
                            # Turn in progress; stop only when lanes are straight again
                            turn_dir = self.executing_turn_dir
                            if road_shape == 'STRAIGHT':
                                self.get_logger().info(
                                    f'[NAV] Turn {turn_dir} complete – resuming straight')
                                self.nav_state          = 'DRIVE_STRAIGHT'
                                self.executing_turn_dir = None
                                # Use look-ahead for immediate straight correction
                                look_ahead_px = imshape[0] * self.look_ahead_fraction
                                x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
                                x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
                                lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
                                error = (lane_center_ahead - imshape[1] / 2.0) / (imshape[1] / 2.0)
                                error = max(-1.0, min(1.0, error))
                                twist = Twist()
                                twist.linear.x  = float(self.forward_speed * (1.0 - 0.5 * abs(error)))
                                twist.angular.z = float(-self.max_angular_speed * error)
                                self.turn_detected  = False
                                self.turn_direction = None
                            else:
                                # Still completing the turn
                                twist = Twist()
                                twist.linear.x  = float(self.forward_speed * 0.3)
                                twist.angular.z = float(
                                    self.max_angular_speed if turn_dir == 'LEFT' else -self.max_angular_speed)
                                self.turn_detected  = True
                                self.turn_direction = turn_dir
                                error = 0.0
                            self.cmd_publisher.publish(twist)
                            nav_published = True

                        elif road_shape == 'INTERSECTION':
                            # T-junction / crossroad → go forward slowly, gentle centering
                            look_ahead_px = imshape[0] * self.look_ahead_fraction
                            x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
                            x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
                            lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
                            error = (lane_center_ahead - imshape[1] / 2.0) / (imshape[1] / 2.0)
                            error = max(-1.0, min(1.0, error))
                            twist = Twist()
                            twist.linear.x  = float(self.forward_speed * 0.5)
                            twist.angular.z = float(-self.max_angular_speed * 0.3 * error)
                            self.turn_detected  = False
                            self.turn_direction = None
                            self.cmd_publisher.publish(twist)
                            nav_published = True

                        else:
                            # STRAIGHT / DRIVE_STRAIGHT – proportional look-ahead controller
                            look_ahead_px = imshape[0] * self.look_ahead_fraction
                            x_left_ahead  = xInterceptPosMean + look_ahead_px / posSlopeMean
                            x_right_ahead = xInterceptNegMean + look_ahead_px / negSlopeMean
                            lane_center_ahead = (x_left_ahead + x_right_ahead) / 2.0
                            frame_center_f    = imshape[1] / 2.0
                            error = (lane_center_ahead - frame_center_f) / frame_center_f
                            error = max(-1.0, min(1.0, error))
                            twist = Twist()
                            twist.linear.x  = float(self.forward_speed * (1.0 - 0.5 * abs(error)))
                            twist.angular.z = float(-self.max_angular_speed * error)
                            if abs(error) > self.turn_threshold:
                                self.turn_detected  = True
                                self.turn_direction = 'RIGHT' if error > 0 else 'LEFT'
                            else:
                                self.turn_detected  = False
                                self.turn_direction = None
                            self.cmd_publisher.publish(twist)
                            nav_published = True
                        
                        #-------------------------------------Blend Image-----------------------------------------
                        laneFill = im.copy()
                        vertices = np.array([[(x1,im.shape[0]-y1),(x2,im.shape[0]-y2),  (x2N,imshape[0]-y2),
                                                              (x1N,imshape[0]-y1N)]], dtype=np.int32)
                        color = [241,255,1]
                        cv2.fillPoly(laneFill, vertices, color)
                        opacity = .25
                        blendedIm = cv2.addWeighted(laneFill,opacity,im,1-opacity,0,im)
                        cv2.line(blendedIm,(x1,im.shape[0]-y1),(x2,imshape[0]-y2),(0,255,0),4)
                        cv2.line(blendedIm,(x1N,im.shape[0]-y1N),(x2N,imshape[0]-y2),(0,255,0),4)
                        
                        #-------------------------------------TURN VISUALIZATION-----------------------------------------
                        if self.turn_detected and self.turn_direction:
                            src = "SHAPE" if road_shape != 'STRAIGHT' else "LANE"
                            warning_text = f"[{src}] TURN {self.turn_direction}!"
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            font_scale = 1.2
                            thickness = 3
                            
                            # Get text size for background box
                            (text_width, text_height), baseline = cv2.getTextSize(warning_text, font, font_scale, thickness)
                            
                            # Position at top center
                            text_x = (imshape[1] - text_width) // 2
                            text_y = 60
                            
                            # Draw red background box
                            box_coords = ((text_x - 10, text_y - text_height - 10), 
                                        (text_x + text_width + 10, text_y + baseline + 10))
                            cv2.rectangle(blendedIm, box_coords[0], box_coords[1], (0, 0, 255), -1)
                            
                            # Draw white text
                            cv2.putText(blendedIm, warning_text, (text_x, text_y), 
                                      font, font_scale, (255, 255, 255), thickness)
                            
                            # Draw direction arrow
                            arrow_y = imshape[0] // 2
                            center_x = imshape[1] // 2
                            arrow_length = 80
                            
                            if self.turn_direction == 'LEFT':
                                # Left arrow
                                start_point = (center_x + 30, arrow_y)
                                end_point = (center_x - arrow_length + 30, arrow_y)
                            else:
                                # Right arrow
                                start_point = (center_x - 30, arrow_y)
                                end_point = (center_x + arrow_length - 30, arrow_y)
                            
                            # Draw thick yellow arrow
                            cv2.arrowedLine(blendedIm, start_point, end_point, 
                                          (0, 255, 255), 8, tipLength=0.4)
                            
                            # Log turn detection
                            if self.frameNum % 30 == 0:
                                self.get_logger().info(f'↩ TURNING {self.turn_direction}  shape={road_shape}  error={error:+.2f}  ang={twist.angular.z:+.2f}')
                        
                        # Display navigation info
                        nav_info = (f"Shape:{road_shape}  Err:{error:+.2f}  "
                                    f"Lin:{twist.linear.x:.2f}  Ang:{twist.angular.z:+.2f}")
                        cv2.putText(blendedIm, nav_info, (10, imshape[0] - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                        
                        outputIm = blendedIm
        
        #-------------------------------------ROI VISUALIZATION-----------------------------------------
        if self.show_roi:
            # Draw ROI overlay on output image
            roi_overlay = outputIm.copy()
            
            # Calculate ROI vertices (same as mask)
            center_x = imshape[1] / 2
            top_left_x = int(center_x - (imshape[1] * self.roi_top_width))
            top_right_x = int(center_x + (imshape[1] * self.roi_top_width))
            top_y = int(imshape[0] * self.roi_height)
            
            roi_vertices = np.array([[(0, imshape[0]),
                                      (top_left_x, top_y), 
                                      (top_right_x, top_y), 
                                      (imshape[1], imshape[0])]], dtype=np.int32)
            
            # Draw semi-transparent cyan overlay
            cv2.fillPoly(roi_overlay, roi_vertices, (255, 255, 0))
            outputIm = cv2.addWeighted(outputIm, 0.9, roi_overlay, 0.1, 0)
            
            # Draw ROI border lines
            cv2.polylines(outputIm, roi_vertices, True, (0, 255, 255), 2)
            
            # Draw corner markers
            marker_size = 15
            # Top-left corner
            cv2.line(outputIm, (top_left_x, top_y), (top_left_x + marker_size, top_y), (0, 255, 255), 3)
            cv2.line(outputIm, (top_left_x, top_y), (top_left_x, top_y + marker_size), (0, 255, 255), 3)
            # Top-right corner
            cv2.line(outputIm, (top_right_x, top_y), (top_right_x - marker_size, top_y), (0, 255, 255), 3)
            cv2.line(outputIm, (top_right_x, top_y), (top_right_x, top_y + marker_size), (0, 255, 255), 3)
            
            # Display ROI settings
            roi_info = f"ROI: Width={self.roi_top_width:.2f} Height={self.roi_height:.2f}"
            cv2.putText(outputIm, roi_info, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Fallback: lane-line geometry unavailable – drive by state machine alone
        if not nav_published:
            twist = Twist()

            if self.nav_state == 'DEAD_END_SPIN':
                # Already handled above; nothing to do here.
                pass

            elif self.nav_state == 'EXECUTING_TURN':
                # Continue the turn that was already started
                turn_dir = self.executing_turn_dir
                twist.linear.x  = float(self.forward_speed * 0.3)
                twist.angular.z = float(
                    self.max_angular_speed if turn_dir == 'LEFT' else -self.max_angular_speed)
                self.turn_detected  = True
                self.turn_direction = turn_dir
                cv2.putText(outputIm, f'EXECUTING TURN {turn_dir}',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                self.cmd_publisher.publish(twist)

            elif self.nav_state == 'TURN_PENDING' and self.pending_turn_dir is not None:
                # ★ KEY LOGIC: no lanes detected after buffering a turn
                #   → the bot is now AT the junction → execute the buffered turn
                self.executing_turn_dir = self.pending_turn_dir
                self.pending_turn_dir   = None
                self.nav_state          = 'EXECUTING_TURN'
                self.get_logger().info(
                    f'[NAV] No lanes – executing buffered turn {self.executing_turn_dir}')
                twist.linear.x  = float(self.forward_speed * 0.3)
                twist.angular.z = float(
                    self.max_angular_speed
                    if self.executing_turn_dir == 'LEFT' else -self.max_angular_speed)
                self.turn_detected  = True
                self.turn_direction = self.executing_turn_dir
                cv2.putText(outputIm, f'JUNCTION – TURNING {self.executing_turn_dir}',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                self.cmd_publisher.publish(twist)

            elif road_shape == 'DEAD_END':
                # Still accumulating confirmation frames (spin not triggered yet)
                cv2.putText(outputIm,
                           f'DEAD END – confirming ({self.dead_end_confirm_count}/{self.dead_end_confirm_thresh})',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                # Slow forward while confirming to get closer / clearer view
                twist.linear.x = float(self.forward_speed * 0.3)
                self.cmd_publisher.publish(twist)

            elif road_shape in ('TURN_LEFT', 'TURN_RIGHT'):
                # Shape visible but no lane geometry → approach, buffer the turn
                new_dir = 'LEFT' if road_shape == 'TURN_LEFT' else 'RIGHT'
                self.nav_state        = 'TURN_PENDING'
                self.pending_turn_dir = new_dir
                twist.linear.x = float(self.forward_speed * 0.5)
                cv2.putText(outputIm, f'APPROACHING {new_dir} TURN (no lanes)',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                self.cmd_publisher.publish(twist)

            elif road_shape == 'INTERSECTION':
                twist.linear.x = float(self.forward_speed * 0.5)
                cv2.putText(outputIm, 'Shape: INTERSECTION – slow forward',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                self.cmd_publisher.publish(twist)

            else:
                # No lanes, no special shape → creep forward slowly
                twist.linear.x = float(self.forward_speed * 0.4)
                cv2.putText(outputIm, 'No lanes detected – creeping forward',
                           (10, outputIm.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
                self.cmd_publisher.publish(twist)

        # Display the resulting frame
        cv2.imshow('Lane Detection - Astra Pro Plus', outputIm)
        
        # Write to video if recording
        if self.recording and self.videoOut is not None:
            self.videoOut.write(outputIm)
        
        # Keyboard controls
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            self.cleanup()
            rclpy.shutdown()
        elif key == ord('s'):
            if not self.recording:
                # Start recording
                self.videoOut = cv2.VideoWriter('output_astra.avi', self.fourcc, 20.0, 
                                          (outputIm.shape[1], outputIm.shape[0]))
                self.recording = True
                self.get_logger().info("Recording started...")
            else:
                # Stop recording
                self.recording = False
                if self.videoOut is not None:
                    self.videoOut.release()
                    self.videoOut = None
                self.get_logger().info("Recording stopped. Saved to output_astra.avi")
        elif key == ord('w'):
            # Widen ROI
            self.roi_top_width = min(0.45, self.roi_top_width + 0.02)
            self.get_logger().info(f"ROI Width: {self.roi_top_width:.2f}")
        elif key == ord('n'):
            # Narrow ROI
            self.roi_top_width = max(0.05, self.roi_top_width - 0.02)
            self.get_logger().info(f"ROI Width: {self.roi_top_width:.2f}")
        elif key == ord('h'):
            # Raise ROI height (move detection zone up)
            self.roi_height = max(0.3, self.roi_height - 0.02)
            self.get_logger().info(f"ROI Height: {self.roi_height:.2f}")
        elif key == ord('l'):
            # Lower ROI height (move detection zone down)
            self.roi_height = min(0.8, self.roi_height + 0.02)
            self.get_logger().info(f"ROI Height: {self.roi_height:.2f}")
        elif key == ord('r'):
            # Toggle ROI visualization
            self.show_roi = not self.show_roi
            self.get_logger().info(f"ROI Visualization: {'ON' if self.show_roi else 'OFF'}")
        elif key == ord('d'):
            # Toggle shape-debug window
            self.show_shape_debug = not self.show_shape_debug
            if not self.show_shape_debug:
                cv2.destroyWindow('Shape Debug – Outer Boundary Lines')
            self.get_logger().info(f"Shape Debug: {'ON' if self.show_shape_debug else 'OFF'}")
        elif key == ord('+') or key == ord('='):
            # Expand outer-boundary zone (include more of the frame for shape detection)
            self.outer_zone_frac = min(1.0, self.outer_zone_frac + 0.02)
            self.get_logger().info(f"Outer zone frac: {self.outer_zone_frac:.2f}  (cutoff y={int(imshape[0]*self.outer_zone_frac)})")
        elif key == ord('-'):
            # Shrink outer-boundary zone (focus on far-field lines only)
            self.outer_zone_frac = max(0.20, self.outer_zone_frac - 0.02)
            self.get_logger().info(f"Outer zone frac: {self.outer_zone_frac:.2f}  (cutoff y={int(imshape[0]*self.outer_zone_frac)})")
    
    def _draw_shape_debug(self, im, debug_lines, outer_zone_y, road_shape):
        """
        Render a separate window showing every Hough line colour-coded by the
        category assigned by classify_road_shape.

        Colour key:
          BLUE    – near-horizontal  (wall / cross-road edge)
          GREEN   – diagonal-left    (outer left  lane boundary)
          RED     – diagonal-right   (outer right lane boundary)
          MAGENTA – near-vertical left
          CYAN    – near-vertical right
          GREY    – excluded (too short or below outer-boundary zone)

        A dashed cyan horizontal line marks the outer-boundary zone cutoff.
        """
        dbg = im.copy()
        h, w = dbg.shape[:2]

        colour_map = {
            'horiz':      (255,   0,   0),   # blue
            'diag_left':  (  0, 255,   0),   # green
            'diag_right': (  0,   0, 255),   # red
            'vert_left':  (255,   0, 255),   # magenta
            'vert_right': (  0, 255, 255),   # cyan
            'excluded':   (100, 100, 100),   # grey
        }
        thickness_map = {
            'horiz': 3, 'diag_left': 3, 'diag_right': 3,
            'vert_left': 3, 'vert_right': 3, 'excluded': 1,
        }

        for cat, segs in debug_lines.items():
            col = colour_map[cat]
            th  = thickness_map[cat]
            for (x1, y1, x2, y2) in segs:
                cv2.line(dbg, (x1, y1), (x2, y2), col, th)

        # Outer-boundary zone cutoff line (dashed cyan)
        zone_yi = int(outer_zone_y)
        for sx in range(0, w, 20):
            cv2.line(dbg, (sx, zone_yi), (min(sx + 12, w), zone_yi), (0, 220, 220), 2)
        cv2.putText(dbg, f'Outer zone cutoff (y < {zone_yi}  frac={self.outer_zone_frac:.2f})',
                    (10, zone_yi - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)

        # Mid-frame vertical divider (left / right half boundary for cx test)
        mid_x = w // 2
        cv2.line(dbg, (mid_x, 0), (mid_x, h), (180, 180, 0), 1)

        # Legend (top-left corner)
        legend = [
            ('HORIZ',      colour_map['horiz'],      len(debug_lines['horiz'])),
            ('DIAG-LEFT',  colour_map['diag_left'],  len(debug_lines['diag_left'])),
            ('DIAG-RIGHT', colour_map['diag_right'], len(debug_lines['diag_right'])),
            ('VERT-LEFT',  colour_map['vert_left'],  len(debug_lines['vert_left'])),
            ('VERT-RIGHT', colour_map['vert_right'], len(debug_lines['vert_right'])),
            ('EXCLUDED',   colour_map['excluded'],   len(debug_lines['excluded'])),
        ]
        for i, (label, col, cnt) in enumerate(legend):
            y_pos = 20 + i * 22
            cv2.rectangle(dbg, (10, y_pos - 12), (26, y_pos + 2), col, -1)
            cv2.putText(dbg, f'{label} ({cnt})', (32, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        # Shape result banner at the bottom
        shape_col = {
            'STRAIGHT':     (200, 200, 200),
            'TURN_LEFT':    (0, 165, 255),
            'TURN_RIGHT':   (0, 165, 255),
            'INTERSECTION': (0, 255, 255),
            'DEAD_END':     (0, 0, 255),
        }.get(road_shape, (200, 200, 200))
        cv2.putText(dbg, f'SHAPE: {road_shape}',
                    (w // 2 - 80, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, shape_col, 2)

        cv2.imshow('Shape Debug – Outer Boundary Lines', dbg)

    def cleanup(self):
        """Clean up resources"""
        stop = Twist()
        self.cmd_publisher.publish(stop)
        if self.videoOut is not None:
            self.videoOut.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Cleanup complete!")

    def classify_road_shape(self, lines, imshape, outer_zone_y=None):
        """
        Detect structural junction shapes from Hough lines.

        Only lines whose centre-point y < outer_zone_y are considered for shape
        classification.  This focuses on the OUTER BOUNDARY of the lane (the far
        end, top of camera view) and prevents close near-field lines from masking
        real junction features when the camera is mounted low.

        If outer_zone_y is None (default), all lines are considered.

        Patterns (y increases downward; bottom = nearest to rover):

          |_  or  L          → TURN_LEFT
          _|  or  inverted-L → TURN_RIGHT
          = + both diagonals → INTERSECTION  (T-junction, one side open)
          = + no  diagonals  → DEAD_END      (all paths blocked)

        Returns: (shape_str, debug_dict)
          shape_str  – 'STRAIGHT' | 'TURN_LEFT' | 'TURN_RIGHT' |
                       'INTERSECTION' | 'DEAD_END'
          debug_dict – dict with keys 'horiz', 'diag_left', 'diag_right',
                       'vert_left', 'vert_right', 'excluded' each containing
                       a list of (x1,y1,x2,y2) tuples for visualization.
        """
        debug = {
            'horiz':      [],
            'diag_left':  [],
            'diag_right': [],
            'vert_left':  [],
            'vert_right': [],
            'excluded':   [],
        }

        if lines is None or len(lines) == 0:
            return 'STRAIGHT', debug

        h, w = imshape[:2]
        mid_x = w / 2.0
        zone_y = outer_zone_y if outer_zone_y is not None else float('inf')

        has_horiz      = False
        has_diag_left  = False
        has_diag_right = False
        has_vert_left  = False
        has_vert_right = False

        for line in lines:
            for x1, y1, x2, y2 in line:
                seg = (x1, y1, x2, y2)
                length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
                if length < 25:
                    debug['excluded'].append(seg)
                    continue

                cy = (y1 + y2) / 2.0          # line centre-point y
                if cy > zone_y:                # below outer-boundary zone
                    debug['excluded'].append(seg)
                    continue

                dx = x2 - x1
                abs_slope = abs((y2 - y1) / dx) if dx != 0 else 999.0
                cx = (x1 + x2) / 2.0

                if abs_slope < 0.36:           # < ~20° → near-horizontal
                    has_horiz = True
                    debug['horiz'].append(seg)
                elif abs_slope > 2.75:         # > ~70° → near-vertical
                    if cx < mid_x:
                        has_vert_left = True
                        debug['vert_left'].append(seg)
                    else:
                        has_vert_right = True
                        debug['vert_right'].append(seg)
                else:                          # 20–70° diagonal lane line
                    if cx < mid_x:
                        has_diag_left = True
                        debug['diag_left'].append(seg)
                    else:
                        has_diag_right = True
                        debug['diag_right'].append(seg)

        # ---- Classification rules ----

        # DEAD_END: front blocked, no diagonal escape on either side
        if has_horiz and not has_diag_left and not has_diag_right:
            return 'DEAD_END', debug

        # INTERSECTION: horizontal cross-road + at least one diagonal still open
        if has_horiz and (has_diag_left or has_diag_right):
            return 'INTERSECTION', debug

        # TURN RIGHT → _| shape: right diagonal absent, left or left-wall present
        if (has_horiz or has_vert_right) and not has_diag_right:
            if has_diag_left or has_vert_left:
                return 'TURN_RIGHT', debug

        # TURN LEFT → |_ shape: left diagonal absent, right or right-wall present
        if (has_horiz or has_vert_left) and not has_diag_left:
            if has_diag_right or has_vert_right:
                return 'TURN_LEFT', debug

        return 'STRAIGHT', debug


def main(args=None):
    rclpy.init(args=args)
    
    try:
        lane_detector = LaneDetectorNode()
        rclpy.spin(lane_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            lane_detector.cleanup()
            lane_detector.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

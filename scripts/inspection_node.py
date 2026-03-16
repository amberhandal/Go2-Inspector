#!/usr/bin/env python3
"""
Inspection Node for Go2 Robot

Periodically captures RGB+depth images from the Go2's RealSense camera,
sends them to the SAM 3 service for object detection, transforms detections
into the map frame, publishes RViz markers, and logs all detections to a
JSON file for cross-run comparison.

Features:
  - Live change detection against a baseline log (NEW/MOVED/UNCHANGED/MISSING)
  - Camera overlay with SAM 3 segmentation masks, bounding boxes, labels
  - 3D change markers in RViz/rtabmap
  - Post-hoc comparison still available via change_detector.py
"""

import base64
import json
import os
import time
import threading
from datetime import datetime
from typing import Optional

import numpy as np
import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image as RosImage, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs

from cv_bridge import CvBridge
import cv2


# Colors for different object types in RViz
LABEL_COLORS = {
    "fire extinguisher": ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),
    "exit sign":         ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
    "smoke detector":    ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
    "sprinkler":         ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9),
    "fire alarm":        ColorRGBA(r=1.0, g=0.0, b=0.5, a=0.9),
}
DEFAULT_COLOR = ColorRGBA(r=0.8, g=0.8, b=0.0, a=0.9)

# Change status colors for RViz markers (matches change_detector.py)
CHANGE_COLORS_RGBA = {
    'NEW':       ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9),
    'MISSING':   ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.4),
    'MOVED':     ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
    'UNCHANGED': ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.5),
}

# Change status colors for camera overlay (BGR for OpenCV)
CHANGE_COLORS_BGR = {
    'NEW':       (255, 0, 0),
    'MISSING':   (0, 0, 255),
    'MOVED':     (0, 128, 255),
    'UNCHANGED': (0, 200, 0),
}


def get_str(node, name):
    """Get string parameter - works on both Humble and Kilted."""
    p = node.get_parameter(name)
    try:
        return p.as_string()
    except AttributeError:
        return p.get_parameter_value().string_value


def get_dbl(node, name):
    """Get double parameter - works on both Humble and Kilted."""
    p = node.get_parameter(name)
    try:
        return p.as_double()
    except AttributeError:
        return p.get_parameter_value().double_value


def get_bool(node, name):
    """Get bool parameter - works on both Humble and Kilted.
    Handles string 'true'/'false' from launch file XML params.
    """
    p = node.get_parameter(name)
    try:
        return p.as_bool()
    except Exception:
        pass
    try:
        return p.get_parameter_value().bool_value
    except Exception:
        pass
    # Fall back: param may be a string "true"/"false" from launch XML
    val = str(p.value).lower().strip()
    return val in ('true', '1', 'yes')


def get_str_arr(node, name):
    """Get string array parameter - works on both Humble and Kilted."""
    p = node.get_parameter(name)
    try:
        return p.as_string_array()
    except AttributeError:
        return list(p.get_parameter_value().string_array_value)


class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_node')

        # Parameters
        self.declare_parameter('sam3_url', 'http://129.105.69.11:8001')
        self.declare_parameter('prompts', ['fire extinguisher', 'exit sign'])
        self.declare_parameter('detection_interval', 3.0)
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('run_id', '')
        self.declare_parameter('log_dir', os.path.expanduser('~/inspection_logs'))
        self.declare_parameter('rgb_topic', '/camera/color/image_restamped')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_restamped')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info_restamped')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        # Live change detection parameters
        self.declare_parameter('baseline_log', '')
        self.declare_parameter('match_distance', 0.8)
        self.declare_parameter('unchanged_threshold', 1.0)

        # Overlay parameters
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('include_masks', True)

        self.sam3_url = get_str(self, 'sam3_url')
        try:
            self.prompts = get_str_arr(self, 'prompts')
        except Exception:
            raw = get_str(self, 'prompts')
            self.prompts = json.loads(raw)
        self.detection_interval = get_dbl(self, 'detection_interval')
        self.score_threshold = get_dbl(self, 'score_threshold')
        self.dedup_distance = get_dbl(self, 'dedup_distance')
        self.run_id = get_str(self, 'run_id')
        self.log_dir = get_str(self, 'log_dir')
        self.rgb_topic = get_str(self, 'rgb_topic')
        self.depth_topic = get_str(self, 'depth_topic')
        self.camera_info_topic = get_str(self, 'camera_info_topic')
        self.camera_frame = get_str(self, 'camera_frame')
        self.match_distance = get_dbl(self, 'match_distance')
        self.unchanged_threshold = get_dbl(self, 'unchanged_threshold')
        self.publish_overlay = get_bool(self, 'publish_overlay')
        self.include_masks = get_bool(self, 'include_masks')

        # Generate run_id if not provided
        if not self.run_id:
            self.run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")

        # Ensure log directory exists
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, f"{self.run_id}.json")

        # State
        self.bridge = CvBridge()
        self.latest_rgb: Optional[RosImage] = None
        self.latest_depth: Optional[RosImage] = None
        self.camera_info: Optional[CameraInfo] = None
        self.detections = []
        self.detection_id = 0

        # Live change detection state
        self.baseline_detections = []
        self.baseline_matched = {}
        self.has_baseline = False
        self.detection_changes = {}  # detection_id -> change type

        # SAM 3 call state (prevent overlapping requests)
        self._sam3_busy = False
        self._pending_overlay = None  # Store overlay msg from bg thread

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for sensor data
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        # Subscribers
        self.rgb_sub = self.create_subscription(
            RosImage, self.rgb_topic, self.rgb_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            RosImage, self.depth_topic, self.depth_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, sensor_qos)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/inspection_markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, '/inspection_labels', 10)

        # Change detection publishers (same topics as change_detector.py)
        self.change_marker_pub = self.create_publisher(
            MarkerArray, '/inspection_changes', 10)
        self.change_label_pub = self.create_publisher(
            MarkerArray, '/inspection_change_labels', 10)

        # Overlay image publisher (RELIABLE for RViz compatibility)
        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(
                RosImage, '/inspection/annotated_image', 10)

        # Timer for periodic detection (SAM 3 calls)
        self.timer = self.create_timer(self.detection_interval, self.detection_callback)

        # Timer for continuously publishing markers
        self.marker_timer = self.create_timer(1.0, self.publish_all_markers)


        # Load baseline for live change detection
        self.load_baseline()

        self.get_logger().info("=" * 60)
        self.get_logger().info("Inspection Node Started")
        self.get_logger().info(f"  Run ID: {self.run_id}")
        self.get_logger().info(f"  SAM 3 URL: {self.sam3_url}")
        self.get_logger().info(f"  Prompts: {self.prompts}")
        self.get_logger().info(f"  Interval: {self.detection_interval}s")
        self.get_logger().info(f"  Log file: {self.log_file}")
        self.get_logger().info(f"  Overlay: {self.publish_overlay}")
        self.get_logger().info(f"  Masks: {self.include_masks}")
        if self.has_baseline:
            self.get_logger().info(
                f"  Baseline: {len(self.baseline_detections)} objects loaded")
        else:
            self.get_logger().info("  Baseline: none (first run, all NEW)")
        self.get_logger().info("=" * 60)

    # ================================================================
    # Baseline Loading
    # ================================================================

    def load_baseline(self):
        """Load baseline log for live change detection."""
        baseline_path = get_str(self, 'baseline_log')

        self.get_logger().info(
            f"Baseline param: '{baseline_path}' | "
            f"Log dir: {self.log_dir} | "
            f"Current log: {os.path.basename(self.log_file)}")

        if not baseline_path:
            # Auto-detect: find the most recent log file (excluding current run)
            if os.path.isdir(self.log_dir):
                current_basename = os.path.basename(self.log_file)
                all_files = os.listdir(self.log_dir)
                logs = sorted([
                    os.path.join(self.log_dir, f)
                    for f in all_files
                    if f.endswith('.json') and f != current_basename
                    and not f.startswith('changes_')
                ], key=os.path.getmtime)
                self.get_logger().info(
                    f"Auto-detect: {len(all_files)} files in log dir, "
                    f"{len(logs)} candidate baselines")
                if logs:
                    baseline_path = logs[-1]
                    self.get_logger().info(
                        f"Auto-selected baseline: {baseline_path}")
                else:
                    self.get_logger().warn(
                        "No baseline candidates found in log dir")

        if baseline_path and os.path.exists(baseline_path):
            try:
                with open(baseline_path, 'r') as f:
                    data = json.load(f)
                self.baseline_detections = data.get('detections', [])
                self.baseline_matched = {
                    i: False for i in range(len(self.baseline_detections))
                }
                self.has_baseline = True
                self.get_logger().info(
                    f"Loaded baseline: {baseline_path} "
                    f"({len(self.baseline_detections)} objects)")
            except Exception as e:
                self.get_logger().error(f"Failed to load baseline: {e}")
        else:
            self.has_baseline = False
            if baseline_path:
                self.get_logger().error(
                    f"Baseline file not found: {baseline_path}")
            else:
                self.get_logger().warn(
                    "No baseline loaded — all detections will be NEW")

    # ================================================================
    # Change Classification
    # ================================================================

    def classify_change(self, detection):
        """Classify a detection against baseline: NEW, MOVED, or UNCHANGED.

        Once a detection is classified as UNCHANGED, it stays locked to
        prevent flip-flopping as the position average drifts between SAM calls.
        """
        if not self.has_baseline:
            self.detection_changes[detection['id']] = 'NEW'
            return 'NEW'

        # If already classified as UNCHANGED, keep it locked
        existing_type = self.detection_changes.get(detection['id'])
        if existing_type == 'UNCHANGED':
            return 'UNCHANGED'

        best_idx = None
        best_dist = float('inf')

        for i, base in enumerate(self.baseline_detections):
            if base['label'] != detection['label']:
                continue
            dx = detection['map_position'][0] - base['map_position'][0]
            dy = detection['map_position'][1] - base['map_position'][1]
            dz = detection['map_position'][2] - base['map_position'][2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx is not None and best_dist < self.match_distance:
            self.baseline_matched[best_idx] = True
            if best_dist < self.unchanged_threshold:
                change_type = 'UNCHANGED'
            else:
                change_type = 'MOVED'
            detection['baseline_position'] = \
                self.baseline_detections[best_idx]['map_position']
            detection['change_distance'] = best_dist
        else:
            change_type = 'NEW'

        self.detection_changes[detection['id']] = change_type
        return change_type

    # ================================================================
    # Callbacks
    # ================================================================

    def rgb_callback(self, msg):
        self.latest_rgb = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def info_callback(self, msg):
        self.camera_info = msg

    def detection_callback(self):
        """Periodically capture images and send to SAM 3 in a background thread."""
        if self._sam3_busy:
            return  # Previous request still in flight

        if self.latest_rgb is None:
            self.get_logger().warning(
                "Waiting for RGB images...", throttle_duration_sec=5.0)
            return

        if self.latest_depth is None:
            self.get_logger().warning(
                "Waiting for depth images...", throttle_duration_sec=5.0)
            return

        try:
            # Capture and encode images on the main thread (fast)
            rgb_cv = self.bridge.imgmsg_to_cv2(
                self.latest_rgb, desired_encoding='bgr8')
            depth_cv = self.bridge.imgmsg_to_cv2(
                self.latest_depth, desired_encoding='passthrough')

            _, rgb_buf = cv2.imencode(
                '.jpg', rgb_cv, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if depth_cv.dtype == np.float32:
                depth_mm = (depth_cv * 1000).astype(np.uint16)
            else:
                depth_mm = depth_cv.astype(np.uint16)
            _, depth_buf = cv2.imencode('.png', depth_mm)

            fx, fy, cx, cy = 605.176, 605.27, 324.636, 252.921
            if self.camera_info is not None:
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]

            # Send to SAM 3 in background thread so node stays responsive
            self._sam3_busy = True
            thread = threading.Thread(
                target=self._sam3_request,
                args=(rgb_cv, rgb_buf.tobytes(), depth_buf.tobytes(),
                      fx, fy, cx, cy),
                daemon=True)
            thread.start()

        except Exception as e:
            self.get_logger().error(f"Detection capture error: {e}")

    def _sam3_request(self, rgb_cv, rgb_bytes, depth_bytes,
                      fx, fy, cx, cy):
        """Run SAM 3 HTTP request in background thread."""
        try:
            self.get_logger().info(
                f"Sending image to SAM 3 ({rgb_cv.shape})...")
            response = requests.post(
                f"{self.sam3_url}/detect_with_depth",
                files={
                    'rgb_image': ('rgb.jpg', rgb_bytes, 'image/jpeg'),
                    'depth_image': ('depth.png', depth_bytes, 'image/png'),
                },
                data={
                    'prompts': json.dumps(list(self.prompts)),
                    'score_threshold': str(self.score_threshold),
                    'include_masks': str(self.include_masks).lower(),
                    'fx': str(fx),
                    'fy': str(fy),
                    'cx': str(cx),
                    'cy': str(cy),
                },
                timeout=30,
            )

            if response.status_code != 200:
                self.get_logger().error(
                    f"SAM 3 error: {response.status_code} {response.text}")
                return

            result = response.json()
            self.get_logger().info(
                f"SAM 3 response: {result['num_objects']} objects "
                f"({result['processing_time_ms']:.0f}ms)")

            # Process detections (TF transform, dedup, classify)
            for obj in result['objects']:
                self.process_detection(obj)

            # Render and publish annotated camera image (always, even with 0 objects)
            if self.publish_overlay:
                try:
                    self.render_overlay(rgb_cv, result.get('objects', []))
                    self.get_logger().info("Overlay published")
                except Exception as overlay_err:
                    self.get_logger().error(
                        f"Overlay rendering failed: {overlay_err}",
                        throttle_duration_sec=5.0)

        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                f"Cannot connect to SAM 3 at {self.sam3_url}",
                throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().error(f"Detection pipeline error: {e}")
        finally:
            self._sam3_busy = False

    def process_detection(self, obj: dict):
        """Transform detection to map frame, deduplicate, classify, and log."""
        if obj.get('centroid_3d_base') is None:
            self.get_logger().warning(
                f"Skipping {obj.get('label', '?')}: no centroid_3d_base")
            return
        self.get_logger().info(
            f"Processing: {obj['label']} (score={obj['score']:.2f})")

        point_base = PointStamped()
        point_base.header.frame_id = 'base_link'
        point_base.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        point_base.point.x = obj['centroid_3d_base'][0]
        point_base.point.y = obj['centroid_3d_base'][1]
        point_base.point.z = obj['centroid_3d_base'][2]

        try:
            point_map = self.tf_buffer.transform(
                point_base, 'map',
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f"TF transform failed: {e}")
            return

        map_x = point_map.point.x
        map_y = point_map.point.y
        map_z = point_map.point.z

        # Deduplication: update existing detection if close enough
        for existing in self.detections:
            if existing['label'] != obj['label']:
                continue
            dx = existing['map_position'][0] - map_x
            dy = existing['map_position'][1] - map_y
            dz = existing['map_position'][2] - map_z
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist < self.dedup_distance:
                if obj['score'] > existing['score']:
                    existing['score'] = obj['score']
                # Running average of map position for stability
                n = existing['sightings']
                existing['map_position'] = [
                    (existing['map_position'][0] * n + map_x) / (n + 1),
                    (existing['map_position'][1] * n + map_y) / (n + 1),
                    (existing['map_position'][2] * n + map_z) / (n + 1),
                ]
                existing['last_seen'] = datetime.now().isoformat()
                existing['sightings'] += 1
                # Re-classify with averaged position
                change_type = self.classify_change(existing)
                self.save_log()
                return

        detection = {
            'id': self.detection_id,
            'label': obj['label'],
            'score': obj['score'],
            'map_position': [map_x, map_y, map_z],
            'centroid_3d_base': obj['centroid_3d_base'],
            'mean_depth_m': obj.get('mean_depth_m'),
            'bbox': obj['bbox'],
            'first_seen': datetime.now().isoformat(),
            'last_seen': datetime.now().isoformat(),
            'sightings': 1,
            'run_id': self.run_id,
        }

        self.detections.append(detection)
        self.detection_id += 1

        # Classify against baseline
        change_type = self.classify_change(detection)

        self.get_logger().info(
            f"[{change_type}] #{detection['id']}: {obj['label']} "
            f"(score={obj['score']:.2f}) at map "
            f"({map_x:.2f}, {map_y:.2f}, {map_z:.2f})")

        self.save_log()

    # ================================================================
    # Camera Overlay
    # ================================================================

    def render_overlay(self, rgb_cv, objects):
        """Render annotated image with masks, bboxes, labels, and change status."""
        overlay = rgb_cv.copy()
        img_h, img_w = overlay.shape[:2]

        for obj in objects:
            # Determine change status for this object
            change_type = self._get_change_type_for_object(obj)
            color_bgr = CHANGE_COLORS_BGR.get(change_type, (200, 200, 0))

            # 1. Draw semi-transparent mask overlay
            mask_b64 = obj.get('mask_base64')
            if mask_b64:
                try:
                    mask_bytes = base64.b64decode(mask_b64)
                    mask_arr = np.frombuffer(mask_bytes, np.uint8)
                    mask_img = cv2.imdecode(mask_arr, cv2.IMREAD_GRAYSCALE)
                    if mask_img is not None:
                        # Resize mask if dimensions don't match
                        if mask_img.shape[:2] != (img_h, img_w):
                            mask_img = cv2.resize(
                                mask_img, (img_w, img_h),
                                interpolation=cv2.INTER_NEAREST)
                        mask_bool = mask_img > 127
                        colored = np.zeros_like(overlay)
                        colored[:] = color_bgr
                        overlay[mask_bool] = cv2.addWeighted(
                            overlay[mask_bool], 0.6,
                            colored[mask_bool], 0.4, 0)
                except Exception:
                    pass  # Skip mask if decode fails

            # 2. Draw bounding box
            bx, by, bw, bh = obj['bbox']
            bx, by = max(0, bx), max(0, by)
            cv2.rectangle(
                overlay, (bx, by), (bx + bw, by + bh), color_bgr, 2)

            # 3. Draw label with change status
            label_text = (
                f"[{change_type}] {obj['label']} ({obj['score']:.0%})")
            (tw, th), _ = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_y = max(by, th + 6)
            cv2.rectangle(
                overlay,
                (bx, label_y - th - 6),
                (bx + tw + 4, label_y),
                color_bgr, -1)
            cv2.putText(
                overlay, label_text,
                (bx + 2, label_y - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)

        # Publish as ROS Image
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        overlay_msg.header.frame_id = self.camera_frame
        self.overlay_pub.publish(overlay_msg)


    def _get_change_type_for_object(self, obj):
        """Match a SAM 3 response object to its change classification.

        Matches by label and centroid_3d_base proximity to internal detections.
        Falls back to label-only match if no spatial match found.
        """
        c3d = obj.get('centroid_3d_base')

        best_id = None
        best_dist = float('inf')

        if c3d is not None:
            for det in self.detections:
                if det['label'] != obj['label']:
                    continue
                base = det.get('centroid_3d_base')
                if base is None:
                    continue
                dx = c3d[0] - base[0]
                dy = c3d[1] - base[1]
                dz = c3d[2] - base[2]
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                if dist < best_dist:
                    best_dist = dist
                    best_id = det['id']

        if best_id is not None and best_dist < self.dedup_distance:
            return self.detection_changes.get(best_id, 'UNCHANGED')

        # Fallback: if baseline is loaded, check if this label exists
        # in the baseline at all — likely UNCHANGED, not NEW
        if self.has_baseline:
            for base in self.baseline_detections:
                if base['label'] == obj['label']:
                    return 'UNCHANGED'

        return 'NEW'

    # ================================================================
    # Marker Publishing
    # ================================================================

    def publish_all_markers(self):
        """Publish markers for all detections to RViz."""
        if not self.detections:
            return

        markers = MarkerArray()
        text_markers = MarkerArray()

        for idx, det in enumerate(self.detections):
            change_type = self.detection_changes.get(det['id'])

            # Use change-based color when baseline is loaded, label color otherwise
            if self.has_baseline and change_type:
                color = CHANGE_COLORS_RGBA[change_type]
            else:
                # Match label colors by substring (e.g. "red EXIT sign" matches "exit sign")
                color = DEFAULT_COLOR
                label_lower = det['label'].lower()
                for key, val in LABEL_COLORS.items():
                    if key in label_lower or label_lower in key:
                        color = val
                        break

            # Sphere marker at detection position
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'inspection_objects'
            m.id = det['id']
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = det['map_position'][0]
            m.pose.position.y = det['map_position'][1]
            m.pose.position.z = det['map_position'][2]
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color = color
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            markers.markers.append(m)

            # Compact text label — stagger height to reduce overlap
            z_offset = 0.25 + (idx % 3) * 0.12
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'inspection_labels'
            t.id = det['id']
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = det['map_position'][0]
            t.pose.position.y = det['map_position'][1]
            t.pose.position.z = det['map_position'][2] + z_offset
            t.pose.orientation.w = 1.0
            t.scale.z = 0.1
            # Color-coded text matches the marker
            t.color = ColorRGBA(
                r=min(color.r + 0.2, 1.0),
                g=min(color.g + 0.2, 1.0),
                b=min(color.b + 0.2, 1.0),
                a=1.0)
            t.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            # Short label: "fire ext. 60%" or "[NEW] exit sign 42%"
            short_label = det['label']
            if len(short_label) > 15:
                short_label = short_label[:13] + '..'
            if self.has_baseline and change_type:
                if change_type == 'UNCHANGED':
                    # Skip text for unchanged — sphere is enough
                    t.text = short_label
                else:
                    dist_str = ""
                    if change_type == 'MOVED' and 'change_distance' in det:
                        dist_str = f" {det['change_distance']:.1f}m"
                    t.text = f"[{change_type}] {short_label}{dist_str}"
            else:
                t.text = f"{short_label} {det['score']:.0%}"

            text_markers.markers.append(t)

        self.marker_pub.publish(markers)
        self.text_marker_pub.publish(text_markers)

        # Publish change-specific markers (3D shapes + MISSING markers)
        if self.has_baseline:
            self.publish_change_markers()

    def publish_change_markers(self):
        """Publish 3D change markers and MISSING markers for baseline objects."""
        change_markers = MarkerArray()
        change_labels = MarkerArray()
        marker_id = 0
        now = self.get_clock().now().to_msg()

        # Markers for current detections with change status
        for det in self.detections:
            change_type = self.detection_changes.get(det['id'])
            if not change_type:
                continue

            color = CHANGE_COLORS_RGBA[change_type]
            pos = det['map_position']

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = f'changes_{change_type.lower()}'
            m.id = marker_id
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.color = color
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            if change_type == 'NEW':
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.25
            elif change_type == 'MOVED':
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 0.2
                m.scale.z = 0.3
            else:  # UNCHANGED
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.1

            change_markers.markers.append(m)

            # Arrow for MOVED objects (from old to new position)
            if change_type == 'MOVED' and 'baseline_position' in det:
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = now
                arrow.ns = 'change_arrows'
                arrow.id = marker_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                start = Point()
                start.x = det['baseline_position'][0]
                start.y = det['baseline_position'][1]
                start.z = det['baseline_position'][2]
                end = Point()
                end.x = pos[0]
                end.y = pos[1]
                end.z = pos[2]
                arrow.points = [start, end]

                arrow.scale.x = 0.03  # shaft diameter
                arrow.scale.y = 0.06  # head diameter
                arrow.scale.z = 0.08  # head length
                arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
                arrow.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                change_markers.markers.append(arrow)

            # Change label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now
            t.ns = 'change_labels'
            t.id = marker_id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = pos[0]
            t.pose.position.y = pos[1]
            t.pose.position.z = pos[2] + 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            dist_str = ""
            if 'change_distance' in det:
                dist_str = f" ({det['change_distance']:.2f}m)"
            t.text = f"[{change_type}] {det['label']}{dist_str}"
            change_labels.markers.append(t)

            marker_id += 1

        # NOTE: MISSING objects (baseline items not yet seen) are NOT shown
        # during live runs — they are most likely just not revisited yet.
        # MISSING is only reported in the JSON log and by change_detector.py
        # after the run completes.

        self.change_marker_pub.publish(change_markers)
        self.change_label_pub.publish(change_labels)

    # ================================================================
    # Logging
    # ================================================================

    def save_log(self):
        """Save all detections and change summary to JSON log file."""
        # Build changes summary
        changes_summary = {}
        if self.has_baseline:
            n_new = sum(
                1 for ct in self.detection_changes.values() if ct == 'NEW')
            n_moved = sum(
                1 for ct in self.detection_changes.values() if ct == 'MOVED')
            n_unchanged = sum(
                1 for ct in self.detection_changes.values()
                if ct == 'UNCHANGED')
            n_not_revisited = sum(
                1 for matched in self.baseline_matched.values()
                if not matched)
            changes_summary = {
                'new': n_new,
                'moved': n_moved,
                'unchanged': n_unchanged,
                'not_revisited': n_not_revisited,
            }

        # Add change_type to each detection for the log
        detections_with_changes = []
        for det in self.detections:
            det_copy = dict(det)
            if self.has_baseline:
                det_copy['change_type'] = self.detection_changes.get(
                    det['id'], 'NEW')
            detections_with_changes.append(det_copy)

        log_data = {
            'run_id': self.run_id,
            'timestamp': datetime.now().isoformat(),
            'prompts': list(self.prompts),
            'num_detections': len(self.detections),
            'detections': detections_with_changes,
        }

        if changes_summary:
            log_data['changes_summary'] = changes_summary

        with open(self.log_file, 'w') as f:
            json.dump(log_data, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_log()
        node.get_logger().info(
            f"Inspection complete. {len(node.detections)} objects detected. "
            f"Log saved to {node.log_file}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

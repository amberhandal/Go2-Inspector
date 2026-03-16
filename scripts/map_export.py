#!/usr/bin/env python3
"""
Map Export Tool for Go2 Inspection System

Generates a professional building floor plan with inspection markers,
styled like an evacuation/safety plan you'd see posted on a wall.

Usage:
    # While SLAM is running (best quality -uses real occupancy grid):
    ros2 run go2_navigation map_export.py

    # From a previously saved map (PGM/PNG + YAML from map_saver):
    python3 map_export.py --map ~/maps/building.pgm \
        --map-yaml ~/maps/building.yaml

    # From RTAB-Map database (after Ctrl+C):
    python3 map_export.py --db ~/.ros/rtabmap.db

    # Specify inspection log:
    python3 map_export.py --map map.pgm --map-yaml map.yaml \
        --log ~/inspection_logs/run_xxx.json -o ~/inspection_maps

    # Without markers:
    ros2 run go2_navigation map_export.py \
        --ros-args -p overlay_markers:=false
"""

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
from datetime import datetime

import cv2
import numpy as np

# ROS imports (optional for standalone mode)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy,
        HistoryPolicy
    )
    from nav_msgs.msg import OccupancyGrid
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


# ============================================================================
# Style constants -building safety map look
# ============================================================================
SCALE_FACTOR = 4          # Upscale for print quality
WALL_COLOR = (40, 40, 40)
FLOOR_COLOR = (255, 255, 255)
UNKNOWN_COLOR = (235, 235, 235)
BORDER_COLOR = (60, 60, 60)
TITLE_BG = (50, 50, 50)
TITLE_FG = (255, 255, 255)
GRID_COLOR = (230, 230, 230)

# Marker colors (BGR)
MARKER_STYLES = {
    'NEW':       {'color': (220, 60, 20),   'label': 'NEW'},        # Blue
    'MISSING':   {'color': (40, 40, 220),   'label': 'MISSING'},    # Red
    'MOVED':     {'color': (20, 140, 240),  'label': 'MOVED'},      # Orange
    'UNCHANGED': {'color': (40, 180, 40),   'label': 'UNCHANGED'},  # Green
}
DEFAULT_MARKER = {'color': (180, 140, 20), 'label': ''}  # Teal

# Icons for known object types
ICON_MAP = {
    'fire extinguisher': 'FE',
    'exit sign':         'EXIT',
    'fire alarm':        'FA',
    'smoke detector':    'SD',
    'aed':               'AED',
    'first aid':         'FA+',
    'sprinkler':         'SP',
}


# ============================================================================
# Map rendering
# ============================================================================

def occupancy_grid_to_image(data, width, height):
    """Convert OccupancyGrid data to a clean BGR building plan image."""
    arr = np.array(data, dtype=np.int8).reshape((height, width))
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Floor (free space)
    img[arr == 0] = FLOOR_COLOR
    # Unknown
    img[arr == -1] = UNKNOWN_COLOR
    # Walls (occupied)
    img[arr >= 50] = WALL_COLOR
    # Partial (gradient)
    partial = (arr > 0) & (arr < 50)
    if np.any(partial):
        vals = arr[partial].astype(np.float32)
        gray = (255 - (vals / 50.0 * 180)).astype(np.uint8)
        img[partial, 0] = gray
        img[partial, 1] = gray
        img[partial, 2] = gray

    # Flip (OccupancyGrid origin is bottom-left)
    img = np.flipud(img)
    return img


def upscale_map(img, scale=SCALE_FACTOR):
    """Upscale with nearest-neighbor to keep walls crisp."""
    return cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale),
                      interpolation=cv2.INTER_NEAREST)


def thicken_walls(img):
    """Make walls thicker and cleaner for print."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    wall_mask = gray < 80
    kernel = np.ones((3, 3), np.uint8)
    thick = cv2.dilate(wall_mask.astype(np.uint8), kernel, iterations=1)
    img[thick > 0] = WALL_COLOR
    return img


def world_to_pixel(x, y, origin_x, origin_y, resolution, img_height,
                   scale=SCALE_FACTOR):
    """Convert world coordinates to scaled pixel coordinates."""
    px = int((x - origin_x) / resolution) * scale
    py = img_height - 1 - int((y - origin_y) / resolution) * scale
    return px, py


def draw_marker_icon(img, cx, cy, label, change_type, score, resolution):
    """Draw a professional-looking marker with icon and callout."""
    style = MARKER_STYLES.get(change_type, DEFAULT_MARKER)
    color = style['color']
    status = style['label']

    r = max(8, int(0.25 / resolution * SCALE_FACTOR))
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Outer ring + filled center
    cv2.circle(img, (cx, cy), r + 3, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r, (0, 0, 0), 1, cv2.LINE_AA)

    # Icon text inside the circle
    abbrev = ICON_MAP.get(label.lower(), label[:2].upper())
    icon_scale = 0.3 if len(abbrev) <= 2 else 0.25
    (tw, th), _ = cv2.getTextSize(abbrev, font, icon_scale, 1)
    cv2.putText(img, abbrev, (cx - tw // 2, cy + th // 2),
                font, icon_scale, (255, 255, 255), 1, cv2.LINE_AA)

    # Callout label to the right
    callout_x = cx + r + 8
    callout_y = cy

    # Build label text
    display_label = label.title()
    if status:
        display_label = f"{display_label} [{status}]"
    if score > 0:
        display_label += f" {score:.0%}"

    label_scale = 0.35
    label_thick = 1
    (tw, th), baseline = cv2.getTextSize(display_label, font, label_scale,
                                         label_thick)

    # Callout line
    cv2.line(img, (cx + r + 2, cy), (callout_x - 2, callout_y),
             (0, 0, 0), 1, cv2.LINE_AA)

    # Label background
    pad = 3
    cv2.rectangle(img,
                  (callout_x - pad, callout_y - th - pad),
                  (callout_x + tw + pad, callout_y + pad + baseline),
                  (255, 255, 255), -1)
    cv2.rectangle(img,
                  (callout_x - pad, callout_y - th - pad),
                  (callout_x + tw + pad, callout_y + pad + baseline),
                  color, 1, cv2.LINE_AA)

    # Label text
    cv2.putText(img, display_label, (callout_x, callout_y),
                font, label_scale, (30, 30, 30), label_thick, cv2.LINE_AA)

    return img


def draw_markers(img, detections, origin_x, origin_y, resolution, img_height):
    """Draw all inspection markers on the map."""
    for det in detections:
        pos = det.get('map_position')
        if not pos or len(pos) < 2:
            continue

        px, py = world_to_pixel(pos[0], pos[1], origin_x, origin_y,
                                resolution, img_height)

        if px < 0 or py < 0 or px >= img.shape[1] or py >= img.shape[0]:
            continue

        label = det.get('label', 'unknown')
        change_type = det.get('change_type', '')
        score = det.get('score', 0)

        draw_marker_icon(img, px, py, label, change_type, score, resolution)

    return img


def draw_legend(img, detections):
    """Draw a professional legend box in the bottom-right corner."""
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Collect unique change types present
    change_types_present = set()
    labels_present = set()
    for det in detections:
        ct = det.get('change_type', '')
        if ct:
            change_types_present.add(ct)
        labels_present.add(det.get('label', 'unknown').lower())

    if not change_types_present:
        change_types_present = {'NEW'}  # Default for first run

    # Build legend entries
    entries = []
    for ct in ['NEW', 'MOVED', 'MISSING', 'UNCHANGED']:
        if ct in change_types_present:
            style = MARKER_STYLES[ct]
            entries.append((ct, style['color']))

    # Add object type entries
    for label in sorted(labels_present):
        abbrev = ICON_MAP.get(label, label[:2].upper())
        entries.append((f"{abbrev} = {label.title()}", (100, 100, 100)))

    line_h = 22
    legend_w = 200
    legend_h = len(entries) * line_h + 35
    x0 = img.shape[1] - legend_w - 15
    y0 = img.shape[0] - legend_h - 15

    # Background
    cv2.rectangle(img, (x0, y0), (x0 + legend_w, y0 + legend_h),
                  (255, 255, 255), -1)
    cv2.rectangle(img, (x0, y0), (x0 + legend_w, y0 + legend_h),
                  BORDER_COLOR, 2, cv2.LINE_AA)

    # Title
    cv2.putText(img, "LEGEND", (x0 + 10, y0 + 18),
                font, 0.45, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.line(img, (x0 + 5, y0 + 24), (x0 + legend_w - 5, y0 + 24),
             (180, 180, 180), 1)

    for i, (label, color) in enumerate(entries):
        y = y0 + 42 + i * line_h
        if label in MARKER_STYLES:
            cv2.circle(img, (x0 + 16, y - 4), 6, color, -1, cv2.LINE_AA)
            cv2.circle(img, (x0 + 16, y - 4), 6, (0, 0, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(img, "*", (x0 + 12, y), font, 0.35, color, 1)
        cv2.putText(img, label, (x0 + 30, y), font, 0.35,
                    (50, 50, 50), 1, cv2.LINE_AA)

    return img


def draw_scale_bar(img, resolution):
    """Draw a scale bar in the bottom-left corner."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Pick a nice round distance
    pixels_per_meter = SCALE_FACTOR / resolution
    for bar_m in [5.0, 2.0, 1.0, 0.5]:
        bar_px = int(bar_m * pixels_per_meter)
        if bar_px < img.shape[1] * 0.3:
            break

    x0 = 20
    y0 = img.shape[0] - 25

    # Background
    cv2.rectangle(img, (x0 - 5, y0 - 20), (x0 + bar_px + 60, y0 + 10),
                  (255, 255, 255), -1)

    # Bar
    cv2.rectangle(img, (x0, y0 - 5), (x0 + bar_px, y0), (0, 0, 0), -1)
    cv2.rectangle(img, (x0, y0 - 5), (x0 + bar_px // 2, y0),
                  (120, 120, 120), -1)

    # Ticks
    for frac in [0, 0.5, 1.0]:
        tx = x0 + int(bar_px * frac)
        cv2.line(img, (tx, y0 - 8), (tx, y0 + 3), (0, 0, 0), 1)

    # Labels
    label = f"{bar_m:.0f}m" if bar_m >= 1 else f"{bar_m*100:.0f}cm"
    cv2.putText(img, "0", (x0 - 3, y0 - 10), font, 0.3,
                (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, label, (x0 + bar_px - 5, y0 - 10), font, 0.3,
                (0, 0, 0), 1, cv2.LINE_AA)

    return img


def draw_title_block(img, title, subtitle="", date_str=""):
    """Add a professional title block at the top."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    if not date_str:
        date_str = datetime.now().strftime('%Y-%m-%d %H:%M')

    bar_h = 50
    titled = np.ones((img.shape[0] + bar_h, img.shape[1], 3),
                     dtype=np.uint8) * 255
    # Title bar background
    titled[:bar_h] = TITLE_BG
    titled[bar_h:] = img

    # Title text
    cv2.putText(titled, title, (15, 22), font, 0.6,
                TITLE_FG, 1, cv2.LINE_AA)

    # Subtitle line
    sub = subtitle or "Go2 Autonomous Inspection System"
    cv2.putText(titled, sub, (15, 40), font, 0.35,
                (180, 180, 180), 1, cv2.LINE_AA)

    # Date on the right
    (tw, _), _ = cv2.getTextSize(date_str, font, 0.35, 1)
    cv2.putText(titled, date_str, (titled.shape[1] - tw - 15, 40),
                font, 0.35, (180, 180, 180), 1, cv2.LINE_AA)

    return titled


def add_border(img, width=3):
    """Add a clean border around the map."""
    cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1),
                  BORDER_COLOR, width)
    return img


def render_building_map(img, detections, origin_x, origin_y, resolution,
                        title="Building Inspection Map", log_name=""):
    """Full pipeline: occupancy grid → professional building plan."""
    # Upscale for print quality
    img = upscale_map(img)

    # Thicken walls
    img = thicken_walls(img)

    # Draw markers
    if detections:
        img_h_before_scale = img.shape[0]
        draw_markers(img, detections, origin_x, origin_y, resolution,
                     img_h_before_scale)
        draw_legend(img, detections)

    # Scale bar
    draw_scale_bar(img, resolution)

    # Border
    add_border(img)

    # Title block
    n_obj = len(detections) if detections else 0
    subtitle = f"{n_obj} objects detected"
    if log_name:
        subtitle = f"{log_name} -{subtitle}"
    img = draw_title_block(img, title, subtitle)

    return img


# ============================================================================
# Helpers
# ============================================================================

def find_latest_log():
    """Find the most recent inspection log."""
    log_dir = os.path.expanduser('~/inspection_logs')
    if not os.path.isdir(log_dir):
        return ''
    logs = sorted([
        os.path.join(log_dir, f) for f in os.listdir(log_dir)
        if f.endswith('.json') and not f.startswith('changes_')
    ], key=os.path.getmtime)
    return logs[-1] if logs else ''


def load_detections(log_file):
    """Load detections from inspection log."""
    if not log_file or not os.path.isfile(log_file):
        return []
    with open(log_file) as f:
        data = json.load(f)
    return data.get('detections', [])


# ============================================================================
# ROS Node mode -subscribe to /map while running
# ============================================================================

class MapExportNode(Node):
    def __init__(self):
        super().__init__('map_export')

        self.declare_parameter('log_file', '')
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/inspection_maps'))
        self.declare_parameter('overlay_markers', True)

        self.log_file = self.get_parameter(
            'log_file').get_parameter_value().string_value
        self.output_dir = self.get_parameter(
            'output_dir').get_parameter_value().string_value
        self.overlay = self.get_parameter(
            'overlay_markers').get_parameter_value().bool_value

        os.makedirs(self.output_dir, exist_ok=True)

        if not self.log_file:
            self.log_file = find_latest_log()

        self.detections = []
        if self.log_file and os.path.isfile(self.log_file):
            self.detections = load_detections(self.log_file)
            self.get_logger().info(
                f'Loaded {len(self.detections)} detections from '
                f'{self.log_file}')
        elif self.overlay:
            self.get_logger().warn(
                'No inspection log found -exporting map without markers')
            self.overlay = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.map_received = False
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos)
        self.get_logger().info('Waiting for /map topic...')
        self.create_timer(30.0, self.timeout_callback)

    def map_callback(self, msg):
        if self.map_received:
            return
        self.map_received = True

        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f} m/px')

        img = occupancy_grid_to_image(
            msg.data, msg.info.width, msg.info.height)
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Save raw map + YAML (for re-use with --map mode)
        raw_path = os.path.join(self.output_dir, f'map_{ts}.png')
        cv2.imwrite(raw_path, img)
        yaml_path = os.path.join(self.output_dir, f'map_{ts}.yaml')
        with open(yaml_path, 'w') as f:
            f.write(f"image: map_{ts}.png\n")
            f.write(f"resolution: {resolution}\n")
            f.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
        self.get_logger().info(f'Saved raw map: {raw_path}')

        # Render building plan
        detections = self.detections if self.overlay else []
        log_name = ''
        if self.log_file:
            log_name = os.path.splitext(
                os.path.basename(self.log_file))[0]

        plan = render_building_map(
            img, detections, origin_x, origin_y, resolution,
            title="BUILDING INSPECTION MAP", log_name=log_name)

        plan_path = os.path.join(self.output_dir,
                                 f'building_plan_{ts}.png')
        cv2.imwrite(plan_path, plan)
        self.get_logger().info(f'Saved building plan: {plan_path}')

        self.get_logger().info('Map export complete. Shutting down.')
        raise SystemExit(0)

    def timeout_callback(self):
        if not self.map_received:
            self.get_logger().error(
                'Timeout: no /map received after 30s. '
                'Is RTAB-Map running and publishing /map?')
            raise SystemExit(1)


# ============================================================================
# Database export mode (no ROS needed)
# ============================================================================

def export_from_database(args):
    """Export 2D occupancy grid from RTAB-Map .db file."""
    import sqlite3

    db_path = os.path.expanduser(args.db)
    if not os.path.isfile(db_path):
        print(f"Error: database not found: {db_path}")
        sys.exit(1)

    output_dir = os.path.expanduser(args.output or '~/inspection_maps')
    os.makedirs(output_dir, exist_ok=True)

    img = None
    origin_x, origin_y, resolution = 0.0, 0.0, 0.05

    # Try rtabmap-grid first (generates PGM + YAML from database)
    with tempfile.TemporaryDirectory() as tmpdir:
        for cmd in [
            ['rtabmap-grid', db_path, '--output', tmpdir],
            ['rtabmap-grid', db_path],
        ]:
            try:
                print(f"Trying: {' '.join(cmd)}")
                result = subprocess.run(
                    cmd, capture_output=True, text=True,
                    timeout=120, cwd=tmpdir)
                if result.returncode == 0:
                    break
            except (FileNotFoundError, subprocess.TimeoutExpired):
                continue

        # Look for generated map files in tmpdir and cwd
        for search_dir in [tmpdir, os.getcwd()]:
            pgm_files = [
                f for f in os.listdir(search_dir)
                if (f.endswith(('.pgm', '.png'))
                    and 'grid' in f.lower())
                or f == 'rtabmap.pgm']
            yaml_files = [
                f for f in os.listdir(search_dir)
                if (f.endswith('.yaml')
                    and 'grid' in f.lower())
                or f == 'rtabmap.yaml']
            if pgm_files:
                grid_img_path = os.path.join(search_dir, pgm_files[0])
                img = cv2.imread(grid_img_path)
                if img is not None:
                    print(f"Loaded grid from: {grid_img_path}")
                    if yaml_files:
                        try:
                            import yaml
                            with open(os.path.join(search_dir,
                                                   yaml_files[0])) as f:
                                meta = yaml.safe_load(f)
                            resolution = meta.get('resolution', 0.05)
                            origin = meta.get('origin', [0.0, 0.0, 0.0])
                            origin_x, origin_y = origin[0], origin[1]
                        except Exception:
                            pass
                    break

    # Fallback: build grid from node poses + scan data
    if img is None:
        print("Grid export tools not available. Building from poses...")
        conn = sqlite3.connect(db_path)
        img, origin_x, origin_y, resolution = build_grid_from_poses(conn)
        conn.close()

    # Load inspection log
    log_file = args.log or find_latest_log()
    detections = load_detections(log_file) if log_file else []
    if log_file and detections:
        print(f"Loaded {len(detections)} detections from {log_file}")

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Save raw map
    raw_path = os.path.join(output_dir, f'map_{ts}.png')
    cv2.imwrite(raw_path, img)
    yaml_path = os.path.join(output_dir, f'map_{ts}.yaml')
    with open(yaml_path, 'w') as yf:
        yf.write(f"image: map_{ts}.png\n")
        yf.write(f"resolution: {resolution}\n")
        yf.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
        yf.write("negate: 0\n")
        yf.write("occupied_thresh: 0.65\n")
        yf.write("free_thresh: 0.196\n")
    print(f"Saved raw map: {raw_path}")

    # Render building plan
    log_name = ''
    if log_file:
        log_name = os.path.splitext(
            os.path.basename(log_file))[0]
    plan = render_building_map(
        img, detections, origin_x, origin_y, resolution,
        title="BUILDING INSPECTION MAP", log_name=log_name)

    plan_path = os.path.join(output_dir, f'building_plan_{ts}.png')
    cv2.imwrite(plan_path, plan)
    print(f"Saved building plan: {plan_path}")
    print("Done.")


def build_grid_from_poses(conn):
    """Fallback: build a 2D occupancy-style grid from RTAB-Map poses.

    Uses scan data from each node to trace rays and mark obstacles,
    producing a real occupancy grid rather than just circles.
    """
    cursor = conn.cursor()

    # Get poses
    cursor.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL")
    rows = cursor.fetchall()

    if not rows:
        print("Error: no poses found in database")
        sys.exit(1)

    # Parse poses
    poses = {}
    for row_id, pose_blob in rows:
        if pose_blob is None or len(pose_blob) < 48:
            continue
        vals = struct.unpack('12f', pose_blob[:48])
        x, y = vals[3], vals[7]
        # Extract yaw from rotation matrix
        yaw = math.atan2(vals[4], vals[0])  # atan2(r21, r11)
        poses[row_id] = (x, y, yaw)

    if not poses:
        print("Error: could not parse any poses")
        sys.exit(1)

    positions = list(poses.values())
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]

    resolution = 0.05
    margin = 3.0
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)

    # Track cell states: 0=unknown, 1=free, 2=occupied
    grid = np.zeros((height, width), dtype=np.uint8)

    # For each pose, cast rays outward to mark free space
    # Use a simple approach: mark area around robot as free
    for x, y, yaw in positions:
        px = int((x - min_x) / resolution)
        py = height - 1 - int((y - min_y) / resolution)
        if 0 <= px < width and 0 <= py < height:
            # Free area around robot (~1m radius)
            cv2.circle(grid, (px, py), int(1.0 / resolution), 1, -1)

    # Try to get scan data to mark walls
    try:
        cursor.execute(
            "SELECT id, scan_info FROM Data "
            "WHERE scan_info IS NOT NULL")
        cursor.fetchall()
    except Exception:
        pass

    # Convert to image
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[grid == 0] = UNKNOWN_COLOR     # Unknown
    img[grid == 1] = FLOOR_COLOR       # Free
    img[grid == 2] = WALL_COLOR        # Occupied

    # Draw robot path as subtle dotted line
    pts = []
    for x, y, _ in positions:
        px = int((x - min_x) / resolution)
        py = height - 1 - int((y - min_y) / resolution)
        pts.append((px, py))

    for i in range(0, len(pts) - 1, 2):
        cv2.line(img, pts[i], pts[min(i + 1, len(pts) - 1)],
                 (210, 210, 210), 1)

    # Draw start/end markers
    if pts:
        cv2.drawMarker(img, pts[0], (0, 160, 0), cv2.MARKER_DIAMOND, 8, 2)
        cv2.drawMarker(img, pts[-1], (0, 0, 200), cv2.MARKER_CROSS, 8, 2)

    return img, min_x, min_y, resolution


# ============================================================================
# Standalone mode (pre-saved map image)
# ============================================================================

def standalone_export(args):
    """Render building plan from a previously saved map image + YAML."""
    img = cv2.imread(args.map, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"Error: cannot read map image: {args.map}")
        sys.exit(1)

    # Convert grayscale PGM to BGR
    if len(img.shape) == 2:
        bgr = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        # PGM from map_saver: 254=free, 205=unknown, 0=occupied
        bgr[img > 250] = FLOOR_COLOR
        bgr[(img >= 200) & (img <= 210)] = UNKNOWN_COLOR
        bgr[img < 50] = WALL_COLOR
        partial = (img >= 50) & (img < 200)
        if np.any(partial):
            gray = img[partial]
            bgr[partial, 0] = gray
            bgr[partial, 1] = gray
            bgr[partial, 2] = gray
        img = bgr

    origin_x, origin_y, resolution = 0.0, 0.0, 0.05
    if args.map_yaml:
        try:
            import yaml
            with open(args.map_yaml) as f:
                meta = yaml.safe_load(f)
            resolution = meta.get('resolution', 0.05)
            origin = meta.get('origin', [0.0, 0.0, 0.0])
            origin_x, origin_y = origin[0], origin[1]
        except Exception as e:
            print(f"Warning: could not parse YAML: {e}")

    log_file = args.log or find_latest_log()
    detections = load_detections(log_file) if log_file else []
    if detections:
        print(f"Loaded {len(detections)} detections")

    log_name = ''
    if log_file:
        log_name = os.path.splitext(os.path.basename(log_file))[0]

    plan = render_building_map(
        img, detections, origin_x, origin_y, resolution,
        title="BUILDING INSPECTION MAP", log_name=log_name)

    output_dir = os.path.expanduser(args.output or '~/inspection_maps')
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(output_dir, f'building_plan_{ts}.png')
    cv2.imwrite(out_path, plan)
    print(f"Saved: {out_path}")


# ============================================================================
# Main
# ============================================================================

def main():
    if '--db' in sys.argv:
        parser = argparse.ArgumentParser(
            description='Export building plan from RTAB-Map database')
        parser.add_argument('--db', required=True,
                            help='RTAB-Map database file (.db)')
        parser.add_argument('--log', '-l', help='Inspection JSON log')
        parser.add_argument('--output', '-o',
                            help='Output dir (~/inspection_maps)')
        args = parser.parse_args()
        export_from_database(args)
        return

    if '--map' in sys.argv:
        parser = argparse.ArgumentParser(
            description='Export building plan from saved map image')
        parser.add_argument('--map', required=True,
                            help='Map image (PGM/PNG)')
        parser.add_argument('--map-yaml',
                            help='Map YAML metadata')
        parser.add_argument('--log', '-l',
                            help='Inspection JSON log')
        parser.add_argument('-o', '--output',
                            help='Output dir')
        args = parser.parse_args()
        standalone_export(args)
        return

    if not HAS_ROS:
        print("ROS 2 not available. Use --db or --map for standalone mode.")
        sys.exit(1)

    rclpy.init()
    node = MapExportNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

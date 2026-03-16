#!/usr/bin/env python3
"""
Change Detection Node for Go2 Robot Inspection System

Compares the current inspection run's detections against a previous run
and publishes change markers (new, missing, moved objects) in RViz.

Usage:
    ros2 run go2_navigation change_detector.py --ros-args \
        -p current_log:="/home/USER/inspection_logs/run_002.json" \
        -p previous_log:="/home/USER/inspection_logs/run_001.json" \
        -p match_distance:=0.8

Or in a launch file:
    <node pkg="go2_navigation" exec="change_detector.py" name="change_detector" output="screen">
      <param name="current_log" value="$(env HOME)/inspection_logs/run_002.json"/>
      <param name="previous_log" value="$(env HOME)/inspection_logs/run_001.json"/>
    </node>
"""

import json
import os
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class ChangeDetector(Node):
    def __init__(self):
        super().__init__('change_detector')

        # Parameters
        self.declare_parameter('current_log', '')
        self.declare_parameter('previous_log', '')
        self.declare_parameter('match_distance', 0.8)  # meters
        self.declare_parameter('log_dir', os.path.expanduser('~/inspection_logs'))

        self.current_log_path = self.get_parameter('current_log').as_string()
        self.previous_log_path = self.get_parameter('previous_log').as_string()
        self.match_distance = self.get_parameter('match_distance').as_double()
        self.log_dir = self.get_parameter('log_dir').as_string()

        # Auto-detect logs if not specified
        if not self.current_log_path or not self.previous_log_path:
            self._auto_detect_logs()

        # Publishers
        self.change_marker_pub = self.create_publisher(
            MarkerArray, '/inspection_changes', 10)
        self.summary_marker_pub = self.create_publisher(
            MarkerArray, '/inspection_change_labels', 10)

        # State
        self.changes: List[Dict] = []
        self.comparison_done = False

        # Run comparison once data is available
        self.timer = self.create_timer(2.0, self.run_comparison_and_publish)

        self.get_logger().info("Change Detector initialized")
        self.get_logger().info(f"  Current:  {self.current_log_path}")
        self.get_logger().info(f"  Previous: {self.previous_log_path}")

    def _auto_detect_logs(self):
        """Find the two most recent log files automatically."""
        if not os.path.isdir(self.log_dir):
            self.get_logger().error(f"Log directory not found: {self.log_dir}")
            return

        logs = sorted([
            os.path.join(self.log_dir, f)
            for f in os.listdir(self.log_dir)
            if f.endswith('.json')
        ], key=os.path.getmtime)

        if len(logs) >= 2:
            self.previous_log_path = logs[-2]
            self.current_log_path = logs[-1]
            self.get_logger().info(f"Auto-detected logs:")
            self.get_logger().info(f"  Previous: {self.previous_log_path}")
            self.get_logger().info(f"  Current:  {self.current_log_path}")
        elif len(logs) == 1:
            self.get_logger().warn("Only one log found, no comparison possible")
        else:
            self.get_logger().warn("No log files found")

    def load_log(self, path: str) -> Optional[Dict]:
        """Load a detection log JSON file."""
        if not path or not os.path.exists(path):
            self.get_logger().error(f"Log file not found: {path}")
            return None
        with open(path, 'r') as f:
            return json.load(f)

    def compare_runs(self, current_data: Dict, previous_data: Dict) -> List[Dict]:
        """Compare two inspection runs and identify changes.

        Change types:
          - MISSING: Object was in previous run but not in current
          - NEW: Object is in current run but not in previous
          - MOVED: Object is in both runs but position changed significantly
          - UNCHANGED: Object is in both runs at approximately same position
        """
        current_dets = current_data.get('detections', [])
        previous_dets = previous_data.get('detections', [])
        changes = []

        # Track which previous detections are matched
        prev_matched = [False] * len(previous_dets)

        for curr in current_dets:
            best_match_idx = None
            best_match_dist = float('inf')

            for i, prev in enumerate(previous_dets):
                if prev['label'] != curr['label']:
                    continue

                dx = curr['map_position'][0] - prev['map_position'][0]
                dy = curr['map_position'][1] - prev['map_position'][1]
                dz = curr['map_position'][2] - prev['map_position'][2]
                dist = (dx*dx + dy*dy + dz*dz) ** 0.5

                if dist < best_match_dist:
                    best_match_dist = dist
                    best_match_idx = i

            if best_match_idx is not None and best_match_dist < self.match_distance:
                prev_matched[best_match_idx] = True
                prev = previous_dets[best_match_idx]

                if best_match_dist < 0.35:  # within 35cm = unchanged
                    changes.append({
                        'type': 'UNCHANGED',
                        'label': curr['label'],
                        'current_position': curr['map_position'],
                        'previous_position': prev['map_position'],
                        'distance': best_match_dist,
                        'current_score': curr['score'],
                    })
                else:
                    changes.append({
                        'type': 'MOVED',
                        'label': curr['label'],
                        'current_position': curr['map_position'],
                        'previous_position': prev['map_position'],
                        'distance': best_match_dist,
                        'current_score': curr['score'],
                    })
            else:
                changes.append({
                    'type': 'NEW',
                    'label': curr['label'],
                    'current_position': curr['map_position'],
                    'current_score': curr['score'],
                })

        # Any unmatched previous detections are MISSING
        for i, prev in enumerate(previous_dets):
            if not prev_matched[i]:
                changes.append({
                    'type': 'MISSING',
                    'label': prev['label'],
                    'previous_position': prev['map_position'],
                    'previous_score': prev.get('score', 0),
                })

        return changes

    def run_comparison_and_publish(self):
        """Run comparison once and then just republish markers."""
        if not self.comparison_done:
            current_data = self.load_log(self.current_log_path)
            previous_data = self.load_log(self.previous_log_path)

            if current_data is None or previous_data is None:
                return  # Will retry on next timer tick

            self.changes = self.compare_runs(current_data, previous_data)
            self.comparison_done = True

            # Log summary
            n_new = sum(1 for c in self.changes if c['type'] == 'NEW')
            n_missing = sum(1 for c in self.changes if c['type'] == 'MISSING')
            n_moved = sum(1 for c in self.changes if c['type'] == 'MOVED')
            n_unchanged = sum(1 for c in self.changes if c['type'] == 'UNCHANGED')

            self.get_logger().info("=" * 50)
            self.get_logger().info("CHANGE DETECTION RESULTS")
            self.get_logger().info(f"  Unchanged: {n_unchanged}")
            self.get_logger().info(f"  New:       {n_new}")
            self.get_logger().info(f"  Missing:   {n_missing}")
            self.get_logger().info(f"  Moved:     {n_moved}")
            self.get_logger().info("=" * 50)

            for change in self.changes:
                if change['type'] == 'UNCHANGED':
                    continue
                pos = change.get('current_position') or change.get('previous_position')
                self.get_logger().info(
                    f"  [{change['type']}] {change['label']} at "
                    f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                    + (f" moved {change['distance']:.2f}m" if change['type'] == 'MOVED' else ""))

            # Save change report
            report_path = os.path.join(
                self.log_dir,
                f"changes_{os.path.basename(self.previous_log_path).replace('.json','')}__vs__"
                f"{os.path.basename(self.current_log_path).replace('.json','')}.json")
            with open(report_path, 'w') as f:
                json.dump({
                    'previous_run': self.previous_log_path,
                    'current_run': self.current_log_path,
                    'summary': {
                        'unchanged': n_unchanged,
                        'new': n_new,
                        'missing': n_missing,
                        'moved': n_moved,
                    },
                    'changes': self.changes,
                }, f, indent=2)
            self.get_logger().info(f"Change report saved to {report_path}")

        # Publish markers (always, so they stay visible)
        self.publish_change_markers()

    def publish_change_markers(self):
        """Publish RViz markers for detected changes."""
        if not self.changes:
            return

        markers = MarkerArray()
        labels = MarkerArray()

        # Color scheme
        colors = {
            'NEW':       ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9),   # Blue
            'MISSING':   ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),   # Red
            'MOVED':     ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),   # Orange
            'UNCHANGED': ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.5),   # Green (dimmer)
        }

        marker_id = 0
        for change in self.changes:
            ctype = change['type']
            color = colors[ctype]
            pos = change.get('current_position') or change.get('previous_position')

            # Main marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'changes_{ctype.lower()}'
            m.id = marker_id
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.color = color
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            if ctype == 'MISSING':
                # X shape for missing items
                m.type = Marker.CUBE
                m.scale.x = 0.25
                m.scale.y = 0.25
                m.scale.z = 0.02
            elif ctype == 'NEW':
                # Diamond for new items
                m.type = Marker.SPHERE
                m.scale.x = 0.25
                m.scale.y = 0.25
                m.scale.z = 0.25
            elif ctype == 'MOVED':
                # Cylinder for moved items
                m.type = Marker.CYLINDER
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.3
            else:
                # Small sphere for unchanged
                m.type = Marker.SPHERE
                m.scale.x = 0.1
                m.scale.y = 0.1
                m.scale.z = 0.1

            markers.markers.append(m)

            # Text label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
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

            dist_str = f" ({change['distance']:.2f}m)" if 'distance' in change else ""
            t.text = f"[{ctype}] {change['label']}{dist_str}"
            labels.markers.append(t)

            # For MOVED objects, add an arrow from old to new position
            if ctype == 'MOVED':
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'change_arrows'
                arrow.id = marker_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                from geometry_msgs.msg import Point
                start = Point()
                start.x = change['previous_position'][0]
                start.y = change['previous_position'][1]
                start.z = change['previous_position'][2]
                end = Point()
                end.x = change['current_position'][0]
                end.y = change['current_position'][1]
                end.z = change['current_position'][2]
                arrow.points = [start, end]

                arrow.scale.x = 0.03  # shaft diameter
                arrow.scale.y = 0.06  # head diameter
                arrow.scale.z = 0.08  # head length
                arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
                arrow.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                markers.markers.append(arrow)

            marker_id += 1

        self.change_marker_pub.publish(markers)
        self.summary_marker_pub.publish(labels)


def main(args=None):
    rclpy.init(args=args)
    node = ChangeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Injects colored marker spheres into an RTAB-Map exported PLY file at
inspection detection positions.

Reads the original PLY (any format), adds colored marker spheres at each
detection position, and writes a clean PLY with x,y,z,red,green,blue.

Usage:
    # Add markers from latest inspection log:
    python3 ply_marker_injector.py --ply rtabmap_cloud.ply

    # Specify log and output:
    python3 ply_marker_injector.py --ply rtabmap_cloud.ply \
        --log ~/inspection_logs/run_xxx.json \
        -o building_with_markers.ply

    # Adjust marker size:
    python3 ply_marker_injector.py --ply rtabmap_cloud.ply --radius 0.15
"""

import argparse
import json
import math
import os
import struct
import sys


# Color scheme (RGB 0-255) matching inspection_node.py / change_detector.py
CHANGE_COLORS = {
    'NEW':       (0, 0, 255),       # Blue
    'MISSING':   (255, 0, 0),       # Red
    'MOVED':     (255, 128, 0),     # Orange
    'UNCHANGED': (0, 200, 0),       # Green
}
DEFAULT_COLOR = (0, 200, 200)       # Cyan for unknown/no-baseline
CLOUD_COLOR = (180, 180, 180)       # Light gray for original cloud points


def generate_sphere_points(cx, cy, cz, radius, color, density=200):
    """Generate points on a sphere + inner shells for PLY injection."""
    r, g, b = color
    points = []
    golden_ratio = (1 + math.sqrt(5)) / 2

    for frac in [1.0, 0.6, 0.3]:
        shell_r = radius * frac
        n = max(int(density * frac * frac), 20)
        for i in range(n):
            theta = math.acos(1 - 2 * (i + 0.5) / n)
            phi = 2 * math.pi * i / golden_ratio
            x = cx + shell_r * math.sin(theta) * math.cos(phi)
            y = cy + shell_r * math.sin(theta) * math.sin(phi)
            z = cz + shell_r * math.cos(theta)
            points.append((x, y, z, r, g, b))

    return points


def generate_stem_points(cx, cy, cz, color, height=0.3):
    """Generate a vertical line above the marker."""
    r, g, b = color
    return [(cx, cy, cz + height * i / 20, r, g, b) for i in range(20)]


def parse_ply(filepath):
    """Parse any PLY file, return list of (x, y, z) tuples and metadata."""
    with open(filepath, 'rb') as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace').strip()
            header_lines.append(line)
            if line == 'end_header':
                break

        data_offset = f.tell()

        # Determine format
        is_ascii = any('format ascii' in l for l in header_lines)
        is_binary_le = any('format binary_little_endian' in l
                           for l in header_lines)
        is_binary_be = any('format binary_big_endian' in l
                           for l in header_lines)

        # Parse elements and their properties
        elements = []
        current_element = None
        for line in header_lines:
            if line.startswith('element'):
                parts = line.split()
                current_element = {
                    'name': parts[1],
                    'count': int(parts[2]),
                    'properties': []
                }
                elements.append(current_element)
            elif line.startswith('property') and current_element is not None:
                parts = line.split()
                if parts[1] == 'list':
                    # list property (e.g., face vertex_indices)
                    current_element['properties'].append({
                        'type': 'list',
                        'count_type': parts[2],
                        'item_type': parts[3],
                        'name': parts[4]
                    })
                else:
                    current_element['properties'].append({
                        'type': parts[1],
                        'name': parts[2]
                    })

        # Find vertex element
        vertex_elem = None
        for elem in elements:
            if elem['name'] == 'vertex':
                vertex_elem = elem
                break

        if vertex_elem is None:
            print("Error: no vertex element in PLY")
            sys.exit(1)

        # Build struct format for vertex
        type_map = {
            'float': ('f', 4), 'float32': ('f', 4),
            'double': ('d', 8), 'float64': ('d', 8),
            'uchar': ('B', 1), 'uint8': ('B', 1),
            'char': ('b', 1), 'int8': ('b', 1),
            'short': ('h', 2), 'int16': ('h', 2),
            'ushort': ('H', 2), 'uint16': ('H', 2),
            'int': ('i', 4), 'int32': ('i', 4),
            'uint': ('I', 4), 'uint32': ('I', 4),
        }

        endian = '<' if is_binary_le else '>'
        vertex_fmt = endian
        vertex_size = 0
        prop_names = []
        for prop in vertex_elem['properties']:
            if prop['type'] == 'list':
                continue  # skip list properties in vertex (unusual)
            fmt_char, size = type_map.get(prop['type'], ('f', 4))
            vertex_fmt += fmt_char
            vertex_size += size
            prop_names.append(prop['name'])

        # Find x, y, z indices
        x_idx = prop_names.index('x') if 'x' in prop_names else 0
        y_idx = prop_names.index('y') if 'y' in prop_names else 1
        z_idx = prop_names.index('z') if 'z' in prop_names else 2

        # Find color indices if they exist
        has_r = 'red' in prop_names
        r_idx = prop_names.index('red') if has_r else -1
        g_idx = prop_names.index('green') if 'green' in prop_names else -1
        b_idx = prop_names.index('blue') if 'blue' in prop_names else -1

        # Read vertices
        vertices = []
        count = vertex_elem['count']

        if is_ascii:
            f.seek(data_offset)
            for _ in range(count):
                line = f.readline().decode('ascii', errors='replace').strip()
                parts = line.split()
                x = float(parts[x_idx])
                y = float(parts[y_idx])
                z = float(parts[z_idx])
                if has_r:
                    r = int(float(parts[r_idx]))
                    g = int(float(parts[g_idx]))
                    b = int(float(parts[b_idx]))
                    vertices.append((x, y, z, r, g, b))
                else:
                    vertices.append((x, y, z, *CLOUD_COLOR))
        else:
            f.seek(data_offset)
            for _ in range(count):
                raw = f.read(vertex_size)
                if len(raw) < vertex_size:
                    break
                vals = struct.unpack(vertex_fmt, raw)
                x, y, z = vals[x_idx], vals[y_idx], vals[z_idx]
                if has_r:
                    r = int(vals[r_idx])
                    g = int(vals[g_idx])
                    b = int(vals[b_idx])
                    vertices.append((x, y, z, r, g, b))
                else:
                    vertices.append((x, y, z, *CLOUD_COLOR))

    return vertices


def write_ply(filepath, vertices):
    """Write a clean binary little-endian PLY with x,y,z,red,green,blue."""
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    fmt = '<fffBBB'  # 15 bytes per vertex

    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        for x, y, z, r, g, b in vertices:
            f.write(struct.pack(fmt, x, y, z,
                                min(255, max(0, int(r))),
                                min(255, max(0, int(g))),
                                min(255, max(0, int(b)))))


def find_latest_log():
    """Find the most recent inspection log."""
    log_dir = os.path.expanduser('~/inspection_logs')
    if not os.path.isdir(log_dir):
        return None
    logs = sorted([
        os.path.join(log_dir, f) for f in os.listdir(log_dir)
        if f.endswith('.json') and not f.startswith('changes_')
    ], key=os.path.getmtime)
    return logs[-1] if logs else None


def inject_markers(ply_path, log_path, output_path, radius=0.1, density=200):
    """Inject colored marker spheres into PLY at detection positions."""

    with open(log_path) as f:
        data = json.load(f)

    detections = data.get('detections', [])
    if not detections:
        print("No detections found in log.")
        return

    # Parse original PLY
    print(f"Reading {ply_path}...")
    vertices = parse_ply(ply_path)
    orig_count = len(vertices)
    print(f"  {orig_count} original points")

    # Generate marker points
    marker_points = []
    valid_detections = 0
    for det in detections:
        pos = det.get('map_position')
        if not pos or len(pos) < 3:
            continue

        valid_detections += 1
        change_type = det.get('change_type', '')
        color = CHANGE_COLORS.get(change_type, DEFAULT_COLOR)

        sphere_pts = generate_sphere_points(
            pos[0], pos[1], pos[2], radius, color, density)
        marker_points.extend(sphere_pts)

        stem_pts = generate_stem_points(
            pos[0], pos[1], pos[2] + radius, color)
        marker_points.extend(stem_pts)

    if not marker_points:
        print("No valid marker positions found.")
        return

    # Combine and write
    all_vertices = vertices + marker_points
    print(f"Writing {output_path}...")
    write_ply(output_path, all_vertices)

    print(f"Done!")
    print(f"  Original points: {orig_count}")
    print(f"  Marker points:   {len(marker_points)}")
    print(f"  Total points:    {len(all_vertices)}")
    print(f"  Detections:      {valid_detections}")

    for det in detections:
        pos = det.get('map_position', [0, 0, 0])
        change = det.get('change_type', 'N/A')
        label = det.get('label', '?')
        score = det.get('score', 0)
        print(f"    [{change:10s}] {label} ({score:.0%}) at "
              f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")


def main():
    parser = argparse.ArgumentParser(
        description='Inject inspection markers into RTAB-Map PLY export')
    parser.add_argument('--ply', required=True,
                        help='Input PLY file from rtabmap-export')
    parser.add_argument('--log', '-l',
                        help='Inspection JSON log (default: latest)')
    parser.add_argument('--output', '-o',
                        help='Output PLY path (default: <input>_markers.ply)')
    parser.add_argument('--radius', type=float, default=0.1,
                        help='Marker sphere radius in meters (default: 0.1)')
    parser.add_argument('--density', type=int, default=200,
                        help='Points per marker sphere (default: 200)')
    args = parser.parse_args()

    ply_path = os.path.expanduser(args.ply)
    if not os.path.isfile(ply_path):
        print(f"Error: PLY file not found: {ply_path}")
        sys.exit(1)

    log_path = args.log
    if not log_path:
        log_path = find_latest_log()
        if log_path:
            print(f"Auto-detected latest log: {log_path}")
        else:
            print("Error: no inspection log found. Use --log to specify one.")
            sys.exit(1)

    log_path = os.path.expanduser(log_path)
    if not os.path.isfile(log_path):
        print(f"Error: log file not found: {log_path}")
        sys.exit(1)

    output_path = args.output
    if not output_path:
        base, ext = os.path.splitext(ply_path)
        output_path = f"{base}_markers{ext}"

    inject_markers(ply_path, log_path, output_path, args.radius, args.density)


if __name__ == '__main__':
    main()

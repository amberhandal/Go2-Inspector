#!/usr/bin/env python3
"""
Inspection Report Generator for Go2 Robot

Compares two inspection run logs and generates a human-readable PDF report
with change detection summary, detailed object listings, and position data.

Usage:
    # Compare two specific runs:
    python3 inspection_report.py \
        --previous ~/inspection_logs/run_001.json \
        --current  ~/inspection_logs/run_002.json

    # Auto-detect the two most recent runs:
    python3 inspection_report.py

    # Specify output path:
    python3 inspection_report.py -o ~/reports/inspection_report.pdf

Dependencies:
    pip install fpdf2
"""

import argparse
import json
import os
import sys
from datetime import datetime
from typing import List, Dict

from fpdf import FPDF


# ============================================================================
# Change Detection (same logic as change_detector.py / inspection_node.py)
# ============================================================================

def compare_runs(
    current_data: Dict,
    previous_data: Dict,
    match_distance: float = 0.8,
    unchanged_threshold: float = 0.35,
) -> List[Dict]:
    """Compare two inspection runs and classify changes."""
    current_dets = current_data.get('detections', [])
    previous_dets = previous_data.get('detections', [])
    changes = []

    prev_matched = [False] * len(previous_dets)

    for curr in current_dets:
        best_idx = None
        best_dist = float('inf')

        for i, prev in enumerate(previous_dets):
            if prev['label'] != curr['label']:
                continue
            dx = curr['map_position'][0] - prev['map_position'][0]
            dy = curr['map_position'][1] - prev['map_position'][1]
            dz = curr['map_position'][2] - prev['map_position'][2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx is not None and best_dist < match_distance:
            prev_matched[best_idx] = True
            prev = previous_dets[best_idx]
            change_type = 'UNCHANGED' if best_dist < unchanged_threshold \
                else 'MOVED'
            changes.append({
                'type': change_type,
                'label': curr['label'],
                'current_position': curr['map_position'],
                'previous_position': prev['map_position'],
                'distance': best_dist,
                'current_score': curr['score'],
                'previous_score': prev.get('score', 0),
                'sightings': curr.get('sightings', 1),
            })
        else:
            changes.append({
                'type': 'NEW',
                'label': curr['label'],
                'current_position': curr['map_position'],
                'current_score': curr['score'],
                'sightings': curr.get('sightings', 1),
            })

    for i, prev in enumerate(previous_dets):
        if not prev_matched[i]:
            changes.append({
                'type': 'MISSING',
                'label': prev['label'],
                'previous_position': prev['map_position'],
                'previous_score': prev.get('score', 0),
            })

    return changes


# ============================================================================
# PDF Report
# ============================================================================

# Colors for change types (R, G, B)
CHANGE_COLORS = {
    'NEW':       (0, 100, 220),
    'MISSING':   (220, 30, 30),
    'MOVED':     (230, 130, 0),
    'UNCHANGED': (30, 160, 30),
}


class InspectionReport(FPDF):
    def __init__(self, title: str = "Go2 Inspection Report"):
        super().__init__()
        self.report_title = title
        self.set_auto_page_break(auto=False)
        self.set_margins(8, 8, 8)

    def header(self):
        self.set_font('Helvetica', 'B', 8)
        self.set_text_color(100, 100, 100)
        self.cell(0, 5, self.report_title, align='L')
        self.ln(2)
        self.set_draw_color(200, 200, 200)
        self.line(8, self.get_y(), 202, self.get_y())
        self.ln(2)

    def footer(self):
        self.set_y(-15)
        self.set_font('Helvetica', 'I', 8)
        self.set_text_color(150, 150, 150)
        self.cell(0, 10, f'Page {self.page_no()}/{{nb}}', align='C')


def fmt_pos(pos: List[float]) -> str:
    """Format a 3D position for display."""
    return f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"


def get_recommendations(changes: List[Dict]) -> List[str]:
    """Generate deterministic recommendations from inspection changes."""
    recs = []
    missing = [c for c in changes if c['type'] == 'MISSING']
    moved = [c for c in changes if c['type'] == 'MOVED']

    if missing:
        label_counts = {}
        for c in missing:
            label_counts[c['label']] = label_counts.get(c['label'], 0) + 1
        for lbl, cnt in label_counts.items():
            recs.append(
                f'{cnt} "{lbl}" not found in current run. '
                f'Verify presence or confirm removal.')

    for item in moved:
        recs.append(
            f'"{item["label"]}" moved {item["distance"]:.2f}m. '
            f'Verify relocation was intentional.')

    return recs


def _render_recommendations(pdf, recommendations: List[str]):
    """Render a list of recommendation strings into the PDF."""
    pdf.set_font('Helvetica', 'B', 10)
    pdf.set_text_color(30, 30, 30)
    pdf.cell(0, 6, 'Recommendations', new_x='LMARGIN', new_y='NEXT')
    pdf.ln(1)

    pdf.set_text_color(50, 50, 50)
    pdf.set_font('Helvetica', '', 7)
    for i, rec in enumerate(recommendations, 1):
        pdf.set_x(pdf.l_margin)
        pdf.multi_cell(0, 4, f'{i}. {rec}')


def generate_report(
    current_data: Dict,
    previous_data: Dict,
    changes: List[Dict],
    output_path: str,
):
    """Generate a PDF inspection report."""
    pdf = InspectionReport()
    pdf.alias_nb_pages()
    pdf.add_page()

    # ── Title ──
    pdf.set_font('Helvetica', 'B', 14)
    pdf.set_text_color(30, 30, 30)
    pdf.cell(0, 8, 'Inspection Change Report', align='C', new_x='LMARGIN',
             new_y='NEXT')
    pdf.set_font('Helvetica', '', 8)
    pdf.set_text_color(100, 100, 100)
    pdf.cell(0, 4,
             f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
             align='C', new_x='LMARGIN', new_y='NEXT')
    pdf.ln(3)

    # ── Run Information ──
    pdf.set_font('Helvetica', 'B', 10)
    pdf.set_text_color(30, 30, 30)
    pdf.cell(0, 6, 'Run Information', new_x='LMARGIN', new_y='NEXT')
    pdf.ln(1)

    prev_id = previous_data.get('run_id', 'unknown')
    curr_id = current_data.get('run_id', 'unknown')
    prev_ts = previous_data.get('timestamp', 'unknown')
    curr_ts = current_data.get('timestamp', 'unknown')
    prev_n = previous_data.get('num_detections', 0)
    curr_n = current_data.get('num_detections', 0)
    prompts = current_data.get('prompts', [])

    info_rows = [
        ('', 'Baseline (Previous)', 'Current Run'),
        ('Run ID', prev_id, curr_id),
        ('Timestamp', prev_ts[:19] if len(prev_ts) > 19 else prev_ts,
         curr_ts[:19] if len(curr_ts) > 19 else curr_ts),
        ('Objects Detected', str(prev_n), str(curr_n)),
    ]

    col_w = [40, 75, 75]
    pdf.set_font('Helvetica', 'B', 7)
    pdf.set_fill_color(240, 240, 240)
    for j, val in enumerate(info_rows[0]):
        pdf.cell(col_w[j], 5, val, border=1, fill=True)
    pdf.ln()

    pdf.set_font('Helvetica', '', 7)
    for row in info_rows[1:]:
        pdf.set_font('Helvetica', 'B', 7)
        pdf.cell(col_w[0], 5, row[0], border=1)
        pdf.set_font('Helvetica', '', 7)
        pdf.cell(col_w[1], 5, row[1], border=1)
        pdf.cell(col_w[2], 5, row[2], border=1)
        pdf.ln()

    pdf.set_font('Helvetica', '', 7)
    pdf.set_text_color(80, 80, 80)
    pdf.cell(0, 4, f'Search prompts: {", ".join(prompts)}',
             new_x='LMARGIN', new_y='NEXT')
    pdf.ln(3)

    # ── Summary ──
    n_new = sum(1 for c in changes if c['type'] == 'NEW')
    n_missing = sum(1 for c in changes if c['type'] == 'MISSING')
    n_moved = sum(1 for c in changes if c['type'] == 'MOVED')
    n_unchanged = sum(1 for c in changes if c['type'] == 'UNCHANGED')
    total = len(changes)

    pdf.set_font('Helvetica', 'B', 10)
    pdf.set_text_color(30, 30, 30)
    pdf.cell(0, 6, 'Change Summary', new_x='LMARGIN', new_y='NEXT')
    pdf.ln(1)

    summary_items = [
        ('UNCHANGED', n_unchanged),
        ('MOVED', n_moved),
        ('NEW', n_new),
        ('MISSING', n_missing),
    ]

    for label, count in summary_items:
        r, g, b = CHANGE_COLORS[label]
        pdf.set_fill_color(r, g, b)
        x = pdf.get_x()
        y = pdf.get_y()
        pdf.rect(x, y + 0.5, 3, 3, style='F')
        pdf.set_x(x + 5)

        pdf.set_font('Helvetica', 'B', 8)
        pdf.set_text_color(r, g, b)
        pdf.cell(8, 4, str(count))

        pdf.set_font('Helvetica', '', 8)
        pdf.set_text_color(30, 30, 30)
        pdf.cell(0, 4, label, new_x='LMARGIN', new_y='NEXT')

    pdf.ln(1)
    pdf.set_font('Helvetica', 'B', 8)
    pdf.set_text_color(30, 30, 30)
    pdf.cell(0, 4, f'Total objects tracked: {total}', new_x='LMARGIN',
             new_y='NEXT')
    pdf.ln(3)

    # ── Detailed Changes ──
    # Group by type, showing actionable items first
    type_order = ['MISSING', 'MOVED', 'NEW', 'UNCHANGED']
    for change_type in type_order:
        items = [c for c in changes if c['type'] == change_type]
        if not items:
            continue

        r, g, b = CHANGE_COLORS[change_type]

        pdf.set_font('Helvetica', 'B', 9)
        pdf.set_text_color(r, g, b)
        pdf.cell(0, 5, f'{change_type} ({len(items)})', new_x='LMARGIN',
                 new_y='NEXT')
        pdf.ln(1)

        # Table header
        if change_type in ('MOVED', 'UNCHANGED'):
            cols = [('Object', 45), ('Previous Position', 50),
                    ('Current Position', 50), ('Distance', 20),
                    ('Score', 18)]
        elif change_type == 'NEW':
            cols = [('Object', 50), ('Position', 55), ('Score', 20),
                    ('Sightings', 25)]
        else:  # MISSING
            cols = [('Object', 50), ('Last Known Position', 60),
                    ('Score', 20)]

        pdf.set_font('Helvetica', 'B', 7)
        pdf.set_fill_color(r, g, b)
        pdf.set_text_color(255, 255, 255)
        for col_name, col_width in cols:
            pdf.cell(col_width, 4, col_name, border=1, fill=True)
        pdf.ln()

        # Table rows
        pdf.set_font('Helvetica', '', 7)
        pdf.set_text_color(30, 30, 30)
        for i, item in enumerate(items):
            fill = i % 2 == 0
            if fill:
                pdf.set_fill_color(248, 248, 248)

            if change_type in ('MOVED', 'UNCHANGED'):
                pdf.cell(45, 4, item['label'], border=1, fill=fill)
                pdf.cell(50, 4, fmt_pos(item['previous_position']),
                         border=1, fill=fill)
                pdf.cell(50, 4, fmt_pos(item['current_position']),
                         border=1, fill=fill)
                pdf.cell(20, 4, f"{item['distance']:.2f}m", border=1,
                         fill=fill)
                pdf.cell(18, 4, f"{item['current_score']:.0%}", border=1,
                         fill=fill)
            elif change_type == 'NEW':
                pdf.cell(50, 4, item['label'], border=1, fill=fill)
                pdf.cell(55, 4, fmt_pos(item['current_position']),
                         border=1, fill=fill)
                pdf.cell(20, 4, f"{item['current_score']:.0%}", border=1,
                         fill=fill)
                pdf.cell(25, 4, str(item.get('sightings', 1)), border=1,
                         fill=fill)
            else:  # MISSING
                pdf.cell(50, 4, item['label'], border=1, fill=fill)
                pdf.cell(60, 4, fmt_pos(item['previous_position']),
                         border=1, fill=fill)
                pdf.cell(20, 4, f"{item.get('previous_score', 0):.0%}",
                         border=1, fill=fill)
            pdf.ln()

        pdf.ln(3)

    # ── Recommendations ──
    recommendations = get_recommendations(changes)
    if recommendations:
        # Start new page if less than 25mm remain
        if pdf.get_y() > 272:
            pdf.add_page()
        _render_recommendations(pdf, recommendations)

    # Save
    pdf.output(output_path)
    print(f"Report saved to: {output_path}")


# ============================================================================
# Main
# ============================================================================

def auto_detect_logs(log_dir: str):
    """Find the two most recent log files."""
    if not os.path.isdir(log_dir):
        print(f"Error: Log directory not found: {log_dir}")
        sys.exit(1)

    logs = sorted([
        os.path.join(log_dir, f)
        for f in os.listdir(log_dir)
        if f.endswith('.json') and not f.startswith('changes_')
    ], key=os.path.getmtime)

    if len(logs) < 2:
        print(f"Error: Need at least 2 log files, found {len(logs)}")
        sys.exit(1)

    return logs[-2], logs[-1]


def main():
    parser = argparse.ArgumentParser(
        description='Generate PDF inspection report comparing two runs')
    parser.add_argument('--previous', '-p', type=str, default='',
                        help='Path to previous (baseline) run log')
    parser.add_argument('--current', '-c', type=str, default='',
                        help='Path to current run log')
    parser.add_argument('--output', '-o', type=str, default='',
                        help='Output PDF path')
    parser.add_argument('--log-dir', type=str,
                        default=os.path.expanduser('~/inspection_logs'),
                        help='Directory containing inspection logs')
    parser.add_argument('--match-distance', type=float, default=0.8,
                        help='Max distance to match objects across runs')
    parser.add_argument('--unchanged-threshold', type=float, default=0.35,
                        help='Max distance to consider unchanged')
    args = parser.parse_args()

    # Resolve log paths
    if args.previous and args.current:
        prev_path, curr_path = args.previous, args.current
    else:
        prev_path, curr_path = auto_detect_logs(args.log_dir)
        print("Auto-detected logs:")
        print(f"  Previous: {prev_path}")
        print(f"  Current:  {curr_path}")

    # Load logs
    with open(prev_path, 'r') as f:
        previous_data = json.load(f)
    with open(curr_path, 'r') as f:
        current_data = json.load(f)

    # Compare
    changes = compare_runs(
        current_data, previous_data,
        match_distance=args.match_distance,
        unchanged_threshold=args.unchanged_threshold,
    )

    # Output path
    if args.output:
        output_path = args.output
    else:
        prev_id = previous_data.get('run_id', 'prev')
        curr_id = current_data.get('run_id', 'curr')
        output_path = os.path.join(
            args.log_dir, f'report_{prev_id}_vs_{curr_id}.pdf')

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Generate
    generate_report(current_data, previous_data, changes, output_path)

    # Summary
    n_new = sum(1 for c in changes if c['type'] == 'NEW')
    n_missing = sum(1 for c in changes if c['type'] == 'MISSING')
    n_moved = sum(1 for c in changes if c['type'] == 'MOVED')
    n_unchanged = sum(1 for c in changes if c['type'] == 'UNCHANGED')
    print(f"\nSummary: {n_unchanged} unchanged, {n_new} new, "
          f"{n_moved} moved, {n_missing} missing")


if __name__ == '__main__':
    main()

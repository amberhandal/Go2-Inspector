#!/usr/bin/env python3
"""
LLM-powered Inspection Report Generator for Go2 Robot

Same PDF layout as inspection_report.py, but uses Claude to generate
the recommendations section instead of deterministic text.

Requires:
    pip install fpdf2 anthropic
    export ANTHROPIC_API_KEY=sk-ant-...

Usage:
    python3 inspection_report_llm.py
    python3 inspection_report_llm.py --previous run_001.json --current run_002.json
"""
# TODO: Test this!
import argparse
import json
import os
import sys
from typing import List, Dict

import anthropic

from inspection_report import (
    compare_runs,
    fmt_pos,
    generate_report,
    get_recommendations,
    _render_recommendations,
    auto_detect_logs,
)


def get_recommendations_llm(changes: List[Dict]) -> List[str]:
    """Use Claude to generate recommendations from inspection changes."""
    missing = [c for c in changes if c['type'] == 'MISSING']
    moved = [c for c in changes if c['type'] == 'MOVED']

    if not missing and not moved:
        return []

    summary_lines = []
    for item in missing:
        summary_lines.append(
            f"MISSING: \"{item['label']}\" last seen at "
            f"{fmt_pos(item['previous_position'])}")
    for item in moved:
        summary_lines.append(
            f"MOVED: \"{item['label']}\" moved {item['distance']:.2f}m "
            f"from {fmt_pos(item['previous_position'])} "
            f"to {fmt_pos(item['current_position'])}")

    prompt = (
        "You are writing the recommendations section of a building safety "
        "inspection report. Based on the following changes detected between "
        "two inspection runs, write actionable recommendations.\n\n"
        f"Changes:\n" + "\n".join(summary_lines) + "\n\n"
        "Rules:\n"
        "- Return a JSON array of strings, one per recommendation\n"
        "- Each recommendation should be one concise sentence\n"
        "- Focus on what action the facility manager should take\n"
        "- Do not include numbering, bullets, or prefixes\n"
        "- Maximum 5 recommendations\n"
    )

    try:
        client = anthropic.Anthropic()
        message = client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=300,
            messages=[{"role": "user", "content": prompt}],
        )
        return json.loads(message.content[0].text)
    except Exception as e:
        print(f"LLM recommendation generation failed: {e}")
        print("Falling back to deterministic recommendations.")
        return get_recommendations(changes)


def generate_report_llm(
    current_data: Dict,
    previous_data: Dict,
    changes: List[Dict],
    output_path: str,
):
    """Generate PDF report with LLM-powered recommendations.

    Reuses the base generate_report for the full layout, but overrides
    the recommendations with LLM-generated text.
    """
    # Generate LLM recommendations before building the PDF
    recommendations = get_recommendations_llm(changes)

    import inspection_report
    original_fn = inspection_report.get_recommendations
    inspection_report.get_recommendations = lambda changes: recommendations
    try:
        generate_report(current_data, previous_data, changes, output_path)
    finally:
        inspection_report.get_recommendations = original_fn


def main():
    parser = argparse.ArgumentParser(
        description='Generate LLM-powered PDF inspection report')
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
        print(f"Auto-detected logs:")
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
            args.log_dir, f'report_llm_{prev_id}_vs_{curr_id}.pdf')

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Generate with LLM recommendations
    generate_report_llm(current_data, previous_data, changes, output_path)

    # Summary
    n_new = sum(1 for c in changes if c['type'] == 'NEW')
    n_missing = sum(1 for c in changes if c['type'] == 'MISSING')
    n_moved = sum(1 for c in changes if c['type'] == 'MOVED')
    n_unchanged = sum(1 for c in changes if c['type'] == 'UNCHANGED')
    print(f"\nSummary: {n_unchanged} unchanged, {n_new} new, "
          f"{n_moved} moved, {n_missing} missing")


if __name__ == '__main__':
    main()

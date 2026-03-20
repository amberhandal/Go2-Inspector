#!/usr/bin/env python3
"""
Watchdog Run - Full inspection lifecycle manager.

Launches SLAM + navigation + inspection, and on Ctrl+C:
1. Saves the 2D occupancy grid (while ROS is still alive)
2. Shuts down all nodes
3. Exports 3D PLY from RTAB-Map database
4. Injects inspection markers into PLY
5. Generates building floor plan with markers
6. Generates inspection comparison report (PDF)
7. Copies RTAB-Map database
8. Collects everything into ~/watchdog_runs/run_YYYYMMDD_HHMMSS/

Usage:
    python3 watchdog_run.py [launch args...]

    # Basic SLAM + inspection:
    python3 watchdog_run.py use_inspection:=true use_explore:=true

    # With baseline comparison:
    python3 watchdog_run.py use_inspection:=true use_explore:=true \
        baseline_log:=~/inspection_logs/run_xxx.json

    # Custom database path:
    python3 watchdog_run.py database_path:=/tmp/my_run.db
"""

import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
WATCHDOG_DIR = os.path.expanduser('~/watchdog_runs')
INSPECTION_LOGS = os.path.expanduser('~/inspection_logs')
DEFAULT_DB = os.path.expanduser('~/.ros/rtabmap.db')


def find_latest_log(after_time=None):
    """Find the most recent inspection log, optionally after a time."""
    if not os.path.isdir(INSPECTION_LOGS):
        return None
    logs = sorted([
        os.path.join(INSPECTION_LOGS, f)
        for f in os.listdir(INSPECTION_LOGS)
        if f.endswith('.json') and not f.startswith('changes_')
    ], key=os.path.getmtime)
    if after_time:
        logs = [l for l in logs
                if os.path.getmtime(l) >= after_time]
    return logs[-1] if logs else None


def find_previous_log(current_log):
    """Find the second-most-recent log (for comparison report)."""
    if not os.path.isdir(INSPECTION_LOGS):
        return None
    logs = sorted([
        os.path.join(INSPECTION_LOGS, f)
        for f in os.listdir(INSPECTION_LOGS)
        if f.endswith('.json') and not f.startswith('changes_')
    ], key=os.path.getmtime)
    if current_log and current_log in logs:
        idx = logs.index(current_log)
        if idx > 0:
            return logs[idx - 1]
    elif len(logs) >= 2:
        return logs[-2]
    return None


def parse_db_path(launch_args):
    """Extract database_path from launch args, or use default."""
    for arg in launch_args:
        if arg.startswith('database_path:='):
            return os.path.expanduser(arg.split(':=', 1)[1])
    return DEFAULT_DB


def run_cmd(cmd, desc, cwd=None, timeout=120):
    """Run a command, print status."""
    print(f"  [{desc}] {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout, cwd=cwd)
        if result.returncode != 0:
            print(f"    Warning: {desc} returned {result.returncode}")
            if result.stderr:
                for line in result.stderr.strip().split('\n')[-3:]:
                    print(f"    stderr: {line}")
        return result
    except subprocess.TimeoutExpired:
        print(f"    Warning: {desc} timed out after {timeout}s")
        return None
    except FileNotFoundError as e:
        print(f"    Warning: {desc} failed: {e}")
        return None


def save_map_while_running(run_dir):
    """Save the /map topic using map_saver_cli while ROS is alive."""
    map_path = os.path.join(run_dir, 'map')
    result = run_cmd(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
         '-f', map_path, '--ros-args', '-p',
         'save_map_timeout:=5.0'],
        'Save 2D map', timeout=15)
    pgm = map_path + '.pgm'
    yaml = map_path + '.yaml'
    if os.path.isfile(pgm):
        print(f"    Saved: {pgm}")
        return pgm, yaml
    print("    Map save failed (RTAB-Map may not be publishing /map)")
    return None, None


def main():
    launch_args = sys.argv[1:]
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(WATCHDOG_DIR, f'run_{ts}')
    os.makedirs(run_dir, exist_ok=True)

    db_path = parse_db_path(launch_args)

    print("=" * 60)
    print(f"  WATCHDOG RUN - {ts}")
    print(f"  Output: {run_dir}")
    print(f"  DB: {db_path}")
    print("=" * 60)
    print()

    # Record start time (to find the inspection log from this run)
    start_time = time.time()

    # Save launch command for reference
    with open(os.path.join(run_dir, 'launch_command.txt'), 'w') as f:
        f.write(f"ros2 launch go2_navigation "
                f"slam_nav_rtabmap.launch.xml "
                f"{' '.join(launch_args)}\n")
        f.write(f"Started: {ts}\n")

    # Launch the ROS system
    launch_cmd = [
        'ros2', 'launch', 'go2_navigation',
        'slam_nav_rtabmap.launch.xml'
    ] + launch_args

    print(f"Launching: {' '.join(launch_cmd)}")
    print("Press Ctrl+C to stop and export...\n")

    # Use os.setsid so child is in its own process group (won't get
    # our SIGINT). We'll send signals to it explicitly during cleanup.
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    # Use a flag + signal handler instead of KeyboardInterrupt,
    # because KeyboardInterrupt delivery during wait() is unreliable.
    stop_requested = False

    def sigint_handler(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, sigint_handler)

    # Poll instead of blocking wait so we can check the flag
    while not stop_requested:
        ret = launch_proc.poll()
        if ret is not None:
            print("\nLaunch exited on its own.")
            break
        time.sleep(0.2)

    user_stopped = stop_requested
    pgm_path, yaml_path = None, None

    if user_stopped:
        # Ignore SIGINT during cleanup so repeated Ctrl+C can't
        # abort the export pipeline
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        print("\n")
        print("=" * 60)
        print("  STOPPING - Saving data before shutdown...")
        print("=" * 60)

        # Step 1: Save map while ROS is still alive
        print("\n[1/7] Saving 2D occupancy grid...")
        pgm_path, yaml_path = save_map_while_running(run_dir)

        # Step 2: Kill the launch
        print("\n[2/7] Shutting down ROS nodes...")
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(launch_proc.pid),
                          signal.SIGKILL)
                launch_proc.wait(timeout=5)
            except Exception:
                pass
        print("    Nodes stopped.")

    # Give filesystem a moment to flush
    time.sleep(1)

    # Find the inspection log from this run
    current_log = find_latest_log(after_time=start_time)
    if current_log:
        print(f"\nInspection log: {current_log}")
    else:
        print("\nNo inspection log found for this run.")

    # Step 3: Copy RTAB-Map database
    print("\n[3/7] Copying RTAB-Map database...")
    db_dest = os.path.join(run_dir, 'rtabmap.db')
    if os.path.isfile(db_path):
        shutil.copy2(db_path, db_dest)
        print(f"    Saved: {db_dest}")
        sz = os.path.getsize(db_dest) / (1024 * 1024)
        print(f"    Size: {sz:.1f} MB")
    else:
        print(f"    Warning: {db_path} not found")
        db_dest = None

    # Step 4: Export 3D PLY
    print("\n[4/7] Exporting 3D point cloud (PLY)...")
    ply_path = os.path.join(run_dir, 'map_3d')
    if db_dest:
        run_cmd(
            ['rtabmap-export', '--scan', '--output',
             ply_path, db_dest],
            'Export PLY', cwd=run_dir)
        # rtabmap-export names the output based on --output
        ply_file = ply_path + '.ply'
        if not os.path.isfile(ply_file):
            # Try alternate naming
            for f in os.listdir(run_dir):
                if f.endswith('.ply') and 'marker' not in f:
                    ply_file = os.path.join(run_dir, f)
                    break
        if os.path.isfile(ply_file):
            print(f"    Saved: {ply_file}")
        else:
            print("    Warning: PLY export produced no file")
            ply_file = None
    else:
        ply_file = None

    # Step 5: Inject markers into PLY
    print("\n[5/7] Adding inspection markers to PLY...")
    markers_ply = None
    if ply_file and current_log:
        markers_ply = os.path.join(run_dir, 'map_3d_markers.ply')
        injector = os.path.join(SCRIPTS_DIR,
                                'ply_marker_injector.py')
        run_cmd(
            ['python3', injector,
             '--ply', ply_file,
             '--log', current_log,
             '-o', markers_ply],
            'Inject markers')
        if os.path.isfile(markers_ply):
            print(f"    Saved: {markers_ply}")
        else:
            markers_ply = None
    else:
        print("    Skipped (no PLY or no inspection log)")

    # Step 6: Generate building floor plan
    print("\n[6/7] Generating 2D building plan...")
    map_export = os.path.join(SCRIPTS_DIR, 'map_export.py')
    if pgm_path and os.path.isfile(pgm_path):
        # Use saved map (best quality - real occupancy grid)
        plan_args = ['python3', map_export,
                     '--map', pgm_path,
                     '--map-yaml', yaml_path,
                     '-o', run_dir]
        if current_log:
            plan_args.extend(['--log', current_log])
        run_cmd(plan_args, 'Building plan from saved map')
    elif db_dest:
        # Fallback to database
        plan_args = ['python3', map_export,
                     '--db', db_dest,
                     '-o', run_dir]
        if current_log:
            plan_args.extend(['--log', current_log])
        run_cmd(plan_args, 'Building plan from database')
    else:
        print("    Skipped (no map data available)")

    # Step 7: Generate inspection comparison report
    print("\n[7/7] Generating inspection report...")
    if current_log:
        previous_log = find_previous_log(current_log)
        report_script = os.path.join(SCRIPTS_DIR,
                                     'inspection_report.py')
        report_path = os.path.join(run_dir,
                                   f'inspection_report_{ts}.pdf')
        report_args = ['python3', report_script,
                       '--current', current_log,
                       '-o', report_path]
        if previous_log:
            report_args.extend(['--previous', previous_log])
            print(f"    Comparing: {os.path.basename(previous_log)}"
                  f" vs {os.path.basename(current_log)}")
        else:
            print("    No previous run to compare against")
        run_cmd(report_args, 'Inspection report')
        if os.path.isfile(report_path):
            print(f"    Saved: {report_path}")
    else:
        print("    Skipped (no inspection log)")

    # Copy inspection log into run folder for self-containment
    if current_log:
        log_dest = os.path.join(run_dir,
                                os.path.basename(current_log))
        if not os.path.isfile(log_dest):
            shutil.copy2(current_log, log_dest)

    # Record end time
    end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(os.path.join(run_dir, 'launch_command.txt'), 'a') as f:
        f.write(f"Finished: {end_time}\n")

    # Summary
    print("\n" + "=" * 60)
    print("  RUN COMPLETE")
    print("=" * 60)
    print(f"\n  Output folder: {run_dir}/")
    print()
    for f in sorted(os.listdir(run_dir)):
        fpath = os.path.join(run_dir, f)
        sz = os.path.getsize(fpath)
        if sz > 1024 * 1024:
            sz_str = f"{sz / (1024*1024):.1f} MB"
        elif sz > 1024:
            sz_str = f"{sz / 1024:.0f} KB"
        else:
            sz_str = f"{sz} B"
        print(f"    {f:40s} {sz_str:>10s}")
    print()


if __name__ == '__main__':
    main()

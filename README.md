# Watchdog - Go2 Navigation & Inspection System
[Portfolio Link](https://www.amberhandal.com/projects/watchdog)

Autonomous building inspection system for the **Unitree Go2 quadruped robot**. The robot autonomously explores indoor environments using frontier-based exploration, builds 3D maps with RTAB-Map SLAM, and detects safety-critical objects (fire extinguishers, exit signs, etc.) using SAM 3 segmentation. After each run, the system exports annotated 2D floor plans, 3D point cloud maps with markers, and PDF comparison reports tracking changes between inspections.

Built on ROS 2 Kilted with Nav2, RTAB-Map, and a RealSense depth camera mounted on the Go2's head.

![SAM_Labeling](https://github.com/user-attachments/assets/3a59c5c3-a3bc-4718-8981-0c714aa0d8dc)

---

## Table of Contents

- [System Overview](#system-overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Running the System](#running-the-system)
  - [Basic SLAM + Navigation](#basic-slam--navigation)
  - [Autonomous Exploration](#autonomous-exploration)
  - [Full Inspection Mode](#full-inspection-mode)
  - [Watchdog Run (Recommended)](#watchdog-run-recommended)
- [Viewing Results](#viewing-results)
- [Configuration](#configuration)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)

---

## High-Level System Overview
![System_High-Level](https://github.com/user-attachments/assets/9a00c2bf-9203-49d5-88b7-02896099a1de)

**Key capabilities:**
- 3D SLAM using lidar point clouds (RTAB-Map with ICP registration)
- Autonomous frontier-based exploration
- Object detection via SAM 3 (Segment Anything Model 3)
- Live change detection against baseline runs
- Camera overlay with segmentation masks, bounding boxes, and labels
- Post-run export: annotated floor plans, 3D maps with markers, PDF reports

---

## Prerequisites

### ROS 2 Kilted

```bash
# Follow official installation: https://docs.ros.org/en/kilted/Installation.html
sudo apt install ros-kilted-desktop
```

### Unitree SDK 2

```bash
# Install vcstool if you don't have it
sudo apt install python3-vcstool

# Import Unitree dependencies
cd ~/go2_ws/src
vcs import < go2_navigation/deps.repos
```

### Python Dependencies

```bash
pip install opencv-python numpy reportlab pyyaml
```

### SAM 3 Server (for inspection mode)

The inspection system calls a SAM 3 HTTP API for object detection. You need a running SAM 3 server accessible over the network. The default URL is `http://129.105.69.11:8001` (configurable via `sam3_url` launch arg).

### CycloneDDS (Recommended)

```bash
sudo apt install ros-kilted-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Installation

```bash
# Create workspace (if not already done)
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src

# Clone this package
git clone <this-repo-url> go2_navigation

# Install all ROS 2 dependencies via rosdep
cd ~/go2_ws
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select go2_navigation

# Source the workspace
source ~/go2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Hardware Setup

### Go2 Robot

1. Power on the Go2 and connect to its network (WiFi or Ethernet)
2. The Go2 publishes on these key topics:
   - `/utlidar/cloud_deskewed` — 3D lidar point cloud
   - `/utlidar/robot_odom` — Robot odometry
   - `/lowstate` — Joint states
   - `/camera/color/image_raw` — RGB camera (RealSense on head)
   - `/camera/aligned_depth_to_color/image_raw` — Depth camera

3. Verify connectivity:
   ```bash
   ros2 topic list | grep utlidar
   ```

### Important: Lidar Transform

The UTLidar pitch for the **real robot** must be `2.8782 rad` in the URDF. The `-0.2 rad` value from `go2_description` is for **Gazebo simulation only**. Using the wrong pitch causes phantom obstacles and navigation failures.

---

## Running the System

### Basic SLAM + Navigation

Maps the environment with lidar SLAM. Manual goal-setting via RViz.

```bash
ros2 launch go2_navigation slam_nav_rtabmap.launch.xml
```

In RViz, use the "2D Goal Pose" tool to send navigation goals.

### Autonomous Exploration

Adds frontier-based exploration — the robot automatically seeks unexplored areas.

```bash
ros2 launch go2_navigation slam_nav_rtabmap.launch.xml \
  use_explore:=true
```

### Full Inspection Mode

Adds SAM 3 object detection on top of exploration. Detects fire extinguishers, exit signs, etc.

```bash
ros2 launch go2_navigation slam_nav_rtabmap.launch.xml \
  use_explore:=true \
  use_inspection:=true
```

**With baseline comparison** (detects NEW/MOVED/MISSING/UNCHANGED objects):

```bash
ros2 launch go2_navigation slam_nav_rtabmap.launch.xml \
  use_explore:=true \
  use_inspection:=true \
  baseline_log:=~/inspection_logs/run_20260309_230650.json
```

**Custom detection prompts:**

```bash
ros2 launch go2_navigation slam_nav_rtabmap.launch.xml \
  use_inspection:=true \
  inspection_prompts:='["fire extinguisher", "exit sign", "AED", "first aid kit"]'
```

### Watchdog Run (Recommended)

The **watchdog runner** wraps the full launch and automatically exports all results on Ctrl+C:

```bash
python3 ~/go2_ws/src/go2_navigation/scripts/watchdog_run.py \
  use_explore:=true \
  use_inspection:=true
```

On Ctrl+C, it automatically:
1. Saves the 2D occupancy grid (while ROS is still alive)
2. Shuts down all nodes
3. Copies the RTAB-Map database
4. Exports the 3D point cloud (PLY)
5. Injects inspection markers into the PLY
6. Generates the annotated 2D building plan (PNG)
7. Generates the inspection comparison report (PDF)

Everything is saved to `~/watchdog_runs/run_YYYYMMDD_HHMMSS/`.

---

## Viewing Results

### After a Watchdog Run

All outputs are in the timestamped run folder:

```
~/watchdog_runs/run_20260310_143000/
    rtabmap.db                  # RTAB-Map database (reloadable)
    map.pgm + map.yaml          # Raw 2D occupancy grid
    map_3d.ply                  # 3D lidar point cloud
    map_3d_markers.ply          # 3D cloud + colored detection markers
    building_plan_*.png         # Annotated 2D floor plan
    inspection_report_*.pdf     # Change comparison report
    run_*.json                  # Inspection detection log
    launch_command.txt          # Run metadata
```

### 2D Building Plan

```bash
# View the annotated floor plan
xdg-open ~/watchdog_runs/run_*/building_plan_*.png

# Or regenerate from a saved map:
python3 ~/go2_ws/src/go2_navigation/scripts/map_export.py \
  --map ~/watchdog_runs/run_*/map.pgm \
  --map-yaml ~/watchdog_runs/run_*/map.yaml \
  --log ~/inspection_logs/run_xxx.json
```

### 3D Point Cloud

```bash
# Install a viewer
sudo apt install cloudcompare

# View the 3D map with markers
cloudcompare.CloudCompare ~/watchdog_runs/run_*/map_3d_markers.ply
```

Marker colors in the PLY:
| Color  | Status    | Meaning                          |
|--------|-----------|----------------------------------|
| Blue   | NEW       | Object not in baseline           |
| Orange | MOVED     | Object moved from baseline pos   |
| Green  | UNCHANGED | Object at same location          |
| Red    | MISSING   | In baseline but not found        |
| Gray   | —         | Original lidar point cloud       |

### Inspection Report (PDF)

```bash
xdg-open ~/watchdog_runs/run_*/inspection_report_*.pdf
```

The report includes run metadata, change summary, detailed object tables, and recommendations.

### Manual Export (without Watchdog)

If you ran without the watchdog, export manually:

```bash
# Save 2D map (while SLAM is still running, in another terminal):
ros2 run go2_navigation map_export.py

# Export 3D PLY (after Ctrl+C):
rtabmap-export --scan ~/.ros/rtabmap.db

# Add markers to PLY:
python3 ~/go2_ws/src/go2_navigation/scripts/ply_marker_injector.py \
  --ply rtabmap_cloud.ply

# Generate comparison report:
python3 ~/go2_ws/src/go2_navigation/scripts/inspection_report.py
```

---

## Configuration

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz visualization |
| `use_rtabmap` | `true` | Enable RTAB-Map 3D SLAM |
| `use_nav2` | `true` | Enable Nav2 navigation stack |
| `use_explore` | `false` | Enable autonomous frontier exploration |
| `use_inspection` | `false` | Enable SAM 3 object detection |
| `use_camera` | `false` | Launch RealSense camera node locally |
| `use_camera_restampers` | `true` | Restamp camera topics |
| `use_visual_slam` | `false` | Enable RGB-D + lidar SLAM (experimental) |
| `use_rtabmap_viz` | `false` | Launch RTAB-Map visualization GUI |
| `use_pointcloud_refiner` | `true` | Filter/refine lidar point clouds |
| `database_path` | `~/.ros/rtabmap.db` | RTAB-Map database path |
| `sam3_url` | `http://129.105.69.11:8001` | SAM 3 server URL |
| `inspection_prompts` | `["fire extinguisher", "exit sign"]` | Objects to detect |
| `baseline_log` | `""` | Previous run log for change detection |
| `inspection_overlay` | `true` | Publish annotated camera image |
| `nav2_params` | `nav2_params_rtabmap.yaml` | Nav2 parameter file |
| `rviz_config` | `nav2_view.rviz` | RViz config file |

### Nav2 Parameters

Edit `config/nav2_params_rtabmap.yaml` for:
- Robot footprint (default: 40cm x 24cm)
- Inflation radius (default: 0.20m)
- Planner and controller settings
- Costmap layer configuration

### RTAB-Map Parameters

Key SLAM parameters are set directly in `slam_nav_rtabmap.launch.xml`:
- `Reg/Strategy 1` — ICP-only registration (lidar mode)
- `Icp/VoxelSize 0.1` — ICP voxel size
- `RGBD/LinearUpdate 0.02` — New node every 2cm of movement
- `Grid/CellSize 0.05` — 5cm occupancy grid resolution

---

## Architecture

### C++ Nodes

Node -> Description
`odom_tf_bridge`: Publishes odom → base_link TF from Go2 odometry 
`joint_state_bridge`: Bridges Go2 joint states to /joint_states 
`cmdvel_to_sport_bridge`: Converts Nav2 cmd_vel to Unitree Sport API
`pointcloud_refiner`: Voxel filtering, ground removal, clustering
`frontier_explorer`: Frontier-based autonomous exploration

### Python Nodes

Node -> Description
`pointcloud_restamper.py`: Restamps lidar clouds with PC time
`laserscan_restamper.py`: Restamps laser scans
`image_restamper.py`: Restamps RGB images 
`camera_info_restamper.py`: Restamps camera info
`depth_sync_restamper.py`: Restamps depth + camera_info with identical timestamps
`inspection_node.py`: SAM 3 detection, 3D localization, change detection
`change_detector.py`: Post-hoc comparison of two inspection runs

### Post-Processing Scripts

Script -> Description
`watchdog_run.py`: Full lifecycle manager with auto-export
`map_export.py`: 2D annotated building floor plan generator
`ply_marker_injector.py`: Adds colored markers to 3D PLY files
`inspection_report.py`: PDF comparison report generator

### Topic Flow

![Low-level1 (1)](https://github.com/user-attachments/assets/9beeab37-a47d-476d-babd-11cc1e53d9dd)
![Low-level2](https://github.com/user-attachments/assets/187aab44-ca94-4752-a919-435c7dc5e4b8)

### Perception Pipeline
<img width="1700" height="2254" alt="Perception Pipeline" src="https://github.com/user-attachments/assets/6ca4e9f3-5492-4d52-8073-e91f278fdbb6" />

---

## Troubleshooting

### Robot gets stuck / "No valid trajectories"
- Check RViz costmap for phantom obstacles
- May be caused by RTAB-Map ICP failures in low-complexity environments
- Try increasing `inflation_radius` or checking the occupancy grid quality

### "Synchronized pairs: 0" on depth_to_pointcloud
- The depth image and camera_info must have identical timestamps
- `depth_sync_restamper.py` handles this — verify it's running

### rtabmap-export produces empty cloud
- Use `--scan` flag: `rtabmap-export --scan ~/.ros/rtabmap.db`
- Lidar-only mode stores scan data, not depth images

### ICP warnings "low complexity"
- Normal in featureless environments (long corridors, open rooms)
- `PointToPlaneMinComplexity 0.04` filters bad matches
- Robot relies on odometry when ICP fails

### SAM 3 connection refused
- Verify the SAM 3 server is running at the configured URL
- Test: `curl http://<sam3_url>/health`

### Camera topics not publishing
- Check RealSense connection: `ros2 topic list | grep camera`
- If running camera on the Go2, ensure `use_camera_restampers:=true`
- Depth sync requires both depth image and camera_info topics

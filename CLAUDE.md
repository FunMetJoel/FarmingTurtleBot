# FarmingTurtleBot — Claude Context

## What this project is
ROS2 Jazzy + TurtleBot3 burger (Gazebo simulation). The robot autonomously maps a field and collects humidity data across it.

## How to run
```bash
./start.local.sh   # builds and launches everything inside Docker
```
This runs `ros2 launch tb3_navigation_dt navigation_slam.launch.py`.

## Architecture — two-phase coverage mission

**Phase 1 — Frontier exploration** (`frontier_explorer.py`)
Robot maps the arena using SLAM Toolbox. `FrontierExplorer` (extends `BasicNavigator`) subscribes to `/map`, finds frontier clusters with BFS, and sends the nearest one as a Nav2 goal. Stops when no frontiers remain.

**Phase 2 — Zigzag sweep** (`zigzag_planner.py` + `coverage_orchestrator.py`)
Reads the finished OccupancyGrid, extracts free-space bounds with a wall margin, generates lawnmower waypoints, and drives through them via `followWaypoints`. The humidity pipeline runs passively the whole time.

**Entry point:** `coverage_orchestrator.py` — sequences both phases, launched as `coverageOrchestrator` node.

## Key packages
- `tb3_navigation_dt` — navigation + coverage logic + launch files
  - `launch/navigation_slam.launch.py` — main launch (Gazebo + SLAM + Nav2 + humidity + orchestrator)
  - `config/slam_params.yaml` — SLAM Toolbox config
  - `config/nav2_params.yaml` — kept for reference, NOT currently used
  - `tb3_navigation_dt/frontier_explorer.py`
  - `tb3_navigation_dt/zigzag_planner.py`
  - `tb3_navigation_dt/coverage_orchestrator.py`
- `tb3_field_dt` — humidity sensor pipeline (RandomHumiditySensor → HumiditySensorInterpreter → HumidityMapNode)
- `my_tb3_world` — Gazebo world definition

## Nav2 params
We use `turtlebot3_navigation2`'s `burger.yaml` (installed in Docker at `/opt/turtlebot3_ws`) because our custom `nav2_params.yaml` used the old Humble-style `planners` list format which fails to parse in Jazzy.

## Current status (2026-05-20)
The Nav2 params fix (switching to burger.yaml) was applied but **not yet confirmed working**. On next session, run `./start.local.sh` and check for:
```
[lifecycle_manager_navigation]: Managed nodes are active
[coverageOrchestrator]: Nav2 active - starting mission
[coverageOrchestrator]: Phase 1: Frontier Exploration
```
If Nav2 still fails, check for ERROR lines from `controller_server` or `planner_server`.

## Tunable constants
In `coverage_orchestrator.py`:
- `STRIP_WIDTH = 0.5` — metres between zigzag strips
- `MARGIN = 0.3` — metres to keep away from walls

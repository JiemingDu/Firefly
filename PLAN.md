# Autonomous Drone Navigation System — Phase 1 Plan
> Claude Code reference document. Read this before touching any file.

---

## Project Goal (Phase 1)

Build a fully autonomous drone navigation system that runs in ArduPilot SITL simulation.
By the end of Phase 1, a simulated drone must take off, build a 3D occupancy map from sensor
data, plan a collision-free path using A*, and navigate to a goal — all without human input.

This is the foundation every future hardware stage runs on. No hardware-specific code belongs
here. Everything is abstracted behind DroneInterface.

---

## Repository Layout

```
drone-nav/
├── PLAN.md                        ← this file
├── README.md
├── requirements.txt
├── .gitignore
│
├── drone/
│   ├── __init__.py
│   ├── interface.py               ← DroneInterface ABC
│   │
│   └── backends/
│       ├── __init__.py
│       └── sitl.py                ← SITLBackend (Phase 1 only)
│
├── navigation/
│   ├── __init__.py
│   ├── voxel_map.py               ← VoxelMap occupancy grid
│   ├── planner.py                 ← A* path planner (3D, 26-neighbour)
│   ├── fsm.py                     ← FlightFSM (FORWARD / TURNING / STOPPED)
│   └── navigator.py               ← AutonomousNavigator (top-level coordinator)
│
├── logging/
│   ├── __init__.py
│   └── data_logger.py             ← DataLogger (writes unified JSON-lines log)
│
├── tests/
│   ├── test_voxel_map.py
│   ├── test_planner.py
│   ├── test_fsm.py
│   └── test_navigator_sitl.py     ← integration test against live SITL
│
└── scripts/
    ├── run_sitl.sh                ← docker command to launch ArduPilot SITL
    └── fly_mission.py             ← entry point: connects, arms, runs navigator
```

---

## Module Contracts

### `drone/interface.py` — DroneInterface

Abstract base class. Every backend must implement all six methods exactly as typed.
No default implementations. Raises NotImplementedError if called directly.

```python
from abc import ABC, abstractmethod

class DroneInterface(ABC):

    @abstractmethod
    def get_position(self) -> tuple[float, float, float]:
        """Returns (x, y, z) in metres. Origin = takeoff point."""

    @abstractmethod
    def get_ranges(self) -> dict[str, float]:
        """
        Returns obstacle distances in metres.
        Keys: 'front', 'back', 'left', 'right', 'up'
        999.0 = no obstacle in range.
        """

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        """Body-frame velocity. Units: m/s linear, rad/s yaw."""

    @abstractmethod
    def takeoff(self, height: float = 1.5) -> None: ...

    @abstractmethod
    def land(self) -> None: ...

    @abstractmethod
    def emergency_stop(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...
```

**Rules:**
- Never add hardware logic here
- Never import cflib, dronekit, pymavlink, or serial here
- This file must be importable with zero external dependencies

---

### `drone/backends/sitl.py` — SITLBackend

Connects to ArduPilot SITL at `tcp:127.0.0.1:5760` using DroneKit.

```python
from dronekit import connect, VehicleMode
from drone.interface import DroneInterface

class SITLBackend(DroneInterface):
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        self.vehicle = connect(connection_string, wait_ready=True)
    ...
```

**Position:** read from `vehicle.location.local_frame` (north/east/down).
Convert: x = north, y = east, z = -down (so z is positive upward).

**Ranges:** SITL has no real sensors. Simulate range data by raycasting against
a hardcoded obstacle list, OR read from MAVLink DISTANCE_SENSOR messages if
the SITL environment has obstacle plugins enabled. For Phase 1, a simple
synthetic range function is acceptable — document clearly that this is simulated.

**Velocity:** use `vehicle.send_mavlink()` with a SET_POSITION_TARGET_LOCAL_NED
message in MAV_FRAME_BODY_NED with type mask set to velocity-only.

**takeoff:** set mode GUIDED, arm, call `vehicle.simple_takeoff(height)`,
block until `vehicle.location.global_relative_frame.alt >= height * 0.95`.

**land:** set mode LAND, block until vehicle is disarmed.

**emergency_stop:** send zero velocity, set mode BRAKE or LOITER.

---

### `navigation/voxel_map.py` — VoxelMap

3D occupancy grid. Each cell: 0 = FREE, 1 = OCCUPIED, 2 = UNKNOWN.

```python
import numpy as np

class VoxelMap:
    FREE     = 0
    OCCUPIED = 1
    UNKNOWN  = 2

    def __init__(self, size_x: int, size_y: int, size_z: int,
                 resolution: float = 0.5, origin: tuple = (0.0, 0.0, 0.0)):
        self.res    = resolution
        self.origin = origin
        self.grid   = np.full((size_x, size_y, size_z), self.UNKNOWN, dtype=np.uint8)
```

**Methods to implement:**
- `world_to_grid(x, y, z) -> tuple[int,int,int]` — clamp to grid bounds
- `grid_to_world(gx, gy, gz) -> tuple[float,float,float]` — voxel centre
- `mark_obstacle(x, y, z)` — set OCCUPIED
- `mark_free(x, y, z)` — set FREE
- `is_free(x, y, z) -> bool` — returns True only for FREE (not UNKNOWN)
- `update_from_ranges(position, ranges)` — given drone pos + 5-direction range dict,
  raycast along each direction and mark free cells along the ray, mark the endpoint OCCUPIED
  if range < sensor max (4.0m). Use Bresenham 3D line for the raycast.
- `inflate_obstacles(radius_m)` — for every OCCUPIED cell, mark all cells within
  radius_m as OCCUPIED. Run once after map is built or periodically.
- `save(path)` / `load(path)` — numpy save/load for persistence

**Bounds:** 20 × 20 × 10 grid at 0.5m resolution = 10m × 10m × 5m volume.
Enough for Phase 1 indoor SITL testing.

---

### `navigation/planner.py` — AStarPlanner

3D A* over the VoxelMap grid.

```python
import heapq, math

def astar_3d(voxel_map: VoxelMap,
             start_world: tuple,
             goal_world: tuple) -> list[tuple] | None:
    """
    Returns list of world-coordinate waypoints (x, y, z) from start to goal,
    or None if no path exists.
    """
```

**Implementation rules:**
- Convert start/goal to grid coordinates before search
- 26-neighbour connectivity (all adjacent voxels including face, edge, corner)
- Move cost: 1.0 for face neighbours, √2 for edge, √3 for corner
- Heuristic: Euclidean distance to goal in grid units (admissible)
- Skip any neighbour cell that is OCCUPIED or out of bounds
- UNKNOWN cells are treated as FREE (optimistic assumption — explore them)
- Reconstruct path, convert back to world coordinates (use voxel centre)
- Path smoothing (optional Phase 1 bonus): remove collinear waypoints

**Return format:** list of (x, y, z) tuples in world metres, including start and goal.

---

### `navigation/fsm.py` — FlightFSM

Reactive safety layer. Operates on top of the A* plan.

```python
from enum import Enum, auto

class FSMState(Enum):
    FORWARD  = auto()
    TURNING  = auto()
    STOPPED  = auto()
```

**State logic:**

| State | Condition to stay | Transition |
|---|---|---|
| FORWARD | front range > CLEAR_THRESHOLD (1.5m) | → TURNING if front < threshold |
| TURNING | front range < CLEAR_THRESHOLD | → FORWARD when clear |
| Any | emergency flag set | → STOPPED |

**compute(path, position, ranges) -> tuple[float,float,float,float]:**
- In FORWARD: steer toward next waypoint on the A* path
  - Compute bearing to next waypoint (atan2), output vx/vy toward it
  - Pop waypoint from path when within WAYPOINT_RADIUS (0.3m)
- In TURNING: yaw in place until front is clear, vx=vy=0
- In STOPPED: all zeros

**Constants (tune during testing):**
```python
CLEAR_THRESHOLD  = 1.5   # metres — obstacle triggers TURNING
WAYPOINT_RADIUS  = 0.3   # metres — waypoint considered reached
CRUISE_SPEED     = 0.5   # m/s forward
YAW_RATE         = 0.3   # rad/s turning
```

---

### `navigation/navigator.py` — AutonomousNavigator

Top-level coordinator. Owns all components. This is the only file that
calls the FSM, planner, map, logger, and drone together.

```python
class AutonomousNavigator:
    LOOP_HZ      = 10    # control loop frequency
    REPLAN_DIST  = 0.5   # replan if obstacle appears within this range

    def __init__(self, drone: DroneInterface):
        self.drone   = drone
        self.map     = VoxelMap(20, 20, 10, resolution=0.5)
        self.planner = AStarPlanner(self.map)
        self.fsm     = FlightFSM()
        self.logger  = DataLogger()

    def run(self, goal: tuple[float, float, float]) -> None:
        self.drone.takeoff(height=1.5)
        path = None

        while not self._reached(goal):
            pos    = self.drone.get_position()
            ranges = self.drone.get_ranges()

            self.map.update_from_ranges(pos, ranges)

            if path is None or self._needs_replan(ranges):
                path = self.planner.plan(pos, goal)
                if path is None:
                    self.drone.emergency_stop()
                    return

            cmd = self.fsm.compute(path, pos, ranges)
            self.drone.set_velocity(*cmd)
            self.logger.record(pos, ranges, cmd, self.fsm.state)

            time.sleep(1 / self.LOOP_HZ)

        self.drone.land()

    def _reached(self, goal, threshold=0.4) -> bool:
        pos = self.drone.get_position()
        return math.dist(pos, goal) < threshold

    def _needs_replan(self, ranges) -> bool:
        return ranges.get('front', 999) < self.REPLAN_DIST
```

---

### `logging/data_logger.py` — DataLogger

Writes one JSON object per line to a `.jsonl` file.

```python
# Each record:
{
    "timestamp":    float,          # seconds since logger init
    "position":     [x, y, z],
    "ranges":       {"front":…, "back":…, "left":…, "right":…, "up":…},
    "velocity_cmd": [vx, vy, vz, yaw],
    "fsm_state":    "FORWARD" | "TURNING" | "STOPPED"
}
```

Default output path: `logs/flight_<ISO_TIMESTAMP>.jsonl`
Create `logs/` dir if it doesn't exist.

---

### `scripts/fly_mission.py` — Entry Point

```python
from drone.backends.sitl import SITLBackend
from navigation.navigator import AutonomousNavigator

drone = SITLBackend()
nav   = AutonomousNavigator(drone)
nav.run(goal=(5.0, 0.0, 1.5))   # fly 5m forward at 1.5m altitude
```

---

### `scripts/run_sitl.sh` — SITL Launcher

```bash
#!/bin/bash
docker run -it --rm \
  --network host \
  ardupilot/ardupilot-dev \
  sim_vehicle.py -v ArduCopter --console --map \
  --out=tcp:0.0.0.0:5760
```

---

## Build Order

Build in this exact order. Each step must pass its tests before moving to the next.

```
1. drone/interface.py              (no deps — write and verify importable)
2. drone/backends/sitl.py          (depends on interface + dronekit)
3. navigation/voxel_map.py         (depends on numpy only)
4. navigation/planner.py           (depends on voxel_map)
5. navigation/fsm.py               (depends on nothing external)
6. logging/data_logger.py          (depends on nothing external)
7. navigation/navigator.py         (depends on all of the above)
8. scripts/run_sitl.sh             (shell script — no Python deps)
9. scripts/fly_mission.py          (integration — requires SITL running)
10. tests/                         (write alongside each module)
```

---

## Testing Strategy

### Unit tests (no SITL required)

| Test file | What it covers |
|---|---|
| `test_voxel_map.py` | world↔grid conversion, mark/query, raycast, inflation |
| `test_planner.py` | finds path in empty map, finds path around wall, returns None when blocked |
| `test_fsm.py` | state transitions on mocked range values, velocity output direction |

### Integration test (SITL must be running)

`test_navigator_sitl.py` — connect SITLBackend, call `nav.run(goal=(5,0,1.5))`,
assert drone reaches goal within 60 seconds and log file is non-empty.

### Manual validation

Watch the ArduPilot SITL map window. The drone's path should visibly avoid any
obstacles placed in the SITL environment and reach the goal position.

---

## Environment Setup

```bash
# 1. Clone repo and set up venv
python3.11 -m venv .venv
source .venv/bin/activate
pip install dronekit pymavlink numpy

# 2. Start SITL
chmod +x scripts/run_sitl.sh
./scripts/run_sitl.sh
# Wait for "APM: EKF2 IMU0 is using GPS" before connecting

# 3. Run mission
python scripts/fly_mission.py
```

---

## Definition of Done — Phase 1

- [ ] All unit tests pass with `pytest tests/`
- [ ] Simulated drone takes off, navigates to goal (5, 0, 1.5), lands
- [ ] At least one obstacle in the SITL environment is avoided (verify in map window)
- [ ] Log file produced with correct schema for every flight
- [ ] Swapping `SITLBackend` for a mock backend requires zero changes outside `fly_mission.py`
- [ ] No hardware-specific imports anywhere except `drone/backends/`

---

## What Does NOT Belong in Phase 1

- CrazyflieBackend (Phase 2)
- MAVLinkBackend (Phase 3)
- RealSense point cloud ingestion (Phase 3)
- Web dashboard (Phase 4)
- Thermal processing (Phase 4)
- Any ML / PyTorch code (Phase 5)
- Anything that requires physical hardware

If you find yourself importing cflib, open3d, or torch, stop — you're in the wrong phase.

---

## Notes for Claude Code

- Always run `source .venv/bin/activate` before any Python command
- SITL must be running before any integration test or `fly_mission.py`
- If DroneKit connect times out, check that Docker container is up with `docker ps`
- All paths in code are relative to the repo root
- Log files go in `logs/` (gitignored)
- Never commit `.venv/`, `logs/`, or `__pycache__/`

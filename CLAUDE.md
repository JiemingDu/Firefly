# CLAUDE.md — Autonomous Drone Navigation System

## What This Is

A hardware-agnostic autonomous drone navigation system built in Python 3.11.
Long-term goal: modular firefighting scout drone for disaster relief.
Current phase: **Phase 1 — SITL simulation only. No physical hardware.**

Read `PLAN.md` for full architecture, module contracts, and build order.

---

## Current Phase: Phase 1

Only these components exist or should be built right now:

- `drone/interface.py` — DroneInterface ABC
- `drone/backends/sitl.py` — SITLBackend (DroneKit → ArduPilot SITL)
- `navigation/voxel_map.py` — 3D occupancy grid
- `navigation/planner.py` — A* path planner (3D, 26-neighbour)
- `navigation/fsm.py` — FlightFSM (FORWARD / TURNING / STOPPED)
- `navigation/navigator.py` — AutonomousNavigator (top-level coordinator)
- `logging/data_logger.py` — DataLogger (.jsonl output)
- `scripts/run_sitl.sh` — Docker SITL launcher
- `scripts/fly_mission.py` — mission entry point
- `tests/` — unit + integration tests

---

## Hard Rules

**1. No hardware logic outside `drone/backends/`.**
`navigation/`, `logging/`, and `scripts/fly_mission.py` must never import
`cflib`, `dronekit`, `pymavlink`, `serial`, or any hardware library directly.

**2. Every backend implements DroneInterface exactly.**
All 7 methods. No extra public methods that the navigator depends on.
The navigator only calls methods defined on DroneInterface.

**3. Swapping backends is one line.**
`AutonomousNavigator(drone=SITLBackend())` → `AutonomousNavigator(drone=CrazyflieBackend())`
If anything else needs to change to swap hardware, the abstraction is broken.

**4. Phase 1 scope is frozen.**
Do not add, suggest, or scaffold anything from Phase 2+:
- No CrazyflieBackend, MAVLinkBackend, CustomPCBBackend
- No RealSense / point cloud code
- No web dashboard, Flask, FastAPI, WebSockets
- No FLIR thermal processing
- No PyTorch, imitation learning, or ML of any kind

If a feature isn't in the Phase 1 module list above, don't build it.

---

## Dev Environment

```bash
# Activate venv (always do this first)
source .venv/bin/activate

# Install dependencies
pip install dronekit pymavlink numpy

# Start SITL (Docker must be running)
./scripts/run_sitl.sh
# Wait for: "APM: EKF2 IMU0 is using GPS" before connecting

# Run mission
python scripts/fly_mission.py

# Run tests
pytest tests/
```

macOS Apple Silicon. Python 3.11. Editor: Neovim.

---

## Architecture in One Glance

```
fly_mission.py
    └── AutonomousNavigator(drone=SITLBackend())
            ├── SITLBackend          ← only file that touches DroneKit
            ├── VoxelMap             ← numpy occupancy grid
            ├── AStarPlanner         ← 3D A*, 26-neighbour
            ├── FlightFSM            ← reactive safety layer
            └── DataLogger           ← .jsonl flight logs
```

---

## Key Contracts (quick ref)

**DroneInterface methods:**
`get_position() -> (x,y,z)` · `get_ranges() -> dict` · `set_velocity(vx,vy,vz,yaw)`
`takeoff(height)` · `land()` · `emergency_stop()` · `is_connected() -> bool`

**get_ranges() keys:** `'front'`, `'back'`, `'left'`, `'right'`, `'up'` — value in metres, `999.0` = no obstacle

**VoxelMap cell values:** `0` = FREE · `1` = OCCUPIED · `2` = UNKNOWN

**FSM states:** `FORWARD` → `TURNING` → `STOPPED`

**Log record fields:** `timestamp`, `position`, `ranges`, `velocity_cmd`, `fsm_state`

---

## Definition of Done — Phase 1

- [ ] `pytest tests/` passes with no errors
- [ ] Drone takes off, navigates to goal `(5.0, 0.0, 1.5)`, lands
- [ ] At least one obstacle is avoided (visible in SITL map window)
- [ ] `.jsonl` log file produced with correct schema
- [ ] No hardware imports outside `drone/backends/`

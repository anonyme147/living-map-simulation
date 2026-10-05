# Living Map Simulation

An emergency-robotics simulation with a Writer robot, an Outside Network Area
(ONA) relay, a Command Post dashboard, and Executor mission dispatch.

## Project Description

The Living Map preserves emergency observations when communication links or
robots fail. A Writer robot detects hazards and victims, creates compact
beacons with position, confidence, timestamp, TTL, and CRC integrity data, and
passes them through the ONA. The Command Post displays confirmed observations
and prepares missions for the most compatible Executor unit.

## Architecture

```text
Writer Robot
    -> beacon observation
Outside Network Area (ONA)
    -> RECEIVE, TRANSLATE, CARRY, BRIEF
Command Post
    -> live map, mission preparation, operator authorization
Outside Network Area (ONA)
    -> controlled mission relay
Executor Robot
    -> target navigation and mission-completed beacon
```

The Command Post never communicates directly with either robot. The ONA is the
only communication boundary. Satellite transport is used when available; RF
mesh routing provides continuity during supported link-failure scenarios.

## Requirements

- Python 3.10 or later
- `pip`
- A modern web browser for the dashboard
- Optional: Webots R2025a or compatible for the separate robot visualization

## Install

```bash
git clone https://github.com/anonyme147/living-map-simulation.git
cd living-map-simulation
python -m venv .venv
```

Activate the environment:

```powershell
# Windows PowerShell
.\.venv\Scripts\Activate.ps1
```

```bash
# Linux or macOS
source .venv/bin/activate
```

Install the dashboard and simulation dependencies:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Run the Demo Menu

```bash
python simulation/demo_menu.py
```

Open `http://127.0.0.1:5050` and select a scenario. The menu starts each
scenario separately and provides its Command Post dashboard address.

## Run the Main Simulation Directly

```bash
python simulation/run_demo.py
```

Open `http://127.0.0.1:5000` while the simulation is running.

## Included Scenarios

- Live Mission Simulation
- Live Mission Simulation + Webotsim
- Satellite Failure to RF Mesh Failover
- RF Mesh Dual-Node Failure
- Zombie Link Heartbeat Loss
- Satellite Loss Without Mesh
- Zombie Link Without Detection
- Signal Integrity Showcase

## Optional Webots Companion

The Webots companion is independent of the Command Post simulation. Install
Webots separately, then open the world file under
`Pioneer_LiDAR_v16_4_FIXED/Pioneer_LiDAR_v16_4_NavigationMaintained/webots/worlds/`.

This repository intentionally excludes downloaded machine-learning models,
model weights, vendor clones, camera recordings, logs, and generated output.
They are not required for the core dashboard simulations. The optional Webots
vision features require their corresponding local model files to be installed
in the ignored `models/` or `weights/` locations configured by the controller.

## Troubleshooting

- If the dashboard port is already in use, close the previous simulation or
  select **Reset and launch** from the menu.
- If PowerShell blocks environment activation, run:

  ```powershell
  Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
  ```

- Run commands from the repository root so Python can resolve the project
  packages.

<p align="center">
  <img src="images/HYDRA_UMC_BANNER.svg" alt="HYDRA-UMC-PATH-PLANNER-3D banner" width="100%">
</p>

# 🗺️ HYDRA-UMC-PATH-PLANNER-3D

<p align="center">🇺🇸 <b>English</b> | <a href="README_spa.md">🇪🇸 Español</a> | <a href="README_fra.md">🇫🇷 Français</a> | <a href="README_ita.md">🇮🇹 Italiano</a> | <a href="README_deu.md">🇩🇪 Deutsch</a> | <a href="README_zho.md">🇨🇳 简体中文</a> | <a href="README_jpn.md">🇯🇵 日本語</a></p>

### 📐 Multi-Robot 3D Path Optimizer & Collision Avoidance Engine

<p align="left">
  <img src="https://img.shields.io/badge/Licencia-GPL%203.0-blue.svg" alt="GPL 3.0">
  <img src="https://img.shields.io/badge/Algorithm-RRT*%20%2F%20Potential%20Fields-orange.svg" alt="Algorithms">
  <img src="https://img.shields.io/badge/Engine-C++20%20%2F%20Rust-blue.svg" alt="Engine">
</p>

---

## 1. 🛠️ TECHNICAL OVERVIEW

**HYDRA-UMC-PATH-PLANNER-3D** is the centralized navigation intelligence for the robot swarm. It calculates collision-free trajectories for multiple arms sharing the same workspace, optimizing for speed, energy efficiency, and smooth motion.

It integrates real-time occupancy data from the Vision Nodes and kinematic constraints from the Digital Twin to ensure that planned paths are physically feasible and safe.

### Key Features:
* 📐 **Swarm Path Optimization:** Synchronous planning for up to 32+ robots simultaneously.
* 🛡️ **Dynamic Collision Avoidance:** Real-time re-planning when new obstacles are detected.
* ⚡ **Performance Optimized:** Highly parallelized C++/Rust implementation for sub-50ms path generation.
* 🔄 **G-Code & URDF Native:** Directly parses industrial motion commands and robot models.
* ⏱️ **Real Time Limit & Trajectory Validation (v0):** `PlannerConfig.max_duration_ms` bounds search time by the wall clock, independent of `max_iterations`. A new `validate` subcommand re-checks an already-computed path (cached, replayed, or hand-edited) against the current obstacles/workspace before it is trusted for real execution.

---

## 2. 🔄 PLANNING WORKFLOW

```mermaid
flowchart TB
    GOAL["Swarm Goal / Mission"] --> PLAN["3D PATH-PLANNER"]
    PLAN --> COLL["Collision Check (Octree)"]
    VIS["Vision Safety Zones"] --> COLL
    TWIN["Robot Constraints (URDF)"] --> COLL
    COLL -- Clean --> OPT["Trajectory Optimizer (S-Curve)"]
    OPT --> SYNC["SWARM-SYNC Dispatch"]
```

---

## 3. 🧱 ARCHITECTURE & DESIGN DECISIONS

* **Why 3D collision-free planning is its own service.** Path search over a live 3D scene (every robot, tool, and obstacle in the cell) is CPU-heavy and latency-sensitive in a different way than job scheduling itself - isolating it means a slow planning query never blocks HYDRA-UMC-JOB-DISPATCHER from assigning other work.
* **Why it validates against HYDRA-UMC-TWIN before real execution.** A planned route gets checked in the digital twin's own physics simulation first - catching a geometrically-valid-but-physically-unreachable path (torque limits, singularities) before it's ever sent to a real arm.
* **Why the search is a plain RRT today, not RRT\* or a multi-robot coordinator.** `src/rrt.rs` implements a real, tested Rapidly-exploring Random Tree search - a single agent, a static obstacle set, a genuinely collision-free result (every returned path is verified obstacle-clear in tests, not just plausible-looking). RRT\* (an optimality-improving rewiring pass) and true multi-robot synchronized planning (this README's "up to 32+ robots simultaneously") are real, scoped-out future work - see `mejoras_futuras.txt` for why proving one agent's search correct came first, rather than building all three at once and debugging them together.
* **Why obstacles are spheres, not an octree of arbitrary meshes.** A sphere is the simplest 3D collision primitive that is still real and geometrically correct - not a bounding-box stand-in for something more detailed. An octree/BVH only starts to matter once a scene has enough colliders that brute-force checking is the bottleneck; today's swarm cell (a handful of arms and static safety zones) does not.
* **Why this is a CLI over a JSON scenario file today, not a network service.** Choosing HTTP vs. the ecosystem's shared gRPC contract (`hydra.common.v1`, see `HYDRA-UMC-ORCHESTRATOR/proto/`) is a real protocol decision that deserves its own pass once HYDRA-UMC-JOB-DISPATCHER is actually ready to call this service - see `mejoras_futuras.txt`. The CLI is still genuinely usable today (`run.bat scenarios/example.json`), it just isn't wired into the network yet.
* **How this fits the rest of the ecosystem.** A sibling service under HYDRA-UMC-ORCHESTRATOR - plans the routes HYDRA-UMC-JOB-DISPATCHER's assigned jobs actually follow, cross-checked against HYDRA-UMC-TWIN before anything moves for real.
* **Why `max_duration_ms` is a wall-clock check, not just a lower `max_iterations`.** Iteration count alone cannot bound real time: a scene with more/denser obstacles makes each iteration's collision checks proportionally slower, so the same iteration budget can take wildly different real time on a pathological scene versus an open one. A planner call sitting in a real-time control loop needs an actual time budget, not a proxy for one.
* **Why `validate` is a new subcommand instead of changing what `plan()` returns.** `rrt::plan()` already only ever returns paths it built collision-free by construction - there's no gap to close there. `validate_path()`/the `validate` subcommand exists for paths that did NOT come from a fresh `plan()` call in this process: a cached/replayed path, one relayed from another process, a hand-edited scenario - where the obstacle set may have changed since the path was computed. Same pattern used across the ecosystem: a new, safety-gated entry point added alongside an unchanged low-level primitive.

---

## 📂 DIRECTORY STRUCTURE

```text
HYDRA-UMC-PATH-PLANNER-3D/
├── src/
│   ├── main.rs       # CLI entry point: loads a scenario, plans, prints JSON
│   ├── geometry.rs   # Vec3 - the minimal 3D vector math the rest needs
│   ├── obstacle.rs   # Sphere obstacles + segment/point collision checks
│   ├── rng.rs        # Deterministic dependency-free PRNG (xorshift64*)
│   ├── rrt.rs        # The real planner: RRT search, Workspace, PlannerConfig
│   └── validate.rs   # Real safety re-check of an already-computed path
├── scenarios/        # Example JSON scenarios (see BUILD & RUN below)
├── build/            # Compiled binaries (build.sh/build.bat output)
├── Cargo.toml        # Rust package manifest (name, version, deps)
├── bump_version.py   # Odometer-style version bump, run by build.sh/.bat
├── build.sh/.bat     # Bumps version, then `cargo build --release`
├── run.sh/.bat       # Runs the compiled binary
└── README.md
```

Pruned from the original template: `hardware/`, `firmware/`, `os/`, `docs/`,
`images/` and `scripts/` — this is a pure software service (Rust binary)
with no dedicated hardware or firmware of its own, no operating system
image to maintain, and no documentation/media/utility-script content
substantial enough yet to warrant their own folders.

---

## 🔧 BUILD & RUN GUIDE

A real collision-free path search, not just a skeleton that compiles: it
plans a route through a JSON scenario file and prints the result.

```bash
# Windows
build.bat
run.bat scenarios/example.json

# Linux / macOS
./build.sh
./run.sh scenarios/example.json
```

`build.sh`/`build.bat` bump the version in `Cargo.toml` (ecosystem-wide
odometer rule, see `bump_version.py`) and then run `cargo build --release`.
`run.sh`/`run.bat` execute the resulting binary directly, forwarding any
arguments (the scenario path) to it.

A scenario is a JSON file with `start`, `goal`, `obstacles` (a list of
`{center, radius}` spheres), a `workspace` (`min`/`max` bounds), and an
optional `seed` (the search is fully deterministic per seed) and `config`
(`max_iterations`, `step_size`, `goal_bias`, `goal_threshold`,
`robot_radius`, `max_duration_ms` - all optional, sane defaults otherwise;
`max_duration_ms` bounds search time by the wall clock, independent of
`max_iterations`). The result is printed as JSON:
`{"status": "ok", "path": [...]}` or
`{"status": "error", "reason": "..."}` (`start_inside_obstacle`,
`goal_outside_workspace`, `no_path_found`, `time_limit_exceeded`, etc. -
see `src/rrt.rs`'s `PlanError` for the full, honest list, including the
case where no path exists at all, not just one the search failed to find
in time).

A second real subcommand re-checks an already-computed path (a bare JSON
array of `{x, y, z}` waypoints) for safety against a scenario's current
obstacles/workspace, without running a new search:

```bash
./run.sh validate scenarios/example.json path.json
# {"status": "safe"}
# or: {"status": "unsafe", "issues": [{"SegmentIntersectsObstacle": {"from_index": 0, "to_index": 1}}]}
```

```bash
cargo test   # geometry + obstacle collision math, the PRNG, the RRT
             # planner (including its real wall-clock time limit), and
             # validate.rs's own safety re-check - 30 tests total
```

---

## 🚀 ROADMAP
* **Phase 1:** Deterministic swarm synchronization over TSN and sub-ms jitter reduction.
* **Phase 2:** 3D Path planning with dynamic obstacle avoidance in multi-robot cells.
* **Phase 3:** Multi-robot job dispatching optimization using real-time resource availability.
* **Phase 4:** Support for non-holonomic mobile base path planning (JuanenBOT) and heterogeneous fleet integration.

---

## 🔗 Related Projects

This project is part of a larger robotics ecosystem by the same author (JuanenRac / Electro Hobby 3D), spanning firmware, control software, AI nodes, and fleet tooling. Worth knowing about, since a request might actually be about one of these rather than this repository.

### Family

**Parent:** **[HYDRA-UMC-ORCHESTRATOR](https://github.com/JuanenRac/HYDRA-UMC-ORCHESTRATOR)** — the integration parent this planner serves.

**Siblings:**
- **[HYDRA-UMC-SWARM-SYNC](https://github.com/JuanenRac/HYDRA-UMC-SWARM-SYNC)** — sibling orchestration service, same parent.
- **[HYDRA-UMC-JOB-DISPATCHER](https://github.com/JuanenRac/HYDRA-UMC-JOB-DISPATCHER)** — sibling orchestration service, same parent.
- **[HYDRA-UMC-NODE-HEALING](https://github.com/JuanenRac/HYDRA-UMC-NODE-HEALING)** — sibling orchestration service, same parent.

### Directly Related (outside the family)

- **[HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)** — validates planned routes in the digital twin before executing them.

### Rest of the Ecosystem

**HYDRA-UMC platform** — the multi-robot micro-factory cell
- **[HYDRA-UMC](https://github.com/JuanenRac/HYDRA-UMC)** — the CM5 + STM32H745 motherboard orchestrating up to 8 robot arms.
- **[HYDRA-UMC-SERVER](https://github.com/JuanenRac/HYDRA-UMC-SERVER)** — the Express/WebSocket backend every control client talks to.
- **[HYDRA-UMC-STUDIO](https://github.com/JuanenRac/HYDRA-UMC-STUDIO)** — web-based control dashboard, multi-robot 3D visualization.
- **[HYDRA-UMC-ANDROID-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-ANDROID-CONTROL)** — Android control app over Wi-Fi/Bluetooth.
- **[HYDRA-UMC-IOS-CONTROL](https://github.com/JuanenRac/HYDRA-UMC-IOS-CONTROL)** — iOS/iPadOS control app built in Flutter.
- **[HYDRA-UMC-SUITE](https://github.com/JuanenRac/HYDRA-UMC-SUITE)** — desktop swarm command center (Python/PySide6).
- **[HYDRA-UMC-EDITOR-URDF](https://github.com/JuanenRac/HYDRA-UMC-EDITOR-URDF)** — desktop URDF model editor for the robot catalog.
- **[HYDRA-UMC-DSI](https://github.com/JuanenRac/HYDRA-UMC-DSI)** — native touch UI for the onboard DSI touchscreen.

**URTC platform** — the tool head controller every HYDRA-UMC robot arm carries
- **[URTC](https://github.com/JuanenRac/URTC)** — CAN bus tool head controller, 25 tool profiles.
- **[URTC-FLASHER](https://github.com/JuanenRac/URTC-FLASHER)** — desktop CAN-OTA + SWD/JTAG flashing tool.
- **[URTC-TESTER](https://github.com/JuanenRac/URTC-TESTER)** — desktop live CAN-bus diagnostic tool.
- **[URTC-WEB-STUDIO](https://github.com/JuanenRac/URTC-WEB-STUDIO)** — browser-based alternative via Web Serial API.

**🎥 Vision AI Node (Hailo-8)**
- [HYDRA-UMC-VISION-NODE](https://github.com/JuanenRac/HYDRA-UMC-VISION-NODE)
- [HYDRA-UMC-VISION-STREAMER](https://github.com/JuanenRac/HYDRA-UMC-VISION-STREAMER)
- [HYDRA-UMC-DETECTION-HEF](https://github.com/JuanenRac/HYDRA-UMC-DETECTION-HEF)
- [HYDRA-UMC-SAFETY-ZONES](https://github.com/JuanenRac/HYDRA-UMC-SAFETY-ZONES)
- [HYDRA-UMC-VISUAL-SERVOING-API](https://github.com/JuanenRac/HYDRA-UMC-VISUAL-SERVOING-API)

**🧠 Cognitive AI Node (Hailo-10)**
- [HYDRA-UMC-COGNITIVE-NODE](https://github.com/JuanenRac/HYDRA-UMC-COGNITIVE-NODE)
- [HYDRA-UMC-VLA-ENGINE](https://github.com/JuanenRac/HYDRA-UMC-VLA-ENGINE)
- [HYDRA-UMC-VOICE-UI](https://github.com/JuanenRac/HYDRA-UMC-VOICE-UI)
- [HYDRA-UMC-SEMANTIC-PLANNER](https://github.com/JuanenRac/HYDRA-UMC-SEMANTIC-PLANNER)
- [HYDRA-UMC-DOCS-QA](https://github.com/JuanenRac/HYDRA-UMC-DOCS-QA)

**🎮 Digital Twin & Simulation**
- [HYDRA-UMC-TWIN](https://github.com/JuanenRac/HYDRA-UMC-TWIN)
- [HYDRA-UMC-PHYSICS-REPLICA](https://github.com/JuanenRac/HYDRA-UMC-PHYSICS-REPLICA)
- [HYDRA-UMC-HIL-BRIDGE](https://github.com/JuanenRac/HYDRA-UMC-HIL-BRIDGE)
- [HYDRA-UMC-SYNTHETIC-DATA-GEN](https://github.com/JuanenRac/HYDRA-UMC-SYNTHETIC-DATA-GEN)

**📊 Data & Analytics**
- [HYDRA-UMC-DATALAKE](https://github.com/JuanenRac/HYDRA-UMC-DATALAKE)
- [HYDRA-UMC-TELEMETRY-COLLECTOR](https://github.com/JuanenRac/HYDRA-UMC-TELEMETRY-COLLECTOR)
- [HYDRA-UMC-ANOMALY-DETECTOR](https://github.com/JuanenRac/HYDRA-UMC-ANOMALY-DETECTOR)
- [HYDRA-UMC-PRODUCTION-REPORTS](https://github.com/JuanenRac/HYDRA-UMC-PRODUCTION-REPORTS)

**🏭 Industrial Gateway**
- [HYDRA-UMC-GATEWAY-INDUSTRIAL](https://github.com/JuanenRac/HYDRA-UMC-GATEWAY-INDUSTRIAL)
- [HYDRA-UMC-OPCUA-SERVER](https://github.com/JuanenRac/HYDRA-UMC-OPCUA-SERVER)
- [HYDRA-UMC-MQTT-BROKER](https://github.com/JuanenRac/HYDRA-UMC-MQTT-BROKER)
- [HYDRA-UMC-MTCONNECT-ADAPTER](https://github.com/JuanenRac/HYDRA-UMC-MTCONNECT-ADAPTER)

**🛠️ Complementary Tools**
- [URTC-SMART-RACK](https://github.com/JuanenRac/URTC-SMART-RACK)
- [URTC-VISION-TOOL](https://github.com/JuanenRac/URTC-VISION-TOOL)
- [HYDRA-UMC-WATCH](https://github.com/JuanenRac/HYDRA-UMC-WATCH)
- [HYDRA-UMC-TOOL-CLI](https://github.com/JuanenRac/HYDRA-UMC-TOOL-CLI)
- [HYDRA-UMC-DASHBOARD-AI](https://github.com/JuanenRac/HYDRA-UMC-DASHBOARD-AI)


## 👤 AUTHOR
**JuanenRac** (Electro Hobby 3D)
📧 electrohobby3d@gmail.com
📺 [youtube.com/@electrohobby3d](https://youtube.com/@electrohobby3d)

## 📜 LICENSE
GPL-3.0 - See LICENSE for details.

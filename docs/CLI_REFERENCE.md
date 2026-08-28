# HYDRA-UMC-PATH-PLANNER-3D — CLI Reference

`hydra-umc-path-planner-3d` is a single Rust binary (`src/main.rs`) driven by
a JSON scenario file — a real single-agent RRT collision-free path search
(`src/rrt.rs`), not yet a network service. Every example below was captured
from a real, built release binary run against the repo's own real
`scenarios/example.json` fixture (and small fixtures constructed the same
way) — the output shown is real, not illustrative.

## Usage

```
$ ./run.sh <scenario.json>
$ ./run.sh validate <scenario.json> <path.json>
```

`run.sh` execs the built binary (`build/hydra-umc-path-planner-3d` if
present, else `target/release/hydra-umc-path-planner-3d`) and forwards all
arguments unchanged. The examples below invoke the release binary directly,
which is equivalent.

Every invocation — including bare, no-argument invocation — first prints
identity/version and role, then the command's own output:

```
$ hydra-umc-path-planner-3d
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
Usage: hydra-umc-path-planner-3d <scenario.json>
       hydra-umc-path-planner-3d validate <scenario.json> <path.json>
See scenarios/example.json for the expected format.
```

Bare invocation exits `0` — printing identity and usage is treated as a
valid no-argument invocation, not a failure.

## Commands

### `<scenario.json>`

Runs the real RRT planner against a scenario (`start`, `goal`, `obstacles`,
`workspace`, optional `config`/`seed`) and prints the result as JSON:
`{"status":"ok","path":[...]}` on success, or
`{"status":"error","reason":"..."}` if the planner can't produce a path.

The repo's own `scenarios/example.json` fixture — start/goal on opposite
sides of a workspace, with three spherical obstacles in the way:

```json
{
  "start": { "x": -5.0, "y": 0.0, "z": 0.0 },
  "goal": { "x": 5.0, "y": 0.0, "z": 0.0 },
  "obstacles": [
    { "center": { "x": 0.0, "y": 0.0, "z": 0.0 }, "radius": 1.5 },
    { "center": { "x": 0.0, "y": 2.0, "z": 0.0 }, "radius": 1.5 },
    { "center": { "x": 0.0, "y": -2.0, "z": 0.0 }, "radius": 1.5 }
  ],
  "workspace": {
    "min": { "x": -10.0, "y": -10.0, "z": -10.0 },
    "max": { "x": 10.0, "y": 10.0, "z": 10.0 }
  },
  "seed": 7
}
```

```
$ hydra-umc-path-planner-3d scenarios/example.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
{
  "status": "ok",
  "path": [
    {
      "x": -5.0,
      "y": 0.0,
      "z": 0.0
    },
    {
      "x": -4.616648770504599,
      "y": -0.23208849045185795,
      "z": -0.22175835371895733
    },
    {
      "x": -4.479412317311736,
      "y": 0.130103055515109,
      "z": -0.5379599349058661
    },
    ... 30 more waypoints ...
    {
      "x": 4.883881202961467,
      "y": -0.020264429212978136,
      "z": -0.05034487943476104
    },
    {
      "x": 5.0,
      "y": 0.0,
      "z": 0.0
    }
  ]
}
```

(The real run produced 35 waypoints; the middle of the list is elided here
for length — nothing was cut from the actual JSON when this ran.) Exits `0`.

**Start point already inside an obstacle** — a real, distinct planner error,
not a generic "no path found" (exit `1`):

```
$ hydra-umc-path-planner-3d scenario-with-start-inside-obstacle.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
{
  "status": "error",
  "reason": "start_inside_obstacle"
}
```

`plan_error_reason` in `main.rs` maps every `PlanError` variant to one of
these `reason` strings: `start_inside_obstacle`, `goal_inside_obstacle`,
`start_outside_workspace`, `goal_outside_workspace`, `no_path_found`,
`time_limit_exceeded`.

**Missing scenario file** (real OS error text — this machine reports it in
Spanish; exit `1`):

```
$ hydra-umc-path-planner-3d scenarios/does-not-exist.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
[path-planner-3d] could not read scenarios/does-not-exist.json: El sistema no puede encontrar el archivo especificado. (os error 2)
```

**Malformed scenario JSON** (exit `1`):

```
$ echo '{not valid json' > malformed.json
$ hydra-umc-path-planner-3d malformed.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
[path-planner-3d] could not parse malformed.json: key must be a string at line 1 column 2
```

### `validate <scenario.json> <path.json>`

Re-checks an **already-computed** path (cached, replayed, or relayed from
another process) against a scenario's *current* obstacles and workspace,
without running a new search — a fail-safe gate before a robot actually
executes the path for real. `<path.json>` is a bare JSON array of
`{"x":.., "y":.., "z":..}` waypoints.

**A real, safe path** — the actual path this project's own planner produced
above for `scenarios/example.json`, re-validated against the same scenario:

```
$ hydra-umc-path-planner-3d validate scenarios/example.json real_path.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
{
  "status": "safe"
}
```

Exits `0`.

**An unsafe path** — a two-point straight line from the same start to the
same goal, which cuts straight through the obstacle centered at the origin
even though neither endpoint itself sits inside it (the segment-vs-obstacle
check `validate.rs` exists specifically to catch):

```json
[{"x":-5.0,"y":0.0,"z":0.0},{"x":5.0,"y":0.0,"z":0.0}]
```

```
$ hydra-umc-path-planner-3d validate scenarios/example.json unsafe_path.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
{
  "status": "unsafe",
  "issues": [
    {
      "SegmentIntersectsObstacle": {
        "from_index": 0,
        "to_index": 1
      }
    }
  ]
}
```

Exits `1`.

**Missing `<path.json>` argument** (exit `1`):

```
$ hydra-umc-path-planner-3d validate scenarios/example.json
HYDRA-UMC-PATH-PLANNER-3D v0.0.3
Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.
Usage: hydra-umc-path-planner-3d validate <scenario.json> <path.json>
<path.json> is a bare JSON array of {"x":.., "y":.., "z":..} waypoints.
```

## Exit codes

| Code | Meaning |
|------|---------|
| `0` | ok — a path was found (`status: "ok"`), or `validate` reports `status: "safe"`, or bare/no-argument usage output |
| `1` | planner error (`status: "error"`), `validate` reports `status: "unsafe"`, missing/unreadable/malformed scenario or path file, or missing `validate` arguments |

## Not yet wired in

This is a CLI driven by local JSON files, not a network service — there is
no HTTP/gRPC endpoint yet (see the module doc at the top of `src/main.rs`
for why exposing one is deferred). It also plans one agent per call, not
true swarm-wide multi-robot coordination, and does not yet do RRT* rewiring
or octree-accelerated collision checks at scale — see the project's own
`mejoras_futuras.txt` for the honest list of what's deferred and why.

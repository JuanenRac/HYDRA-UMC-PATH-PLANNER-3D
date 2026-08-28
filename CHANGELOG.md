# Changelog

All notable work on **HYDRA-UMC-PATH-PLANNER-3D** is summarized here, newest first. Full
session-by-session detail (including dates) lives in a private,
unpublished internal log - this file is public, so it intentionally
omits calendar dates.

## Versioning scheme

`Cargo.toml`'s `version` field is bumped automatically by
`bump_version.py`, run from `build.sh`/`build.bat` before every real
release build (`cargo build --release`).

It follows the ecosystem-wide base-10 "odometer" rule rather than
semantic-versioning judgment calls:

- `PATCH` +1 on every build
- when `PATCH` would exceed 9, it resets to 0 and `MINOR` +1 instead (e.g. `0.0.9` -> `0.1.0`, never `0.0.10`)
- the same carry cascades into `MAJOR` if `MINOR` would exceed 9

---

## [0.0.3] - Real time limit, obstacle corpus, and unsafe-trajectory rejection

- **`src/rrt.rs`** - `PlannerConfig` gains an opt-in `max_duration_ms: Option<u64>` (default `None`, so every existing scenario/test behaves exactly as before). `plan()` now checks wall-clock elapsed time each iteration and returns the new `PlanError::TimeLimitExceeded` if the budget runs out - `max_iterations` alone cannot bound wall-clock time, since a scene with more obstacles makes each iteration's collision checks proportionally slower, and this planner may sit in a real-time control loop that cannot wait indefinitely for an answer.
- **`src/corpus.rs`** (new, test-only) - a reusable corpus of obstacle/workspace fixtures (`open_workspace`, `single_blocking_obstacle`, `wall_of_obstacles`, `workspace_spanning_wall`) shared by `rrt.rs`'s and `validate.rs`'s own tests, replacing several ad-hoc obstacle scenes that used to be re-derived per test function.
- **`src/validate.rs`** (new) - real safety validation for an ALREADY-COMPUTED path: `plan()` only ever returns collision-free paths by construction, but a path from anywhere else (a cached/replayed plan, one relayed from another process, a hand-edited scenario) needs re-checking against the CURRENT obstacles/workspace before a robot actually executes it. `validate_path()` checks every waypoint against the workspace bounds and every obstacle, and every segment between consecutive waypoints (catching a straight line that clips an obstacle even when both of its endpoints are individually clear) - returns every issue found, not just the first.
- **`src/main.rs`** - new `validate <scenario.json> <path.json>` subcommand wraps `validate_path()`, printing `{"status":"safe"}` or `{"status":"unsafe","issues":[...]}` (exit 0/1); the existing bare `<scenario.json>` invocation is unchanged. `plan_error_reason` gained the `time_limit_exceeded` case.
- **`build.sh`** - fixed a version double-bump: the manifest-sync step ran `bump_manifest_version.py` without `--sync` *before* the native `bump_version.py` step, so `Cargo.toml` advanced twice per build while the manifest advanced once. Reordered to bump native first, then `--sync` after (matching `build.bat`'s already-correct order). Also added `cargo test` to both `build.sh` and `build.bat` - previously neither actually ran the test suite as part of a real build, despite advertising "verification" in their own banner text.
- 10 new tests (time-limit behavior in `rrt.rs`, all of `validate.rs`) - 28 total, all passing with zero warnings. Verified live: the planner's own real output for `scenarios/example.json` validates as `"safe"`; a hand-crafted straight line through the same scenario's obstacle wall is correctly flagged `"unsafe"` with a real `SegmentIntersectsObstacle`; and a scenario with `max_duration_ms: 0` correctly reports `"time_limit_exceeded"` instead of quietly running the full iteration budget.

## [0.0.2] - Real single-agent RRT path search

- **`src/geometry.rs`** - minimal `Vec3` math (add/sub/scale/dot/length/
  distance/normalized), with a defined (not NaN) result for normalizing
  a zero-length vector.
- **`src/obstacle.rs`** - sphere obstacles; point-in-obstacle and
  segment-vs-sphere intersection (closest-point projection, clamped to
  the segment) collision checks, both respecting a caller-supplied
  clearance (the planning agent's own radius).
- **`src/rng.rs`** - a small, dependency-free, deterministic PRNG
  (xorshift64\*) for RRT's random sampling - no `rand` crate pulled in
  for one generator, and determinism from a fixed seed is what makes the
  planner's own tests reproducible instead of flaky.
- **`src/rrt.rs`** - the real search: a standard RRT (not yet RRT\*,
  no rewiring pass) that grows a tree from `start` toward `goal` through
  a `Workspace`, biased toward the goal, rejecting any extension that
  would clip an obstacle, and returns the walked path once a tree node
  gets within `goal_threshold` of `goal` with a clear line of sight.
  Upfront validation rejects a `start`/`goal` that's already inside an
  obstacle or outside the workspace before searching at all.
  `PlanError::NoPathFound` is an honest outcome, not a bug: the search
  exhausted its iteration budget without connecting - it cannot tell a
  truly unreachable goal apart from one that just needed more
  iterations, and does not pretend to.
- **`src/main.rs`** - now a real CLI: loads a JSON scenario
  (`scenarios/example.json`), runs the planner, prints the result
  (`{"status":"ok","path":[...]}` or `{"status":"error","reason":"..."}`)
  - instead of only printing identity and exiting.
- Added `serde`/`serde_json` as the crate's first real dependencies (for
  scenario I/O) - still no async runtime or web framework.
- Verified for real: `cargo build`/`cargo build --release` clean; 18
  `cargo test` cases, including one that verifies every segment of a
  returned path is genuinely obstacle-clear (not just plausible-looking)
  by routing around a real 3-obstacle wall, one that verifies full
  determinism for a fixed seed, and one that constructs a real
  workspace-spanning obstacle (proven geometrically, not just "the
  search gave up") to verify `NoPathFound` is reported honestly.
  Additionally smoke-tested the compiled release binary end-to-end
  against `scenarios/example.json` - a real path routing around the
  example's 3-obstacle wall, printed as valid JSON.
- What's still not real, on purpose - see `mejoras_futuras.txt`: RRT\*
  (path optimality via rewiring), true multi-robot synchronized planning
  (today plans one agent against a static obstacle set, not the
  README's "32+ robots simultaneously"), octree/BVH-accelerated
  collision at scale, and exposing this as a network service (HTTP or
  the shared `hydra.common.v1` gRPC contract) instead of a CLI.

## [0.0.1] - Initial scaffolding

- **`src/main.rs`** - minimal real entry point. No planning logic yet - real-time 3D collision-free path planning across the cell's full robot roster lands in a later pass.
- **`Cargo.toml`** - crate metadata, no runtime dependencies yet.
- **`build.sh` / `build.bat`**, **`run.sh` / `run.bat`** - `cargo build --release` and run the resulting binary.

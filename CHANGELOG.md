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

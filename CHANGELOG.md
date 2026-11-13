# Changelog

All notable changes to this project are documented here.

## [Unreleased]

- **`POST /plan` plans a scenario sent in the body of a request (off by default, `plan` section of the configuration file of `serve`):** `{"plan": {"enabled": true, "max_body_bytes": 1048576, "max_duration_ms": 10000}}`. Closed, the route answers 404 like any unknown path. Open, it needs the bearer token when one is set; without a token only a caller on this machine is answered, and `serve` refuses to start with the route open on an address other than the loopback unless a token is set (`--allow-anonymous` is not enough). A body larger than `max_body_bytes` is refused with 413, the search never runs longer than `max_duration_ms` whatever the scenario asks for (504 when it hits the limit), a scenario that cannot be read or is not valid is 400, a start or goal that cannot be used or no path is 422, and a scenario cannot name a `zones_file` in a request (send the boxes). The answer is the one of the command line. The last 32 answers are kept by the text of the scenario: the search is deterministic for a text (its seed is part of it), so a repeated request gets the same answer without a new search, and a change of the obstacles is a different text. The service answers one request at a time, so a long search holds the others up to the limit. 5 new tests and a changed one; the configuration of the metrics is read by the same function and keeps its rules.
- **Several robots planned in the order of their priority (`fleet <fleet.json>`):** the first robot plans as usual; its path is then kept clear by every later robot as a row of spheres along its whole length (the robot radius, a little inflated, spaced so that no gap is left), and so on down the list. The result lists every robot with its path or its reason (`partial`, status 1, when one has none; the others still get their plan). This is the version without time: the row stands for the robot at every time at once, so a later robot never plans through a place an earlier one passes, which is safe and conservative; a fleet that could share a corridor by timing may get no path for a later robot. Planning with time as a dimension is the refinement still to do. 4 unit tests and 1 command-line test.
- **Planning for an arm in the space of its joint angles (`arm <arm-scenario.json>`):** the scenario describes a serial chain of revolute joints the way a URDF does (the `origin` of each joint in the frame of the one before, its `rpy`, its `axis` and its `lower` and `upper` limits), the `tip` of the last link, the `link_radius`, the angles of the `start` and the `goal` in radians, and the spheres and boxes to avoid. The position of every joint comes from forward kinematics with the convention of `fk` of the physics replica (origin, then Rz * Ry * Rx, then the turn about the axis), each link is a segment with its radius tested against the obstacles, and an RRT in the space of the angles (`config`: `step_rad`, `resolution_rad`, `goal_threshold_rad`, `goal_bias`, `max_iterations`) returns a list of angle vectors; a straight move that is free is returned as it is, so the arm can fold to get past an obstacle that blocks it. The same scenario and seed give the same path. The arm is not checked against itself, and nothing is computed by the replica (the chain is in the scenario, so its numbers can be compared with its `fk --envelope`). 6 unit tests and 1 command-line test.
- **The danger zones of a zones file of the safety zones are avoided (`zones_file` in the scenario):** every zone of the level `danger` of that file becomes a box that the plan and the independent validator keep the robot radius away from; the zones of the level `warning` are not used yet (they would be a cost). A zone that is not a box (an oriented box, a capsule, a cylinder) is refused by name instead of being made smaller, a file that cannot be read or has no zones list is refused as invalid input, and `shrink` does not handle a scenario with a zones file. A relative path is read from the working directory. 3 unit tests and 2 command-line tests.
- **Box obstacles next to the spheres (`boxes` in the scenario):** each box is two opposite corners, `min` and `max`, in metres. The plan keeps the robot radius away from a box as it does from a sphere (the box is inflated on every side, so the rounded corners of the sphere count as filled and the check stays on the safe side), a start or goal inside a box is refused as `start_inside_obstacle` or `goal_inside_obstacle`, and the independent validator checks every waypoint and segment against the boxes as well. A box with a corner that is not a number, or with `min` above `max`, is refused as invalid input. `shrink` does not handle boxes yet and says so. Scenarios without `boxes` behave as before. 5 unit tests of the box and 2 command-line tests.
- **The coordinates of the result can be rounded (`output_decimals` in the scenario, off by default):** a whole number from 0 to 12 rounds every coordinate of the path to that many decimals; without it the coordinates are written as computed, so existing callers and tests see the same numbers. A value above 12 is refused as invalid input. Every coordinate is in metres. 1 new test.

- **The planner can read its scenario from standard input and tell the causes of a failure apart by its exit status (`--exit-codes`, off by default):** `hydra-umc-path-planner-3d - [--exit-codes]` reads the scenario from standard input, so a caller can plan without a file in between. Without the switch every failure still ends with status 1, as before; with it, 1 means no path was found, 2 an input that cannot be used (an unreadable or malformed scenario, a start or goal inside an obstacle or outside the workspace) and 3 the time limit. 3 new tests.

- **A simulation bench and a coverage floor on CI:** the workflow `Simulation bench` runs the real binary through synthetic worlds drawn from a fixed seed (30 sparse worlds planned and re-checked by the independent validator, the same seed giving the same bytes, RRT* never longer than the plain search, a goal walled in by spheres answered with `no_path_found`, a wall crossed only through its gap) and injects faults (a waypoint pushed into an obstacle or out of the workspace is refused). The workflow `Coverage` measures the line coverage with `cargo llvm-cov` and fails under a floor. Four tests of the command line cover the refusals of `serve` (unknown option, open address without a token, unreadable or unknown configuration, an address that cannot be bound).
- **README: the platform statement:** a note says that where the README speaks of the CM5 it means the pilot bench that runs today, and that the target platform of HYDRA-UMC, an NVIDIA Jetson Orin NX 16 GB, is a design that has not been built or tested.
- **The collision checks of the search use a bounding-volume hierarchy:** `obstacle.rs` gains `ObstacleIndex`, built once per `plan()` call (median split on the widest axis, leaves of four spheres), so a segment check on a crowded scene visits only the obstacles whose boxes the segment crosses. It answers exactly what the scan of every obstacle answers (the leaves run the same sphere test and the boxes are padded so rounding cannot skip one), which 6 sets of up to 300 spheres and 9,000 random segments check against the scan, along with zero-length and axis-aligned segments, the clearance edge, an empty set and identical spheres. The independent safety validator keeps scanning every obstacle on purpose. The results of the search are unchanged; 6 new tests.
- **RRT*, off by default:** `config.optimize_iterations` (default `0`, the plain RRT as before) and `config.rewire_radius` (default `1.0`) switch on a search where every new node takes the cheapest collision-free parent within the radius, the neighbours that get cheaper through it are rewired (the cost of their whole subtree follows) and, after the first path, that many more iterations run to shorten it; the answer is the cheapest way to the goal over every node that sees it. For one seed the path is never longer than the plain RRT one, the same seed gives the same path, a time limit hit after a path was found returns it, and a radius that is not a finite number above zero is refused with `NonFiniteRewireRadius` or `NonPositiveRewireRadius` (read only when RRT* is on). 9 new tests in `rrt.rs` and `semantics.rs` and one of the command line.

## [0.1.4] - The build test of the repository compiles only on CI or when allowed: for the stacks that compile (rust, go, android, flutter, firmware-c), tools/buildtest.py prints BUILDTEST=SKIPPED with exit status 2 on a plain PC, never a...

- **The build test of the repository compiles only on CI or when allowed:** for the stacks that compile (`rust`, `go`, `android`, `flutter`, `firmware-c`), `tools/build_test.py` prints `BUILD_TEST=SKIPPED` with exit status 2 on a plain PC, never a pass; `ALLOW_LOCAL_BUILD=1` allows it and a CI runner always may. The `python` and `node` stacks are not locked.

## [0.1.3] - The service takes a configuration file, the metrics route is off by default and opens only from it, an optional HTTP listener serves health and metrics, and the documentation no longer says the twin validates the routes

- **`GET /metrics` is off by default and loopback-only:** `serve --config FILE` (or `PATH_PLANNER_CONFIG`) reads a JSON file whose `metrics` section opens it (`{"metrics": {"enabled": true}}`; an unknown key stops the start). Closed, the route answers `404`; open, it answers a caller on this machine, or any caller with the bearer token when the service has one.

- **An optional HTTP listener with `GET /health` and `GET /metrics`:** `serve [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous]` (the token can also come from `PATH_PLANNER_TOKEN`). It is off unless the command is given, listens on `127.0.0.1:8117` by default, refuses another address without a token (unless `--allow-anonymous`) and answers only those two routes: liveness, open, and the numbers of the service in the text format of Prometheus (the version, the uptime, the responses by class of status and the health), behind the token when one is set (`401` without it). It plans nothing and never publishes a scenario, a path or a token. The metrics code is the canonical `service_metrics.rs` of HYDRA-UMC-SDK, copied byte for byte; the only new dependency is `tiny_http`.

- **The documentation no longer says the twin validates the planned routes.** The related-projects entry, the ecosystem paragraph and the contributing guide said that routes are checked in the digital twin; nothing sends a route to the twin yet, and the text now says so.
## [0.1.2] - The test count out of the manifest notes and the universal README

- **The notes of the project manifest no longer carry a hand-written test count:** the counts went out of date at every change; the number of tests is stated by the README, which is checked against the code.
- **README in the universal layout:** the README and its seven translations follow the layout that the four families share (22 sections in a fixed order), with an architecture picture and a repository-structure picture drawn from `docs/architecture.json` and `docs/structure.json`; the CI validator checks both pictures and the file tree.

- **The layout of the README is checked in the CI:** the validator runs `tools/universal_readme_check.py` and lists, as a warning, what differs from the universal layout of the README.

## [0.1.1] - Building no longer changes the version; raising it is the explicit `bu

- Building no longer changes the version; raising it is the explicit `bump` command (`python tools/hydra_project_tool.py bump`). The build scripts only report the version they built, and CI fails if a build script runs a bump.
- The banner follows the animated style of the ecosystem banners, with the name, the kind and the technologies of the project.
- Added `tools/hydra_project_tool.py`: the same `validate`, `test`, `version`, `status` and `bump` commands in every repository. The checks run with `HYDRA_UMC_CI=1` and fail if the version changes; only `bump` changes it.

## [0.1.0]

- Maturity is functional: nothing here has been verified on real hardware yet.
- The READMEs say, in the eight languages, what is planned and what has only been tested against fakes, emulators or fixtures, and carry a short safety and security status note.

## [0.0.9]

- Build version synchronized with `hydra-umc.project.json` and the repository-native version source.
- The licence is written as GPL-3.0-or-later in every header and in the package metadata.

## [0.0.8] - The whole body stays inside the workspace, a path must reach the goal, and a long corridor no longer aborts

- **The workspace is the volume the whole body must stay in.** `robot_radius` only inflated the obstacles, so a centre touching a wall counted as inside. Start, goal, samples and waypoints must now keep `robot_radius` clear of the workspace walls (`plan` answers outside-workspace, `validate` reports the waypoint), and a workspace narrower than the robot is refused.
- **`validate` checks both ends of the path.** A path that does not begin at the scenario's `start` or end at its `goal` (within 1e-6) is reported as `PathDoesNotStartAtStart` / `PathDoesNotReachGoal`, so a cached path from another pose, or a single waypoint, is no longer `safe` while the jump from the robot's real pose to its first waypoint was never checked.
- **A long straight corridor no longer aborts the process.** The nearest-neighbour tree degenerated into a linked list when points arrived in a monotonic order (a goal bias of 1, a straight corridor) and its recursion overflowed the stack at around 40,000 nodes. It is now stored in one vector, inserted into and searched with loops and an explicit stack, and rebuilt balanced each time it doubles; answers are identical, ties included.
- **`max_iterations` is capped at 10,000,000** (`TooManyIterations`) instead of running for hours while the tree grows without bound.
- **Stdout is only the JSON result.** The identity lines go to stderr, so `planner scenario.json | jq` works and a caller need not strip lines.
- **A partial `config` works:** `{"robot_radius": 0.3}` takes the defaults for the rest, as the README always said.
- **No repeated final waypoint:** a goal-biased sample that landed on the goal no longer ends the path with a segment of length 0. A start equal to the goal is a path of one waypoint.
- Documentation: the package description, the badge and the diagram no longer present RRT*, potential fields, an octree, a trajectory optimiser or SWARM-SYNC dispatch as what the planner does; the README says HYDRA-UMC-TWIN does not validate routes; the directory listing has `kdtree.rs` and `shrink.rs`; `docs/CLI_REFERENCE.md` is current; the manifest `build` command has no stray dot; a comment in `bump_version.py` had the wrong example. 66 tests.

## [0.0.7] - Frame label

- A scenario may declare an optional frame name (at most 64 bytes); it is echoed in the plan and validation result and never changes a coordinate. An empty or over-long name is refused.

## [0.0.6] - Real incremental KD-tree for nearest-neighbor lookup

- **`kdtree.rs`** (new) - a real, incremental 3D KD-tree replacing
  `rrt.rs`'s own linear scan, the deliberate v0 choice that file's own
  header comment already named as real follow-up work once proven
  correct. Insertion-only (no rebalancing - the RRT tree only ever
  grows during a single `plan()` call, never removes a node), with
  explicit lowest-index tie-breaking so a query with more than one
  equidistant match returns the exact same index the old
  `Iterator::min_by`-based linear scan would have - verified directly
  against a real, independent linear scan across many random
  insertion/query sequences (500 points x 20 seeds, checked
  incrementally as the tree grows, not just once at the end), not just
  "the distance looks right". `rrt.rs`'s own existing seed-determinism
  tests pass unchanged, confirming real planning output is identical to
  before this swap. 6 new tests, 51 total.

## [0.0.5] - real case-minimization for a failing scenario

`validate.rs` already answers "is this exact path safe against this
exact scenario, right now" - the independent verifier the idea calls
for. What was still missing: when the one real soundness property this
crate's own tests check by hand for a handful of curated fixtures
(`plan()` must never hand back a path its own independent validator
would call unsafe) ever breaks on a larger scene, a human had no way to
reduce that failure down to something small enough to actually debug.

New `src/shrink.rs`: `planner_output_is_unsafe()` runs the real
plan()+validate_path() pipeline and reports the real issues found, never
conflating a genuine "unsafe path" with the honestly different "no path
found" outcome. `shrink_scenario()` reduces a failing scenario to the
smallest one that still reproduces the *exact same* failure - deleting
obstacles one at a time to a fixed point, then binary-searching the
workspace bounds toward the start/goal bounding box, then binary-
searching `max_iterations` down - re-checking the real failure predicate
after every tentative change, so a "shrunk" result can never silently
stop reproducing (the idea's own explicit warning: never relax
obstacles just to make the test pass). New `shrink <scenario.json>
[--out <minimal.json>]` CLI subcommand reports `no_failure` honestly
when there is nothing to reduce, or the minimized scenario plus its real
issues, in the same JSON shape the plain CLI invocation already accepts.

Deliberately scoped to deletion + bound-tightening, not obstacle
repositioning - stated honestly as real, separate future work in
`shrink.rs`'s own module doc.

8 new tests, 48 total. `cargo fmt`/`cargo clippy --all-targets -D
warnings`/`cargo test --all-targets` all clean. `docs/CLI_REFERENCE.md` +
README x7 synced.

## [0.0.4] - Reject a semantically invalid scenario before any geometry check

- **Negative-radius scenario reported as "safe":**
  `Obstacle::contains_point`/`intersects_segment` compute an
  effective collision radius as `obstacle.radius + robot_radius` - a
  negative `robot_radius` (e.g. `-2` against an obstacle radius of `1`)
  made that effective radius negative, which no real, non-negative
  distance can ever satisfy. `validate` reported `status: "safe"` for a
  straight segment that visibly crossed the obstacle's own sphere. New
  `src/semantics.rs` is a shared gate both `rrt::plan()` and
  `validate::validate_path()` now run BEFORE any geometry check: a
  negative/non-finite robot or obstacle radius, an inverted workspace
  bound, or an out-of-range planner parameter (non-positive `step_size`,
  `goal_bias` outside `[0.0, 1.0]`, a negative `goal_threshold`, zero
  `max_iterations`) is rejected with a real, structured reason - never
  silently treated as an honest "safe" or "no path found" verdict. New
  `PlanError::InvalidInput`/`PathSafetyIssue::InvalidScenario` variants
  carry that reason through to the CLI's own JSON output. 10 new
  regression tests reproduce the finding's own exact scenario (robot
  radius -2, obstacle radius 1, a segment straight through its center)
  for both `plan()` and `validate_path()`, plus the full semantic-gate
  matrix. `cargo fmt --check`/`clippy --all-targets -- -D warnings`/
  `test --all-targets` (40/40): all passing.
- **Inline deferral reasons:** `mejoras_futuras.txt` here is a real,
  tracked, public file - but every one of its
  20 cross-references (`CHANGELOG.md`, `docs/CLI_REFERENCE.md`,
  `src/main.rs`, `src/obstacle.rs`, `src/rrt.rs`, README in all 7
  languages) required the reader to jump elsewhere for a short reason
  that fits in the sentence itself. Each site now states its own reason
  inline; `mejoras_futuras.txt` stays as the one place all four deferred
  items are written up in full, and is now listed in the README's own
  directory structure (all 7 languages) so it stays discoverable now
  that nothing links to it inline anymore.

- **`src/geometry.rs` / `src/validate.rs`** - the execution-side path gate now
  rejects waypoints containing `NaN` or infinity explicitly and skips further
  meaningless geometry operations for their segments. A malformed relayed or
  cached path therefore remains unsafe rather than depending on floating-point
  comparison side effects.

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
- What's still not real, on purpose: RRT\*
  (path optimality via rewiring), true multi-robot synchronized planning
  (today plans one agent against a static obstacle set, not the
  README's "32+ robots simultaneously"), octree/BVH-accelerated
  collision at scale, and exposing this as a network service (HTTP or
  the shared `hydra.common.v1` gRPC contract) instead of a CLI.

## [0.0.1] - Initial scaffolding

- **`src/main.rs`** - minimal real entry point. No planning logic yet - real-time 3D collision-free path planning across the cell's full robot roster lands in a later pass.
- **`Cargo.toml`** - crate metadata, no runtime dependencies yet.
- **`build.sh` / `build.bat`**, **`run.sh` / `run.bat`** - `cargo build --release` and run the resulting binary.

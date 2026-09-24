// HYDRA-UMC-PATH-PLANNER-3D - shrink.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// ("Verificador independiente y reduccion de casos fallidos"):
// validate.rs already answers "is this exact path safe against this
// exact scenario, right now" - the independent verifier the idea calls
// for. What was still missing: when a real property this crate cares
// about breaks (the core one being `planner_output_is_unsafe` below -
// plan() is supposed to only ever hand back paths validate_path() would
// call safe, by construction), a human should never have to debug that
// failure against the full-size scene that happened to trigger it.
// `shrink_scenario()` reduces obstacles/workspace/iteration-budget down
// to the smallest scenario that still reproduces the EXACT SAME failure
// - never a scenario that merely looks smaller while testing something
// else, which is exactly the "no relajar obstaculos para que el test
// pase" the idea's own acceptance test warns against: every tentative
// reduction is re-checked against the real failure predicate before
// being kept, so a "shrunk" result can never silently stop reproducing.
//
// Scope, stated honestly: this shrinks by DELETING obstacles and
// TIGHTENING the workspace/iteration budget - it never relocates an
// obstacle or perturbs start/goal/robot_radius. Repositioning-based
// shrinking (finding the smallest DISPLACEMENT of an obstacle that
// still triggers the failure, not just the smallest COUNT) is real,
// separate future work; deletion-based reduction is the well-understood,
// safe-to-implement-correctly core of delta-debugging, and already gets
// a human from "a 40-obstacle scene failed somewhere" to "these 2
// obstacles are the whole story".

use crate::geometry::Vec3;
use crate::obstacle::Obstacle;
use crate::rrt::{plan, PlannerConfig, Workspace};
use crate::validate::{validate_path, PathSafetyIssue};
use serde::{Deserialize, Serialize};

/// A fully-resolved scenario - every field always present, unlike
/// main.rs's own `Scenario` (which lets `config`/`seed` be omitted and
/// defaulted for a hand-authored file). Serializes to the exact same
/// field shape, so a minimized scenario written to disk is directly
/// replayable by the plain CLI invocation (`hydra-umc-path-planner-3d
/// minimal-failure.json`) - a human never has to hand-translate it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    pub start: Vec3,
    pub goal: Vec3,
    pub obstacles: Vec<Obstacle>,
    pub workspace: Workspace,
    pub config: PlannerConfig,
    pub seed: u64,
    /// Name of the coordinate frame the scenario is written in. A label
    /// only: it is carried through unchanged and never used to transform
    /// a coordinate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frame: Option<String>,
}

/// The one real soundness property this crate's own test suite already
/// checks by hand for a handful of curated fixtures
/// (`a_real_planner_output_always_validates_as_safe`): `plan()` must
/// never hand back a path its own independent validator considers
/// unsafe. Returns the real, specific issues found when that property
/// breaks, `None` when it holds - including when `plan()` itself fails
/// to find a path at all (`NoPathFound`/`TimeLimitExceeded`/etc.), which
/// is a genuinely different, honest outcome, never conflated with
/// "unsafe" here - shrinking a "no path found" case down to a smaller
/// "still no path found" case is a different, legitimate question this
/// function deliberately does not answer.
pub fn planner_output_is_unsafe(scenario: &Scenario) -> Option<Vec<PathSafetyIssue>> {
    let path = plan(
        scenario.start,
        scenario.goal,
        &scenario.obstacles,
        scenario.workspace,
        scenario.config,
        scenario.seed,
    )
    .ok()?;
    let issues = validate_path(
        &path,
        &scenario.obstacles,
        &scenario.workspace,
        scenario.config.robot_radius,
    );
    if issues.is_empty() {
        None
    } else {
        Some(issues)
    }
}

/// Reduces `scenario` to the smallest one, by this function's own real
/// rules below, that still satisfies `is_failure` - the caller-supplied
/// real failure predicate (typically
/// `|s| planner_output_is_unsafe(s).is_some()`, but kept generic so a
/// test can shrink against a synthetic predicate without needing a real
/// planner bug to exist). Returns `scenario` completely unchanged if it
/// does not already satisfy `is_failure` - shrinking a case that was
/// never actually failing would silently fabricate a "minimal failure"
/// that reproduces nothing.
///
/// Real reduction passes, in this fixed order, each re-running
/// `is_failure` after every tentative change and keeping it ONLY when
/// the exact same failure still reproduces:
/// 1. Remove obstacles one at a time, repeating full passes until none
///    can be removed - the dominant real win for a scene with many
///    obstacles where only a few are actually load-bearing for the bug.
/// 2. Tighten each of the workspace's 6 bound components toward the
///    start/goal bounding box via binary search.
/// 3. Binary-search `max_iterations` down toward the smallest budget
///    that still reproduces - a faster, more focused regression test.
///
/// `seed`, `start`, `goal`, and every other config field are never
/// touched - determinism and the scenario's own real geometry endpoints
/// are load-bearing for what "the same failure" even means.
pub fn shrink_scenario(scenario: Scenario, is_failure: impl Fn(&Scenario) -> bool) -> Scenario {
    if !is_failure(&scenario) {
        return scenario;
    }
    let mut current = scenario;
    current = shrink_obstacles(current, &is_failure);
    current = shrink_workspace(current, &is_failure);
    current = shrink_iterations(current, &is_failure);
    current
}

fn shrink_obstacles(mut scenario: Scenario, is_failure: &impl Fn(&Scenario) -> bool) -> Scenario {
    loop {
        let mut removed_one = false;
        let mut index = 0;
        while index < scenario.obstacles.len() {
            let mut candidate = scenario.clone();
            candidate.obstacles.remove(index);
            if is_failure(&candidate) {
                scenario = candidate;
                removed_one = true;
                // Do not advance `index`: the next obstacle just shifted
                // into this slot.
            } else {
                index += 1;
            }
        }
        if !removed_one {
            return scenario;
        }
    }
}

/// Binary-searches one scalar bound toward `target`, keeping the
/// tightest value (closest to `target`) that still reproduces the
/// failure. `apply` writes a candidate value into a fresh clone of
/// `scenario`'s workspace; `current` is the bound's own present value.
fn shrink_bound(
    scenario: &Scenario,
    is_failure: &impl Fn(&Scenario) -> bool,
    current: f64,
    target: f64,
    apply: impl Fn(&mut Workspace, f64),
) -> f64 {
    if current == target {
        return current;
    }
    let mut best = current;
    let mut lo = current;
    let mut hi = target;
    // A fixed number of bisections rather than looping to exact
    // convergence - real floating-point workspace bounds do not need
    // infinite precision to be "minimal" in any way a human would
    // notice, and a fixed budget makes this provably terminating.
    for _ in 0..40 {
        let mid = lo + (hi - lo) / 2.0;
        if (mid - lo).abs() < 1e-9 || (mid - hi).abs() < 1e-9 {
            break;
        }
        let mut candidate = scenario.clone();
        apply(&mut candidate.workspace, mid);
        if is_failure(&candidate) {
            best = mid;
            hi = mid;
        } else {
            lo = mid;
        }
    }
    best
}

fn shrink_workspace(mut scenario: Scenario, is_failure: &impl Fn(&Scenario) -> bool) -> Scenario {
    let target_min = Vec3::new(
        scenario.start.x.min(scenario.goal.x),
        scenario.start.y.min(scenario.goal.y),
        scenario.start.z.min(scenario.goal.z),
    );
    let target_max = Vec3::new(
        scenario.start.x.max(scenario.goal.x),
        scenario.start.y.max(scenario.goal.y),
        scenario.start.z.max(scenario.goal.z),
    );

    scenario.workspace.min.x = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.min.x,
        target_min.x,
        |w, v| w.min.x = v,
    );
    scenario.workspace.min.y = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.min.y,
        target_min.y,
        |w, v| w.min.y = v,
    );
    scenario.workspace.min.z = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.min.z,
        target_min.z,
        |w, v| w.min.z = v,
    );
    scenario.workspace.max.x = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.max.x,
        target_max.x,
        |w, v| w.max.x = v,
    );
    scenario.workspace.max.y = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.max.y,
        target_max.y,
        |w, v| w.max.y = v,
    );
    scenario.workspace.max.z = shrink_bound(
        &scenario,
        is_failure,
        scenario.workspace.max.z,
        target_max.z,
        |w, v| w.max.z = v,
    );
    scenario
}

fn shrink_iterations(mut scenario: Scenario, is_failure: &impl Fn(&Scenario) -> bool) -> Scenario {
    let mut lo: u32 = 0;
    let mut hi: u32 = scenario.config.max_iterations;
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        let mut candidate = scenario.clone();
        candidate.config.max_iterations = mid;
        if is_failure(&candidate) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    scenario.config.max_iterations = hi;
    scenario
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corpus;

    fn base_scenario() -> Scenario {
        Scenario {
            start: corpus::start(),
            goal: corpus::goal(),
            obstacles: corpus::wall_of_obstacles(),
            workspace: corpus::open_workspace(),
            config: PlannerConfig {
                max_iterations: 20_000,
                ..Default::default()
            },
            seed: 7,
            frame: None,
        }
    }

    #[test]
    fn shrink_scenario_returns_input_unchanged_when_it_does_not_actually_fail() {
        // The real safety net the idea's own acceptance test asks for:
        // never fabricate a "minimal failure" out of a case that was
        // never failing in the first place.
        let scenario = base_scenario();
        let shrunk = shrink_scenario(scenario.clone(), |_| false);
        assert_eq!(shrunk.obstacles.len(), scenario.obstacles.len());
        assert_eq!(shrunk.workspace.min.x, scenario.workspace.min.x);
    }

    #[test]
    fn shrink_scenario_removes_every_obstacle_a_synthetic_predicate_does_not_need() {
        // Synthetic predicate (not a real planner bug): "fails" as long
        // as at least 2 obstacles remain. Proves the deletion pass
        // converges to the true minimum this predicate allows, not just
        // "removed some".
        let mut scenario = base_scenario();
        scenario.obstacles = vec![
            Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(1.0, 0.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(2.0, 0.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(3.0, 0.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(4.0, 0.0, 0.0), 1.0),
        ];
        let is_failure = |s: &Scenario| s.obstacles.len() >= 2;

        let shrunk = shrink_scenario(scenario.clone(), is_failure);

        assert_eq!(
            shrunk.obstacles.len(),
            2,
            "must reduce to the true minimum, not stop early"
        );
        assert!(
            is_failure(&shrunk),
            "the shrunk scenario must still reproduce the real failure"
        );
    }

    #[test]
    fn shrink_scenario_never_produces_a_result_that_stops_reproducing() {
        // The idea's own literal acceptance test, generalized: whatever
        // shrink_scenario returns, re-running the SAME predicate against
        // it must still say "yes, this fails" - a shrinker that silently
        // drifted off the real failure (e.g. by relaxing obstacles past
        // the point that still matters) would violate this immediately.
        let mut scenario = base_scenario();
        scenario.obstacles = vec![
            Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(0.0, 2.0, 0.0), 1.0),
            Obstacle::new(Vec3::new(0.0, -2.0, 0.0), 1.0),
        ];
        let is_failure = |s: &Scenario| !s.obstacles.is_empty();

        let shrunk = shrink_scenario(scenario, is_failure);
        assert!(is_failure(&shrunk));
    }

    #[test]
    fn shrink_scenario_tightens_the_workspace_toward_start_and_goal() {
        let mut scenario = base_scenario();
        scenario.obstacles = Vec::new();
        // Predicate cares only about the workspace being at least this
        // large - proves the bound-shrinking pass actually moves bounds,
        // not just leaves them alone.
        let is_failure = |s: &Scenario| s.workspace.max.x >= 6.0;

        let shrunk = shrink_scenario(scenario, is_failure);

        assert!(
            shrunk.workspace.max.x >= 6.0,
            "must not overshoot past what the predicate still needs"
        );
        assert!(
            shrunk.workspace.max.x < 10.0,
            "must actually tighten from the original 10.0"
        );
    }

    #[test]
    fn shrink_scenario_reduces_max_iterations_toward_the_true_minimum() {
        let scenario = base_scenario();
        let is_failure = |s: &Scenario| s.config.max_iterations >= 100;

        let shrunk = shrink_scenario(scenario, is_failure);

        assert_eq!(shrunk.config.max_iterations, 100);
    }

    #[test]
    fn shrink_scenario_never_touches_seed_start_or_goal() {
        let scenario = base_scenario();
        let is_failure = |_: &Scenario| true; // trivially "always failing"
        let shrunk = shrink_scenario(scenario.clone(), is_failure);
        assert_eq!(shrunk.seed, scenario.seed);
        assert_eq!(shrunk.start, scenario.start);
        assert_eq!(shrunk.goal, scenario.goal);
    }

    #[test]
    fn planner_output_is_unsafe_is_none_for_every_real_curated_corpus_fixture() {
        // A real regression check even without a known bug today: every
        // curated fixture this crate ships must keep satisfying the one
        // property shrink_scenario exists to catch violations of.
        for obstacles in [
            corpus::no_obstacles(),
            corpus::single_blocking_obstacle(),
            corpus::wall_of_obstacles(),
        ] {
            let scenario = Scenario {
                start: corpus::start(),
                goal: corpus::goal(),
                obstacles,
                workspace: corpus::open_workspace(),
                config: PlannerConfig {
                    max_iterations: 20_000,
                    ..Default::default()
                },
                seed: 7,
                frame: None,
            };
            assert!(
                planner_output_is_unsafe(&scenario).is_none(),
                "a real curated fixture must never violate the planner's own safety invariant"
            );
        }
    }

    #[test]
    fn planner_output_is_unsafe_is_none_when_no_path_was_found_at_all() {
        // The other real half of this function's own contract: "no path
        // found" is a distinct, honest outcome from "unsafe path found" -
        // never conflated.
        let (workspace, obstacles) = corpus::workspace_spanning_wall();
        let scenario = Scenario {
            start: corpus::start(),
            goal: corpus::goal(),
            obstacles,
            workspace,
            config: PlannerConfig {
                max_iterations: 200,
                ..Default::default()
            },
            seed: 3,
            frame: None,
        };
        assert!(planner_output_is_unsafe(&scenario).is_none());
    }
}

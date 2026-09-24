// HYDRA-UMC-PATH-PLANNER-3D - entry point
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// Real single-agent RRT collision-free path search (src/rrt.rs), driven
// by a JSON scenario file - not yet a network service. Why a CLI first:
// exposing this over HTTP or the ecosystem's shared gRPC contract
// (hydra.common.v1, see HYDRA-UMC-ORCHESTRATOR/proto/) is a real
// decision (which protocol, what message shape) that deserves its own
// pass once a real caller (HYDRA-UMC-JOB-DISPATCHER) is ready to
// integrate against it. Proving the planning algorithm itself correct,
// with a scriptable interface that's still genuinely usable today, came
// first.
//
// What this does NOT do yet, honestly: multi-robot synchronized
// planning (one agent per call, not the swarm-wide 32+ robot
// coordination the README describes), RRT* rewiring for path
// optimality, and octree-accelerated collision checks at scale - real,
// deliberately deferred future work, not forgotten.

mod geometry;
mod kdtree;
mod obstacle;
mod rng;
mod rrt;
mod semantics;
mod shrink;
mod validate;

#[cfg(test)]
mod corpus;

use geometry::Vec3;
use obstacle::Obstacle;
use rrt::{plan, PlanError, PlannerConfig, Workspace};
use serde::{Deserialize, Serialize};
use shrink::{planner_output_is_unsafe, shrink_scenario};
use std::env;
use std::fs;
use std::process::ExitCode;
use validate::{validate_path, PathSafetyIssue};

const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Deserialize)]
struct Scenario {
    start: Vec3,
    goal: Vec3,
    #[serde(default)]
    obstacles: Vec<Obstacle>,
    workspace: Workspace,
    #[serde(default)]
    config: Option<PlannerConfig>,
    #[serde(default)]
    seed: Option<u64>,
    #[serde(default)]
    frame: Option<String>,
}

impl Scenario {
    /// Fills in the same defaults the bare invocation already applies
    /// (`config` -> `PlannerConfig::default()`, `seed` -> `1`), turning
    /// this sparse, hand-authorable shape into shrink.rs's always-fully-
    /// resolved one.
    fn resolve(self) -> shrink::Scenario {
        shrink::Scenario {
            start: self.start,
            goal: self.goal,
            obstacles: self.obstacles,
            workspace: self.workspace,
            config: self.config.unwrap_or_default(),
            seed: self.seed.unwrap_or(1),
            frame: self.frame,
        }
    }
}

#[derive(Serialize)]
#[serde(tag = "status")]
enum PlanOutcome {
    #[serde(rename = "ok")]
    Ok {
        path: Vec<Vec3>,
        #[serde(skip_serializing_if = "Option::is_none")]
        frame: Option<String>,
    },
    #[serde(rename = "error")]
    Error { reason: String },
}

const MAX_FRAME_NAME_LEN: usize = 64;

/// The frame name a scenario declares, if any. It is a label carried into
/// the result so a caller can check the path is in the frame it expects; it
/// never changes a coordinate. An empty or over-long name is refused rather
/// than passed on.
fn checked_frame(frame: Option<&str>) -> Result<Option<String>, String> {
    match frame {
        None => Ok(None),
        Some(name) if name.trim().is_empty() => Err("frame must not be empty".to_string()),
        Some(name) if name.len() > MAX_FRAME_NAME_LEN => {
            Err(format!("frame must be at most {MAX_FRAME_NAME_LEN} bytes"))
        }
        Some(name) => Ok(Some(name.to_string())),
    }
}

fn plan_error_reason(e: PlanError) -> String {
    match e {
        PlanError::StartInsideObstacle => "start_inside_obstacle".to_string(),
        PlanError::GoalInsideObstacle => "goal_inside_obstacle".to_string(),
        PlanError::StartOutsideWorkspace => "start_outside_workspace".to_string(),
        PlanError::GoalOutsideWorkspace => "goal_outside_workspace".to_string(),
        PlanError::NoPathFound => "no_path_found".to_string(),
        PlanError::TimeLimitExceeded => "time_limit_exceeded".to_string(),
        // carries the real, specific reason (which numeric
        // input was invalid) instead of a single generic label - an
        // operator seeing this in the JSON output can tell exactly what
        // to fix in the scenario file.
        PlanError::InvalidInput(reason) => format!("invalid_input: {reason:?}"),
    }
}

#[derive(Serialize)]
#[serde(tag = "status")]
enum ValidateOutcome {
    #[serde(rename = "safe")]
    Safe {
        #[serde(skip_serializing_if = "Option::is_none")]
        frame: Option<String>,
    },
    #[serde(rename = "unsafe")]
    Unsafe { issues: Vec<PathSafetyIssue> },
}

/// Validates an ALREADY-COMPUTED path (e.g. cached/replayed, or relayed
/// from another process) against a scenario's CURRENT obstacles and
/// workspace, without running a new search - a fail-safe re-check
/// before a robot actually executes the path for real. New subcommand
/// alongside the unchanged bare `<scenario.json>` invocation.
fn run_validate(scenario_path: &str, path_path: &str) -> ExitCode {
    let scenario_raw = match fs::read_to_string(scenario_path) {
        Ok(raw) => raw,
        Err(e) => {
            eprintln!("[path-planner-3d] could not read {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let scenario: Scenario = match serde_json::from_str(&scenario_raw) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[path-planner-3d] could not parse {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let path_raw = match fs::read_to_string(path_path) {
        Ok(raw) => raw,
        Err(e) => {
            eprintln!("[path-planner-3d] could not read {path_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let path: Vec<Vec3> = match serde_json::from_str(&path_raw) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("[path-planner-3d] could not parse {path_path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let frame = match checked_frame(scenario.frame.as_deref()) {
        Ok(frame) => frame,
        Err(reason) => {
            eprintln!("[path-planner-3d] invalid scenario {scenario_path}: {reason}");
            return ExitCode::FAILURE;
        }
    };
    let robot_radius = scenario.config.unwrap_or_default().robot_radius;
    let issues = validate_path(
        &path,
        &scenario.obstacles,
        &scenario.workspace,
        robot_radius,
    );
    let is_safe = issues.is_empty();
    let outcome = if is_safe {
        ValidateOutcome::Safe { frame }
    } else {
        ValidateOutcome::Unsafe { issues }
    };

    match serde_json::to_string_pretty(&outcome) {
        Ok(json) => println!("{json}"),
        Err(e) => {
            eprintln!("[path-planner-3d] could not serialize result: {e}");
            return ExitCode::FAILURE;
        }
    }

    if is_safe {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

#[derive(Serialize)]
#[serde(tag = "status")]
enum ShrinkOutcome {
    /// this project's own literal acceptance point, made explicit rather than
    /// implied: a scenario that never actually violated the planner's
    /// own safety invariant has nothing to shrink - reported honestly,
    /// never fabricated into a fake "minimal failure".
    #[serde(rename = "no_failure")]
    NoFailure,
    #[serde(rename = "minimal_failure")]
    MinimalFailure {
        issues: Vec<PathSafetyIssue>,
        scenario: Box<shrink::Scenario>,
    },
}

/// ("Verificador independiente y reduccion de casos fallidos"): given
/// a scenario where `plan()`'s own output fails its own independent
/// validator (a real soundness bug, not "no path found" - see
/// shrink::planner_output_is_unsafe's own doc), reduces it to the
/// smallest scenario that still reproduces the EXACT SAME failure and
/// reports it - to stdout, or to `out_path` when given, always in the
/// same JSON shape the plain CLI invocation itself accepts, so the
/// minimized case is directly replayable.
fn run_shrink(scenario_path: &str, out_path: Option<&str>) -> ExitCode {
    let raw = match fs::read_to_string(scenario_path) {
        Ok(raw) => raw,
        Err(e) => {
            eprintln!("[path-planner-3d] could not read {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let scenario: Scenario = match serde_json::from_str(&raw) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[path-planner-3d] could not parse {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let resolved = scenario.resolve();

    let outcome = match planner_output_is_unsafe(&resolved) {
        None => ShrinkOutcome::NoFailure,
        Some(_) => {
            let minimal = shrink_scenario(resolved, |s| planner_output_is_unsafe(s).is_some());
            // Re-derive the issues against the ACTUAL minimized scenario
            // (never reuse the original scenario's issue list) - the
            // real failure that survives shrinking is the one worth
            // reporting, and shrinking obstacles/workspace can change
            // which specific segments/waypoints it names.
            let issues = planner_output_is_unsafe(&minimal)
                .expect("shrink_scenario's own contract: its result must still satisfy is_failure");
            ShrinkOutcome::MinimalFailure {
                issues,
                scenario: Box::new(minimal),
            }
        }
    };

    let json = match serde_json::to_string_pretty(&outcome) {
        Ok(json) => json,
        Err(e) => {
            eprintln!("[path-planner-3d] could not serialize result: {e}");
            return ExitCode::FAILURE;
        }
    };

    match out_path {
        Some(path) => {
            if let Err(e) = fs::write(path, &json) {
                eprintln!("[path-planner-3d] could not write {path}: {e}");
                return ExitCode::FAILURE;
            }
            println!("[path-planner-3d] wrote {path}");
        }
        None => println!("{json}"),
    }

    match outcome {
        ShrinkOutcome::NoFailure => ExitCode::SUCCESS,
        // A real failure was found (and minimized) - non-zero so a CI
        // script invoking this directly notices, same convention the
        // bare invocation's own PlanOutcome::Error already uses.
        ShrinkOutcome::MinimalFailure { .. } => ExitCode::FAILURE,
    }
}

fn main() -> ExitCode {
    println!("HYDRA-UMC-PATH-PLANNER-3D v{VERSION}");
    println!("Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.");

    let args: Vec<String> = env::args().collect();

    if args.get(1).map(String::as_str) == Some("validate") {
        let (Some(scenario_path), Some(path_path)) = (args.get(2), args.get(3)) else {
            eprintln!("Usage: hydra-umc-path-planner-3d validate <scenario.json> <path.json>");
            eprintln!(
                "<path.json> is a bare JSON array of {{\"x\":.., \"y\":.., \"z\":..}} waypoints."
            );
            return ExitCode::FAILURE;
        };
        return run_validate(scenario_path, path_path);
    }

    if args.get(1).map(String::as_str) == Some("shrink") {
        let Some(scenario_path) = args.get(2) else {
            eprintln!(
                "Usage: hydra-umc-path-planner-3d shrink <scenario.json> [--out <minimal.json>]"
            );
            return ExitCode::FAILURE;
        };
        let out_path = if args.get(3).map(String::as_str) == Some("--out") {
            match args.get(4) {
                Some(p) => Some(p.as_str()),
                None => {
                    eprintln!("--out requires a file path");
                    return ExitCode::FAILURE;
                }
            }
        } else {
            None
        };
        return run_shrink(scenario_path, out_path);
    }

    let Some(scenario_path) = args.get(1) else {
        eprintln!("Usage: hydra-umc-path-planner-3d <scenario.json>");
        eprintln!("       hydra-umc-path-planner-3d validate <scenario.json> <path.json>");
        eprintln!("       hydra-umc-path-planner-3d shrink <scenario.json> [--out <minimal.json>]");
        eprintln!("See scenarios/example.json for the expected format.");
        return ExitCode::SUCCESS; // printing identity and usage is a valid no-arg invocation, not a failure
    };

    let raw = match fs::read_to_string(scenario_path) {
        Ok(raw) => raw,
        Err(e) => {
            eprintln!("[path-planner-3d] could not read {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let scenario: Scenario = match serde_json::from_str(&raw) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[path-planner-3d] could not parse {scenario_path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let frame = match checked_frame(scenario.frame.as_deref()) {
        Ok(frame) => frame,
        Err(reason) => {
            let outcome = PlanOutcome::Error {
                reason: format!("invalid_input: {reason}"),
            };
            match serde_json::to_string_pretty(&outcome) {
                Ok(json) => println!("{json}"),
                Err(e) => eprintln!("[path-planner-3d] could not serialize result: {e}"),
            }
            return ExitCode::FAILURE;
        }
    };
    let config = scenario.config.unwrap_or_default();
    let seed = scenario.seed.unwrap_or(1);

    let outcome = match plan(
        scenario.start,
        scenario.goal,
        &scenario.obstacles,
        scenario.workspace,
        config,
        seed,
    ) {
        Ok(path) => PlanOutcome::Ok { path, frame },
        Err(e) => PlanOutcome::Error {
            reason: plan_error_reason(e),
        },
    };

    let is_ok = matches!(outcome, PlanOutcome::Ok { .. });
    match serde_json::to_string_pretty(&outcome) {
        Ok(json) => println!("{json}"),
        Err(e) => {
            eprintln!("[path-planner-3d] could not serialize result: {e}");
            return ExitCode::FAILURE;
        }
    }

    if is_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

#[cfg(test)]
mod frame_tests {
    use super::*;

    #[test]
    fn no_frame_is_allowed_and_stays_absent() {
        assert_eq!(checked_frame(None), Ok(None));
    }

    #[test]
    fn a_named_frame_is_carried_unchanged() {
        assert_eq!(
            checked_frame(Some("robot_base")),
            Ok(Some("robot_base".to_string()))
        );
    }

    #[test]
    fn an_empty_or_over_long_frame_is_refused() {
        assert!(checked_frame(Some("")).is_err());
        assert!(checked_frame(Some("   ")).is_err());
        assert!(checked_frame(Some(&"x".repeat(MAX_FRAME_NAME_LEN + 1))).is_err());
        assert!(checked_frame(Some(&"x".repeat(MAX_FRAME_NAME_LEN))).is_ok());
    }

    #[test]
    fn the_frame_appears_in_the_output_only_when_given() {
        let with = serde_json::to_value(PlanOutcome::Ok {
            path: vec![],
            frame: Some("world".into()),
        })
        .unwrap();
        assert_eq!(with["frame"], "world");
        let without = serde_json::to_value(PlanOutcome::Ok {
            path: vec![],
            frame: None,
        })
        .unwrap();
        assert!(without.get("frame").is_none());
    }

    #[test]
    fn a_scenario_file_may_declare_a_frame() {
        let raw = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":1,"y":0,"z":0},
            "workspace":{"min":{"x":-2,"y":-2,"z":-2},"max":{"x":2,"y":2,"z":2}},"frame":"table"}"#;
        let scenario: Scenario = serde_json::from_str(raw).unwrap();
        assert_eq!(scenario.resolve().frame.as_deref(), Some("table"));
    }
}

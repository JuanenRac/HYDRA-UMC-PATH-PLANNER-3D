// HYDRA-UMC-PATH-PLANNER-3D - entry point
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
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
mod server;
mod service_metrics;
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
use validate::{check_endpoints, check_frame, validate_path, PathSafetyIssue};

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
    /// Decimals of the coordinates in the result (0 to 12). Absent: the coordinates are written as computed. The unit of every coordinate is the metre.
    #[serde(default)]
    output_decimals: Option<u8>,
}

/// The highest number of decimals `output_decimals` accepts.
const MAX_OUTPUT_DECIMALS: u8 = 12;

fn round_path(path: Vec<Vec3>, decimals: Option<u8>) -> Vec<Vec3> {
    let Some(decimals) = decimals else {
        return path;
    };
    let scale = 10f64.powi(i32::from(decimals));
    let round = |v: f64| (v * scale).round() / scale;
    path.into_iter()
        .map(|p| Vec3 {
            x: round(p.x),
            y: round(p.y),
            z: round(p.z),
        })
        .collect()
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

/// A path file: the bare array of waypoints, or an object with a `path` and
/// the `frame` they are in (the output of the planner is one, so it can be
/// handed back as it is; its other fields are ignored).
#[derive(Deserialize)]
#[serde(untagged)]
enum PathFile {
    Bare(Vec<Vec3>),
    Framed {
        path: Vec<Vec3>,
        #[serde(default)]
        frame: Option<String>,
    },
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
    let (path, path_frame): (Vec<Vec3>, Option<String>) = match serde_json::from_str(&path_raw) {
        Ok(PathFile::Bare(path)) => (path, None),
        Ok(PathFile::Framed { path, frame }) => (path, frame),
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
    let mut issues = validate_path(
        &path,
        &scenario.obstacles,
        &scenario.workspace,
        robot_radius,
    );
    issues.extend(check_endpoints(&path, scenario.start, scenario.goal));
    let frame_issues = check_frame(path_frame.as_deref(), frame.as_deref());
    if !frame_issues.is_empty() {
        eprintln!(
            "[path-planner-3d] the path is in frame {:?} but the scenario is in {:?}",
            path_frame, frame
        );
    }
    issues.extend(frame_issues);
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
    // The identity lines go to stderr: stdout carries the JSON result only,
    // so `planner scenario.json | jq` and a caller that reads the output
    // need not strip anything.
    eprintln!("HYDRA-UMC-PATH-PLANNER-3D v{VERSION}");
    eprintln!("Path planner: computes collision-free RRT paths for one agent through a static obstacle set.");

    let mut args: Vec<String> = env::args().collect();
    // Off unless asked for: one exit status (1) for every failure, as before. With --exit-codes a caller can tell the causes apart.
    let distinct_codes = args.iter().skip(1).any(|a| a == "--exit-codes");
    args.retain(|a| a != "--exit-codes");

    // serve is the only way the program listens on a network, and it is never started without being asked for.
    if args.get(1).map(String::as_str) == Some("serve") {
        return server::serve_command(&args[2..]);
    }

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
        eprintln!("Usage: hydra-umc-path-planner-3d <scenario.json>   (\"-\" reads the scenario from standard input; add --exit-codes for 1 no path, 2 invalid input, 3 time limit)");
        eprintln!("       hydra-umc-path-planner-3d validate <scenario.json> <path.json>");
        eprintln!("       hydra-umc-path-planner-3d shrink <scenario.json> [--out <minimal.json>]");
        eprintln!("       hydra-umc-path-planner-3d serve [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous] [--config FILE]   (off unless asked for; GET /health, and GET /metrics only when the configuration opens it)");
        eprintln!("See scenarios/example.json for the expected format.");
        return ExitCode::SUCCESS; // printing identity and usage is a valid no-arg invocation, not a failure
    };

    // "-" reads the scenario from standard input, so a caller can plan without a file in between.
    let read = if scenario_path == "-" {
        let mut text = String::new();
        std::io::Read::read_to_string(&mut std::io::stdin(), &mut text).map(|_| text)
    } else {
        fs::read_to_string(scenario_path)
    };
    let raw = match read {
        Ok(raw) => raw,
        Err(e) => {
            eprintln!("[path-planner-3d] could not read {scenario_path}: {e}");
            return failure_code("invalid_input", distinct_codes);
        }
    };

    let scenario: Scenario = match serde_json::from_str(&raw) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[path-planner-3d] could not parse {scenario_path}: {e}");
            return failure_code("invalid_input", distinct_codes);
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
            return failure_code("invalid_input", distinct_codes);
        }
    };
    if scenario
        .output_decimals
        .is_some_and(|d| d > MAX_OUTPUT_DECIMALS)
    {
        let outcome = PlanOutcome::Error {
            reason: format!(
                "invalid_input: output_decimals must be between 0 and {MAX_OUTPUT_DECIMALS}"
            ),
        };
        if let Ok(json) = serde_json::to_string_pretty(&outcome) {
            println!("{json}");
        }
        return failure_code("invalid_input", distinct_codes);
    }
    let output_decimals = scenario.output_decimals;
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
        Ok(path) => PlanOutcome::Ok {
            path: round_path(path, output_decimals),
            frame,
        },
        Err(e) => PlanOutcome::Error {
            reason: plan_error_reason(e),
        },
    };

    let is_ok = matches!(outcome, PlanOutcome::Ok { .. });
    let failure_reason = match &outcome {
        PlanOutcome::Error { reason } => reason.clone(),
        PlanOutcome::Ok { .. } => String::new(),
    };
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
        failure_code(&failure_reason, distinct_codes)
    }
}

/// The exit status of a failed plan. Without `--exit-codes` it is always 1. With it: 1 no path was found, 2 the input was invalid
/// (an unreadable or malformed scenario, or a start or goal that cannot be used), 3 the time limit was hit.
fn failure_code(reason: &str, distinct: bool) -> ExitCode {
    if !distinct {
        return ExitCode::FAILURE;
    }
    match reason {
        "no_path_found" => ExitCode::from(1),
        "time_limit_exceeded" => ExitCode::from(3),
        _ => ExitCode::from(2),
    }
}

#[cfg(test)]
mod exit_code_tests {
    use super::*;

    #[test]
    fn without_the_switch_every_failure_is_one() {
        for reason in [
            "no_path_found",
            "time_limit_exceeded",
            "invalid_input",
            "start_inside_obstacle",
        ] {
            assert_eq!(
                format!("{:?}", failure_code(reason, false)),
                format!("{:?}", ExitCode::FAILURE)
            );
        }
    }

    #[test]
    fn with_the_switch_the_cause_is_told_apart() {
        assert_eq!(
            format!("{:?}", failure_code("no_path_found", true)),
            format!("{:?}", ExitCode::from(1))
        );
        assert_eq!(
            format!("{:?}", failure_code("time_limit_exceeded", true)),
            format!("{:?}", ExitCode::from(3))
        );
        assert_eq!(
            format!("{:?}", failure_code("goal_inside_obstacle", true)),
            format!("{:?}", ExitCode::from(2))
        );
        assert_eq!(
            format!("{:?}", failure_code("invalid_input", true)),
            format!("{:?}", ExitCode::from(2))
        );
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

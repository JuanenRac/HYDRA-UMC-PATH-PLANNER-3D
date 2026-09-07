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
mod obstacle;
mod rng;
mod rrt;
mod semantics;
mod validate;

#[cfg(test)]
mod corpus;

use geometry::Vec3;
use obstacle::Obstacle;
use rrt::{plan, PlanError, PlannerConfig, Workspace};
use serde::{Deserialize, Serialize};
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
}

#[derive(Serialize)]
#[serde(tag = "status")]
enum PlanOutcome {
    #[serde(rename = "ok")]
    Ok { path: Vec<Vec3> },
    #[serde(rename = "error")]
    Error { reason: String },
}

fn plan_error_reason(e: PlanError) -> String {
    match e {
        PlanError::StartInsideObstacle => "start_inside_obstacle".to_string(),
        PlanError::GoalInsideObstacle => "goal_inside_obstacle".to_string(),
        PlanError::StartOutsideWorkspace => "start_outside_workspace".to_string(),
        PlanError::GoalOutsideWorkspace => "goal_outside_workspace".to_string(),
        PlanError::NoPathFound => "no_path_found".to_string(),
        PlanError::TimeLimitExceeded => "time_limit_exceeded".to_string(),
        // PATH-01: carries the real, specific reason (which numeric
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
    Safe,
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

    let robot_radius = scenario.config.unwrap_or_default().robot_radius;
    let issues = validate_path(
        &path,
        &scenario.obstacles,
        &scenario.workspace,
        robot_radius,
    );
    let is_safe = issues.is_empty();
    let outcome = if is_safe {
        ValidateOutcome::Safe
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

    let Some(scenario_path) = args.get(1) else {
        eprintln!("Usage: hydra-umc-path-planner-3d <scenario.json>");
        eprintln!("       hydra-umc-path-planner-3d validate <scenario.json> <path.json>");
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
        Ok(path) => PlanOutcome::Ok { path },
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

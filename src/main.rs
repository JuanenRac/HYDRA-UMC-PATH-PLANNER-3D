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
// integrate against it - see this repo's mejoras_futuras.txt. Proving
// the planning algorithm itself correct, with a scriptable interface
// that's still genuinely usable today, came first.
//
// What this does NOT do yet, honestly: multi-robot synchronized
// planning (one agent per call, not the swarm-wide 32+ robot
// coordination the README describes), RRT* rewiring for path
// optimality, and octree-accelerated collision checks at scale - see
// mejoras_futuras.txt for why each is deferred rather than half-built.

mod geometry;
mod obstacle;
mod rng;
mod rrt;

use geometry::Vec3;
use obstacle::Obstacle;
use rrt::{plan, PlanError, PlannerConfig, Workspace};
use serde::{Deserialize, Serialize};
use std::env;
use std::fs;
use std::process::ExitCode;

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

fn plan_error_reason(e: PlanError) -> &'static str {
    match e {
        PlanError::StartInsideObstacle => "start_inside_obstacle",
        PlanError::GoalInsideObstacle => "goal_inside_obstacle",
        PlanError::StartOutsideWorkspace => "start_outside_workspace",
        PlanError::GoalOutsideWorkspace => "goal_outside_workspace",
        PlanError::NoPathFound => "no_path_found",
    }
}

fn main() -> ExitCode {
    println!("HYDRA-UMC-PATH-PLANNER-3D v{VERSION}");
    println!("Multi-robot 3D path optimizer: computes collision-free, RRT trajectories for the swarm sharing one workspace.");

    let args: Vec<String> = env::args().collect();
    let Some(scenario_path) = args.get(1) else {
        eprintln!("Usage: hydra-umc-path-planner-3d <scenario.json>");
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
            reason: plan_error_reason(e).to_string(),
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

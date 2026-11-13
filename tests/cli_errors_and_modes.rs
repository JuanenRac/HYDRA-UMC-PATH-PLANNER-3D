// HYDRA-UMC-PATH-PLANNER-3D - tests/cli_errors_and_modes.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// Characterization of the command line around the planner: usage and
// read/parse failures of every mode, the planning error labels, the frame
// label, determinism for one seed, and the shrink mode on a sound scenario.

use std::path::{Path, PathBuf};
use std::process::Command;

fn temp(name: &str, content: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("path-planner-modes-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
}

fn run(args: &[&Path]) -> (Option<i32>, String, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-path-planner-3d"))
        .args(args)
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

const OPEN: &str = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"seed":7}"#;

#[test]
fn bare_invocation_prints_usage_and_a_missing_file_is_reported() {
    // identity and usage are a valid no-argument invocation, not a failure
    let (code, stdout, stderr) = run(&[]);
    assert_eq!(code, Some(0));
    assert!(stdout.is_empty(), "{stdout}");
    assert!(stderr.contains("HYDRA-UMC-PATH-PLANNER-3D v"), "{stderr}");
    assert!(
        stderr.contains("Usage: hydra-umc-path-planner-3d <scenario.json>"),
        "{stderr}"
    );

    let (code, stdout, stderr) = run(&[Path::new("definitely-missing-scenario.json")]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stderr.contains("could not read"), "{stderr}");
}

#[test]
fn a_scenario_that_is_not_json_is_reported_not_planned() {
    let bad = temp("bad.json", "{not json");
    let (code, stdout, stderr) = run(&[&bad]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stderr.contains("could not parse"), "{stderr}");
}

#[test]
fn the_planning_error_labels_are_stable() {
    let inside = temp(
        "start_inside.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":4,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
        "obstacles":[{"center":{"x":0,"y":0,"z":0},"radius":1.0}]}"#,
    );
    let (code, stdout, _) = run(&[&inside]);
    assert_eq!(code, Some(1), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "error");
    assert_eq!(parsed["reason"], "start_inside_obstacle");

    let outside = temp(
        "goal_outside.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":40,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[&outside]);
    assert_eq!(code, Some(1), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["reason"], "goal_outside_workspace");
}

#[test]
fn the_same_seed_gives_the_same_path_and_the_path_ends_at_the_goal() {
    let scenario = temp("open.json", OPEN);
    let (code_a, out_a, _) = run(&[&scenario]);
    let (code_b, out_b, _) = run(&[&scenario]);
    assert_eq!((code_a, code_b), (Some(0), Some(0)));
    assert_eq!(out_a, out_b, "a fixed seed must reproduce the path");
    let parsed: serde_json::Value = serde_json::from_str(&out_a).unwrap();
    assert_eq!(parsed["status"], "ok");
    let path = parsed["path"].as_array().unwrap();
    assert_eq!(path.first().unwrap()["x"], 0.0);
    assert_eq!(path.last().unwrap()["x"], 2.0);
}

#[test]
fn a_declared_frame_is_carried_into_the_result_and_bad_frames_are_refused() {
    let named = temp(
        "frame.json",
        r#"{"frame":"world","start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[&named]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["frame"], "world");

    let empty = temp(
        "frame_empty.json",
        r#"{"frame":"  ","start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    // a refused frame is answered as a JSON error on stdout
    let (code, stdout, _) = run(&[&empty]);
    assert_eq!(code, Some(1));
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "error");
    assert!(
        parsed["reason"]
            .as_str()
            .unwrap()
            .contains("frame must not be empty"),
        "{stdout}"
    );

    let long_frame = "f".repeat(65);
    let long = temp(
        "frame_long.json",
        &format!(
            r#"{{"frame":"{long_frame}","start":{{"x":0,"y":0,"z":0}},"goal":{{"x":2,"y":0,"z":0}},
            "workspace":{{"min":{{"x":-5,"y":-5,"z":-5}},"max":{{"x":5,"y":5,"z":5}}}}}}"#
        ),
    );
    let (code, stdout, _) = run(&[&long]);
    assert_eq!(code, Some(1));
    assert!(stdout.contains("at most 64 bytes"), "{stdout}");
}

#[test]
fn validate_usage_and_read_failures() {
    let (code, _, stderr) = run(&[Path::new("validate")]);
    assert_eq!(code, Some(1));
    assert!(
        stderr.contains("Usage: hydra-umc-path-planner-3d validate"),
        "{stderr}"
    );

    let scenario = temp("v_open.json", OPEN);
    let missing = Path::new("no-such-path.json");
    let (code, _, stderr) = run(&[Path::new("validate"), &scenario, missing]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not read"), "{stderr}");

    let junk = temp("v_junk.json", "[1,2");
    let (code, _, stderr) = run(&[Path::new("validate"), &scenario, &junk]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not parse"), "{stderr}");

    let (code, _, stderr) = run(&[Path::new("validate"), missing, &junk]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not read"), "{stderr}");
}

#[test]
fn validate_reports_a_path_through_an_obstacle_as_unsafe() {
    let scenario = temp(
        "v_obstacle.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":4,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
        "obstacles":[{"center":{"x":2,"y":0,"z":0},"radius":0.5}]}"#,
    );
    let straight = temp(
        "v_straight.json",
        r#"[{"x":0,"y":0,"z":0},{"x":4,"y":0,"z":0}]"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &straight]);
    assert_eq!(code, Some(1), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "unsafe");
    assert!(!parsed["issues"].as_array().unwrap().is_empty());
}

#[test]
fn validate_carries_the_frame_of_a_safe_result() {
    let scenario = temp(
        "v_frame.json",
        r#"{"frame":"base","start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let good = temp(
        "v_frame_path.json",
        r#"[{"x":0,"y":0,"z":0},{"x":2,"y":0,"z":0}]"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &good]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "safe");
    assert_eq!(parsed["frame"], "base");
}

#[test]
fn shrink_usage_errors_and_a_sound_scenario_has_nothing_to_shrink() {
    let (code, _, stderr) = run(&[Path::new("shrink")]);
    assert_eq!(code, Some(1));
    assert!(
        stderr.contains("Usage: hydra-umc-path-planner-3d shrink"),
        "{stderr}"
    );

    let scenario = temp("s_open.json", OPEN);
    let (code, _, stderr) = run(&[Path::new("shrink"), &scenario, Path::new("--out")]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("--out requires a file path"), "{stderr}");

    let (code, _, stderr) = run(&[Path::new("shrink"), Path::new("no-such-scenario.json")]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not read"), "{stderr}");

    let bad = temp("s_bad.json", "nope");
    let (code, _, stderr) = run(&[Path::new("shrink"), &bad]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not parse"), "{stderr}");

    let (code, stdout, _) = run(&[Path::new("shrink"), &scenario]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "no_failure");

    let out = std::env::temp_dir().join(format!(
        "path-planner-shrink-out-{}.json",
        std::process::id()
    ));
    let (code, stdout, _) = run(&[Path::new("shrink"), &scenario, Path::new("--out"), &out]);
    assert_eq!(code, Some(0), "{stdout}");
    assert!(stdout.contains("wrote"), "{stdout}");
    let written: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&out).unwrap()).unwrap();
    assert_eq!(written["status"], "no_failure");
    let _ = std::fs::remove_file(out);
}

#[test]
fn validate_accepts_a_path_file_with_a_frame_and_refuses_another_frame() {
    let scenario = temp(
        "vf_scenario.json",
        r#"{"frame":"base","start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    // the same frame, as the planner prints it (with a status the validator ignores)
    let same = temp(
        "vf_same.json",
        r#"{"status":"ok","frame":"base","path":[{"x":0,"y":0,"z":0},{"x":2,"y":0,"z":0}]}"#,
    );
    let (code, stdout, stderr) = run(&[Path::new("validate"), &scenario, &same]);
    assert_eq!(code, Some(0), "{stdout} {stderr}");

    // the path is in another frame: the numbers are not trusted
    let other = temp(
        "vf_other.json",
        r#"{"frame":"world","path":[{"x":0,"y":0,"z":0},{"x":2,"y":0,"z":0}]}"#,
    );
    let (code, stdout, stderr) = run(&[Path::new("validate"), &scenario, &other]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("PathFrameDoesNotMatch"), "{stdout}");
    assert!(
        stderr.contains("world") && stderr.contains("base"),
        "{stderr}"
    );

    // an object without a frame is read like the bare array
    let unframed = temp(
        "vf_unframed.json",
        r#"{"path":[{"x":0,"y":0,"z":0},{"x":2,"y":0,"z":0}]}"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &unframed]);
    assert_eq!(code, Some(0), "{stdout}");

    // a path that names a frame against a scenario that names none
    let bare_scenario = temp(
        "vf_bare_scenario.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &bare_scenario, &same]);
    assert_eq!(code, Some(1), "{stdout}");
}

#[test]
fn the_exit_codes_tell_the_causes_apart_only_when_asked_and_a_scenario_can_come_from_standard_input(
) {
    use std::io::Write;
    use std::process::Stdio;
    let wall = temp(
        "wall.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
  "obstacles":[{"center":{"x":2,"y":0,"z":0},"radius":0.5}],
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"seed":7}"#,
    );
    let (code, _, _) = run(&[&wall]);
    assert_eq!(code, Some(1)); // as before: one status for every failure
    let (code, stdout, _) = run(&[&wall, Path::new("--exit-codes")]);
    assert_eq!(code, Some(2), "{stdout}"); // the goal is inside an obstacle: invalid input
    let bad = temp("bad-codes.json", "{not json");
    let (code, _, _) = run(&[Path::new("--exit-codes"), &bad]);
    assert_eq!(code, Some(2));
    let (code, _, _) = run(&[
        Path::new("--exit-codes"),
        Path::new("definitely-missing-scenario.json"),
    ]);
    assert_eq!(code, Some(2));

    let mut child = Command::new(env!("CARGO_BIN_EXE_hydra-umc-path-planner-3d"))
        .arg("-")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(OPEN.as_bytes())
        .unwrap();
    let output = child.wait_with_output().unwrap();
    assert_eq!(output.status.code(), Some(0));
    assert!(
        String::from_utf8_lossy(&output.stdout).contains("\"status\": \"ok\""),
        "{}",
        String::from_utf8_lossy(&output.stdout)
    );
}

#[test]
fn output_decimals_rounds_the_coordinates_and_a_bad_value_is_refused() {
    let rounded = temp(
        "decimals.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2.123456789,"y":0,"z":0},
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"seed":7,"output_decimals":3}"#,
    );
    let (code, stdout, _) = run(&[&rounded]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    for point in parsed["path"].as_array().unwrap() {
        for axis in ["x", "y", "z"] {
            let v = point[axis].as_f64().unwrap();
            assert!(((v * 1000.0).round() - v * 1000.0).abs() < 1e-6, "{v}");
        }
    }
    let last = parsed["path"].as_array().unwrap().last().unwrap();
    assert_eq!(last["x"].as_f64().unwrap(), 2.123);

    let bad = temp(
        "decimals_bad.json",
        r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"output_decimals":40}"#,
    );
    let (code, stdout, _) = run(&[&bad]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("output_decimals"), "{stdout}");
}

const WALL_WITH_A_GAP: &str = r#"{"start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},
  "boxes":[{"min":{"x":-0.2,"y":-5,"z":-5},"max":{"x":0.2,"y":1,"z":5}}],
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"seed":3}"#;

#[test]
fn a_box_obstacle_is_avoided_by_the_plan_and_caught_by_the_validator() {
    let scenario = temp("wall.json", WALL_WITH_A_GAP);
    let (code, stdout, _) = run(&[&scenario]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let path: Vec<[f64; 3]> = parsed["path"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| {
            [
                p["x"].as_f64().unwrap(),
                p["y"].as_f64().unwrap(),
                p["z"].as_f64().unwrap(),
            ]
        })
        .collect();
    // no point of the path, sampled along every segment, is inside the wall
    for pair in path.windows(2) {
        for i in 0..=200 {
            let t = f64::from(i) / 200.0;
            let p = [0, 1, 2].map(|k| pair[0][k] + (pair[1][k] - pair[0][k]) * t);
            let inside = p[0] >= -0.2 && p[0] <= 0.2 && p[1] <= 1.0 && p[1] >= -5.0;
            assert!(!inside, "{p:?} is inside the box");
        }
    }
    // the same path validates as safe against the scenario that made it
    let planned = temp("wall-path.json", &stdout);
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &planned]);
    assert_eq!(code, Some(0), "{stdout}");
    // a straight line through the wall does not
    let straight = temp(
        "wall-straight.json",
        r#"[{"x":-3,"y":0,"z":0},{"x":3,"y":0,"z":0}]"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &straight]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("SegmentIntersectsObstacle"), "{stdout}");
}

#[test]
fn a_goal_inside_a_box_a_wrong_box_and_the_shrink_of_a_box_scenario_are_refused() {
    let inside = temp(
        "box-goal.json",
        r#"{"start":{"x":-3,"y":0,"z":0},"goal":{"x":0,"y":0,"z":0},
  "boxes":[{"min":{"x":-1,"y":-1,"z":-1},"max":{"x":1,"y":1,"z":1}}],
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[&inside]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("goal_inside_obstacle"), "{stdout}");

    let inverted = temp(
        "box-inverted.json",
        r#"{"start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},
  "boxes":[{"min":{"x":1,"y":0,"z":0},"max":{"x":-1,"y":1,"z":1}}],
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[&inverted]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("InvertedBox"), "{stdout}");

    let scenario = temp("wall-shrink.json", WALL_WITH_A_GAP);
    let (code, _, stderr) = run(&[Path::new("shrink"), &scenario]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("box obstacles"), "{stderr}");
}

fn scenario_with_zones(zones: &str, name: &str) -> PathBuf {
    let zones_path = temp(&format!("{name}-zones.json"), zones);
    let zones_path = zones_path.to_string_lossy().replace('\\', "/");
    temp(
        &format!("{name}-scenario.json"),
        &format!(
            r#"{{"start":{{"x":-3,"y":0,"z":0}},"goal":{{"x":3,"y":0,"z":0}},"zones_file":"{zones_path}",
  "workspace":{{"min":{{"x":-5,"y":-5,"z":-5}},"max":{{"x":5,"y":5,"z":5}}}},"seed":3}}"#
        ),
    )
}

#[test]
fn the_danger_zones_of_a_zones_file_are_avoided_and_the_warning_zones_are_not_used() {
    let scenario = scenario_with_zones(
        r#"{"zones":[
          {"id":"wall","level":"danger","min":{"x":-0.2,"y":-5,"z":-5},"max":{"x":0.2,"y":1,"z":5}},
          {"id":"everything","level":"warning","min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}]}"#,
        "danger",
    );
    let (code, stdout, _) = run(&[&scenario]);
    assert_eq!(code, Some(0), "{stdout}");
    let planned = temp("danger-path.json", &stdout);
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &planned]);
    assert_eq!(code, Some(0), "{stdout}");
    let straight = temp(
        "danger-straight.json",
        r#"[{"x":-3,"y":0,"z":0},{"x":3,"y":0,"z":0}]"#,
    );
    let (code, stdout, _) = run(&[Path::new("validate"), &scenario, &straight]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("SegmentIntersectsObstacle"), "{stdout}");
}

#[test]
fn a_zones_file_that_cannot_be_read_or_holds_a_danger_zone_that_is_not_a_box_is_refused() {
    let capsule = scenario_with_zones(
        r#"{"zones":[{"id":"arm","level":"danger","shape":"capsule"}]}"#,
        "capsule",
    );
    let (code, stdout, _) = run(&[&capsule]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(
        stdout.contains("invalid_input") && stdout.contains("arm"),
        "{stdout}"
    );

    let missing = temp(
        "zones-missing.json",
        r#"{"start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},"zones_file":"definitely-missing-zones.json",
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#,
    );
    let (code, stdout, _) = run(&[&missing]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(stdout.contains("could not read the zones file"), "{stdout}");
}

#[test]
fn the_fleet_command_plans_the_robots_in_order_and_reports_each() {
    let fleet = temp(
        "fleet.json",
        r#"{"workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
  "robots":[
    {"id":"a","start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},"seed":1},
    {"id":"b","start":{"x":0,"y":-3,"z":0},"goal":{"x":0,"y":3,"z":0},"seed":2}]}"#,
    );
    let (code, stdout, _) = run(&[Path::new("fleet"), &fleet]);
    assert_eq!(code, Some(0), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "ok");
    let robots = parsed["robots"].as_array().unwrap();
    assert_eq!(robots.len(), 2);
    assert_eq!(robots[0]["id"], "a");
    assert_eq!(robots[1]["id"], "b");
    assert!(robots
        .iter()
        .all(|r| r["path"].as_array().is_some_and(|p| p.len() >= 2)));

    let blocked = temp(
        "fleet-blocked.json",
        r#"{"workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
  "robots":[
    {"id":"a","start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},"seed":1},
    {"id":"b","start":{"x":0,"y":0,"z":0},"goal":{"x":0,"y":3,"z":0},"seed":2}]}"#,
    );
    let (code, stdout, _) = run(&[Path::new("fleet"), &blocked]);
    assert_eq!(code, Some(1), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "partial");
    assert_eq!(parsed["robots"][1]["reason"], "start_inside_obstacle");

    let (code, _, stderr) = run(&[
        Path::new("fleet"),
        Path::new("definitely-missing-fleet.json"),
    ]);
    assert_eq!(code, Some(1));
    assert!(stderr.contains("could not read"), "{stderr}");
}

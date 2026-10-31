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

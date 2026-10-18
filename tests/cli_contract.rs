// HYDRA-UMC-PATH-PLANNER-3D - tests/cli_contract.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// What a caller of the binary sees: stdout is the JSON result and nothing
// else, a partial config uses defaults, a path that does not begin at the
// start and end at the goal is not "safe".

use std::path::PathBuf;
use std::process::Command;

fn temp(name: &str, content: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("path-planner-cli-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
}

fn run(args: &[&std::path::Path]) -> (Option<i32>, String, String) {
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

const SCENARIO: &str = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
  "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
  "obstacles":[{"center":{"x":1,"y":3,"z":0},"radius":0.5}],
  "config":{"robot_radius":0.3},"seed":3}"#;

#[test]
fn stdout_is_only_the_json_result_and_a_partial_config_uses_defaults() {
    let scenario = temp("scenario.json", SCENARIO);
    let (code, stdout, stderr) = run(&[&scenario]);
    assert_eq!(code, Some(0), "{stdout} {stderr}");
    let parsed: serde_json::Value =
        serde_json::from_str(&stdout).expect("stdout must be pure JSON");
    assert_eq!(parsed["status"], "ok");
    assert!(
        stderr.contains("HYDRA-UMC-PATH-PLANNER-3D v"),
        "the banner goes to stderr: {stderr}"
    );
}

#[test]
fn a_path_must_begin_at_the_start_and_end_at_the_goal() {
    let scenario = temp("scenario2.json", SCENARIO);
    // clear of the obstacle, but it neither starts at (0,0,0) nor ends at the goal
    let elsewhere = temp(
        "elsewhere.json",
        r#"[{"x":-3,"y":-3,"z":0},{"x":-3,"y":-2,"z":0}]"#,
    );
    let (code, stdout, _) = run(&[std::path::Path::new("validate"), &scenario, &elsewhere]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(
        stdout.contains("PathDoesNotStartAtStart") && stdout.contains("PathDoesNotReachGoal"),
        "{stdout}"
    );

    let single = temp("single.json", r#"[{"x":0,"y":0,"z":0}]"#);
    let (code, stdout, _) = run(&[std::path::Path::new("validate"), &scenario, &single]);
    assert_eq!(
        code,
        Some(1),
        "a single waypoint short of the goal is not safe: {stdout}"
    );

    let good = temp("good.json", r#"[{"x":0,"y":0,"z":0},{"x":2,"y":0,"z":0}]"#);
    let (code, stdout, _) = run(&[std::path::Path::new("validate"), &scenario, &good]);
    assert_eq!(code, Some(0), "{stdout}");
}

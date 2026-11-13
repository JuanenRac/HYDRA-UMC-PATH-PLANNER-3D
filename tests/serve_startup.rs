// HYDRA-UMC-PATH-PLANNER-3D - tests/serve_startup.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// What an operator sees when `serve` cannot start: a nonzero status and a
// line that says why, never a listener that is not what was asked for.

use std::process::{Command, Stdio};

fn serve(args: &[&str]) -> (Option<i32>, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-path-planner-3d"))
        .arg("serve")
        .args(args)
        .env_remove("PATH_PLANNER_TOKEN")
        .env_remove("PATH_PLANNER_CONFIG")
        .stdin(Stdio::null())
        .output()
        .expect("the binary must run");
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stderr).to_string(),
    )
}

fn temp_file(name: &str, content: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("path-planner-serve-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
}

#[test]
fn an_unknown_option_or_a_missing_value_stops_the_start_with_the_usage() {
    for args in [
        vec!["--nope"],
        vec!["--addr"],
        vec!["--token"],
        vec!["--config"],
    ] {
        let (code, stderr) = serve(&args);
        assert_eq!(code, Some(1), "{args:?}: {stderr}");
        assert!(stderr.contains("serve:"), "{args:?}: {stderr}");
        assert!(stderr.contains("Usage: serve"), "{args:?}: {stderr}");
    }
}

#[test]
fn an_address_off_this_machine_is_refused_without_a_token() {
    let (code, stderr) = serve(&["--addr", "0.0.0.0:0"]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(stderr.contains("refusing to listen"), "{stderr}");
    assert!(
        !stderr.contains("listening on"),
        "it must not announce a listener it will not keep: {stderr}"
    );
}

#[test]
fn a_configuration_that_cannot_be_read_or_has_an_unknown_key_stops_the_start() {
    let (code, stderr) = serve(&["--config", "no-such-configuration.json"]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(stderr.contains("could not read"), "{stderr}");

    let unknown = temp_file("unknown.json", r#"{"metrics":{"enabled":true,"x":1}}"#);
    let (code, stderr) = serve(&["--config", unknown.to_str().unwrap()]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(stderr.contains("unknown field"), "{stderr}");

    let junk = temp_file("junk.json", "not json at all");
    let (code, stderr) = serve(&["--config", junk.to_str().unwrap()]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(stderr.contains("configuration"), "{stderr}");
}

#[test]
fn an_address_that_cannot_be_bound_stops_the_start() {
    let (code, stderr) = serve(&["--addr", "localhost:99999"]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(stderr.contains("could not listen on"), "{stderr}");
}

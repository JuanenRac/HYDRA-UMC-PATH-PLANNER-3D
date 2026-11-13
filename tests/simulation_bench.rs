// HYDRA-UMC-PATH-PLANNER-3D - tests/simulation_bench.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// The simulation bench: the real binary planning through synthetic worlds
// (spheres drawn from a fixed seed), every path checked again by the
// independent validator of the same binary, with injected faults (a
// waypoint pushed into an obstacle, a goal walled in, a corridor with one
// gap). Nothing here touches a robot: a pass means "passed in simulation".

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

fn temp(content: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("path-planner-bench-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!(
        "file-{}.json",
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::write(&path, content).unwrap();
    path
}

fn run(args: &[&Path]) -> (Option<i32>, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_hydra-umc-path-planner-3d"))
        .args(args)
        .output()
        .unwrap();
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).to_string(),
    )
}

/// A small generator with a fixed seed: the same world every run, on every machine.
struct Lcg(u64);

impl Lcg {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

#[derive(Clone)]
struct World {
    start: [f64; 3],
    goal: [f64; 3],
    obstacles: Vec<([f64; 3], f64)>,
    radius: f64,
    max_iterations: u32,
    seed: u64,
}

impl World {
    fn scenario(&self) -> String {
        let obstacles: Vec<String> = self
            .obstacles
            .iter()
            .map(|(c, r)| {
                format!(
                    r#"{{"center":{{"x":{},"y":{},"z":{}}},"radius":{}}}"#,
                    c[0], c[1], c[2], r
                )
            })
            .collect();
        format!(
            r#"{{"start":{{"x":{},"y":{},"z":{}}},"goal":{{"x":{},"y":{},"z":{}}},
"workspace":{{"min":{{"x":-10,"y":-10,"z":-10}},"max":{{"x":10,"y":10,"z":10}}}},
"obstacles":[{}],
"config":{{"robot_radius":{},"max_iterations":{},"step_size":0.6}},"seed":{}}}"#,
            self.start[0],
            self.start[1],
            self.start[2],
            self.goal[0],
            self.goal[1],
            self.goal[2],
            obstacles.join(","),
            self.radius,
            self.max_iterations,
            self.seed
        )
    }
}

fn clear_of(p: [f64; 3], obstacles: &[([f64; 3], f64)], margin: f64) -> bool {
    obstacles.iter().all(|(c, r)| {
        let d = ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt();
        d > r + margin
    })
}

/// A sparse random world: a handful of spheres, a start and a goal both clear of them.
fn random_world(index: u64) -> World {
    let mut rng = Lcg(0x9E37_79B9_7F4A_7C15 ^ (index + 1).wrapping_mul(0x1234_5678_9ABC_DEF1));
    let count = 4 + (rng.unit() * 8.0) as usize;
    let mut obstacles = Vec::new();
    for _ in 0..count {
        let c = [
            rng.range(-7.0, 7.0),
            rng.range(-7.0, 7.0),
            rng.range(-7.0, 7.0),
        ];
        obstacles.push((c, rng.range(0.5, 1.4)));
    }
    let pick = |rng: &mut Lcg, obstacles: &[([f64; 3], f64)]| loop {
        let p = [
            rng.range(-8.0, 8.0),
            rng.range(-8.0, 8.0),
            rng.range(-8.0, 8.0),
        ];
        if clear_of(p, obstacles, 1.0) {
            return p;
        }
    };
    let start = pick(&mut rng, &obstacles);
    let goal = pick(&mut rng, &obstacles);
    World {
        start,
        goal,
        obstacles,
        radius: 0.2,
        max_iterations: 20000,
        seed: index + 11,
    }
}

fn path_of(stdout: &str) -> Vec<[f64; 3]> {
    let parsed: serde_json::Value = serde_json::from_str(stdout).expect("stdout is pure JSON");
    parsed["path"]
        .as_array()
        .expect("an ok result carries a path")
        .iter()
        .map(|p| {
            [
                p["x"].as_f64().unwrap(),
                p["y"].as_f64().unwrap(),
                p["z"].as_f64().unwrap(),
            ]
        })
        .collect()
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (a[0] - b[0]).abs() < 1e-6 && (a[1] - b[1]).abs() < 1e-6 && (a[2] - b[2]).abs() < 1e-6
}

fn path_json(path: &[[f64; 3]]) -> String {
    let points: Vec<String> = path
        .iter()
        .map(|p| format!(r#"{{"x":{},"y":{},"z":{}}}"#, p[0], p[1], p[2]))
        .collect();
    format!("[{}]", points.join(","))
}

#[test]
fn random_worlds_are_solved_and_every_path_passes_the_independent_validator() {
    let mut solved = 0;
    let total = 30;
    for index in 0..total {
        let world = random_world(index);
        let scenario = temp(&world.scenario());
        let (code, stdout) = run(&[&scenario]);
        let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
        if code == Some(0) {
            solved += 1;
            let path = path_of(&stdout);
            assert!(
                near(path[0], world.start),
                "world {index}: begins at the start"
            );
            assert!(
                near(path[path.len() - 1], world.goal),
                "world {index}: ends at the goal"
            );
            let path_file = temp(&path_json(&path));
            let (vcode, vout) = run(&[Path::new("validate"), &scenario, &path_file]);
            assert_eq!(vcode, Some(0), "world {index} must validate: {vout}");
        } else {
            let reason = parsed["reason"].as_str().unwrap_or_default();
            assert!(
                reason == "no_path_found" || reason == "time_limit_exceeded",
                "world {index}: an unsolved world says why, not a crash: {stdout}"
            );
        }
    }
    assert!(
        solved * 10 >= total * 8,
        "at least 80% of the sparse worlds are solved, got {solved} of {total}"
    );
}

#[test]
fn the_same_world_and_seed_give_the_same_answer_byte_for_byte() {
    for index in [3u64, 8, 21] {
        let world = random_world(index);
        let scenario = temp(&world.scenario());
        let (_, first) = run(&[&scenario]);
        let (_, second) = run(&[&scenario]);
        assert_eq!(first, second, "world {index}");
    }
}

#[test]
fn rrt_star_in_a_random_world_is_valid_and_not_longer_than_the_plain_search() {
    fn length(path: &[[f64; 3]]) -> f64 {
        path.windows(2)
            .map(|w| {
                ((w[1][0] - w[0][0]).powi(2)
                    + (w[1][1] - w[0][1]).powi(2)
                    + (w[1][2] - w[0][2]).powi(2))
                .sqrt()
            })
            .sum()
    }
    let mut checked = 0;
    for index in 0..8u64 {
        let world = random_world(index);
        let plain = temp(&world.scenario());
        let (code, stdout) = run(&[&plain]);
        if code != Some(0) {
            continue;
        }
        let plain_path = path_of(&stdout);
        let starred_text = world.scenario().replace(
            r#""step_size":0.6"#,
            r#""step_size":0.6,"optimize_iterations":1500,"rewire_radius":2.0"#,
        );
        let starred = temp(&starred_text);
        let (scode, sout) = run(&[&starred]);
        assert_eq!(scode, Some(0), "world {index}: {sout}");
        let star_path = path_of(&sout);
        let path_file = temp(&path_json(&star_path));
        let (vcode, vout) = run(&[Path::new("validate"), &starred, &path_file]);
        assert_eq!(vcode, Some(0), "world {index}: {vout}");
        assert!(
            length(&star_path) <= length(&plain_path) + 1e-6,
            "world {index}: RRT* is never longer than the plain search of the same seed"
        );
        checked += 1;
    }
    assert!(checked >= 4, "enough worlds were compared: {checked}");
}

#[test]
fn a_waypoint_pushed_into_an_obstacle_is_caught_by_the_validator() {
    let mut caught = 0;
    for index in 0..10u64 {
        let world = random_world(index);
        let scenario = temp(&world.scenario());
        let (code, stdout) = run(&[&scenario]);
        if code != Some(0) {
            continue;
        }
        let mut path = path_of(&stdout);
        let centre = world.obstacles[0].0;
        path.insert(1, centre);
        let tampered = temp(&path_json(&path));
        let (vcode, vout) = run(&[Path::new("validate"), &scenario, &tampered]);
        assert_eq!(vcode, Some(1), "world {index}: {vout}");
        assert!(
            vout.contains("WaypointInsideObstacle"),
            "world {index}: {vout}"
        );
        caught += 1;
    }
    assert!(caught >= 5, "enough tampered paths were checked: {caught}");
}

#[test]
fn a_waypoint_pushed_out_of_the_workspace_is_caught_by_the_validator() {
    let (scenario, mut path) = (0..10u64)
        .find_map(|index| {
            let scenario = temp(&random_world(index).scenario());
            let (code, stdout) = run(&[&scenario]);
            (code == Some(0)).then(|| (scenario, path_of(&stdout)))
        })
        .expect("one of ten sparse worlds is solved");
    path.insert(1, [50.0, 0.0, 0.0]);
    let tampered = temp(&path_json(&path));
    let (vcode, vout) = run(&[Path::new("validate"), &scenario, &tampered]);
    assert_eq!(vcode, Some(1), "{vout}");
    assert!(vout.contains("WaypointOutsideWorkspace"), "{vout}");
}

/// The goal sits in the middle of a closed shell of overlapping spheres: no path exists.
fn walled_in_world() -> World {
    let mut obstacles = Vec::new();
    for ix in [-2.0, 0.0, 2.0] {
        for iy in [-2.0, 0.0, 2.0] {
            for iz in [-2.0, 0.0, 2.0] {
                if ix != 0.0 || iy != 0.0 || iz != 0.0 {
                    obstacles.push(([ix, iy, iz], 1.4));
                }
            }
        }
    }
    World {
        start: [-8.0, -8.0, -8.0],
        goal: [0.0, 0.0, 0.0],
        obstacles,
        radius: 0.2,
        max_iterations: 3000,
        seed: 5,
    }
}

#[test]
fn a_goal_walled_in_by_spheres_is_reported_as_no_path_and_never_as_a_path() {
    let scenario = temp(&walled_in_world().scenario());
    let (code, stdout) = run(&[&scenario]);
    assert_eq!(code, Some(1), "{stdout}");
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed["status"], "error");
    assert_eq!(parsed["reason"], "no_path_found");
}

#[test]
fn a_straight_line_through_the_walled_in_world_is_refused_by_the_validator() {
    let scenario = temp(&walled_in_world().scenario());
    let straight = temp(&path_json(&[[-8.0, -8.0, -8.0], [0.0, 0.0, 0.0]]));
    let (code, stdout) = run(&[Path::new("validate"), &scenario, &straight]);
    assert_eq!(code, Some(1), "{stdout}");
    assert!(
        stdout.contains("SegmentIntersectsObstacle") || stdout.contains("WaypointInsideObstacle")
    );
}

#[test]
fn the_planner_never_fails_its_own_validator_on_random_worlds() {
    // `shrink` reports "no_failure" exactly when the planner's output passes the validator.
    for index in [1u64, 6, 12, 18] {
        let scenario = temp(&random_world(index).scenario());
        let (code, stdout) = run(&[Path::new("shrink"), &scenario]);
        assert_eq!(code, Some(0), "world {index}: {stdout}");
        let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
        assert_eq!(parsed["status"], "no_failure", "world {index}");
    }
}

#[test]
fn a_wall_with_one_gap_is_crossed_only_through_the_gap() {
    let mut obstacles = Vec::new();
    let mut y = -9.0;
    while y <= 9.0 {
        let mut z = -9.0;
        while z <= 9.0 {
            if (y * y + z * z).sqrt() > 2.5 {
                obstacles.push(([0.0, y, z], 1.0));
            }
            z += 1.5;
        }
        y += 1.5;
    }
    let world = World {
        start: [-6.0, 0.0, 0.0],
        goal: [6.0, 0.0, 0.0],
        obstacles,
        radius: 0.2,
        max_iterations: 20000,
        seed: 4,
    };
    let scenario = temp(&world.scenario());
    let (code, stdout) = run(&[&scenario]);
    assert_eq!(code, Some(0), "{stdout}");
    let path = path_of(&stdout);
    let mut crossings = 0;
    for pair in path.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if (a[0] < 0.0) != (b[0] < 0.0) {
            let t = (0.0 - a[0]) / (b[0] - a[0]);
            let y = a[1] + t * (b[1] - a[1]);
            let z = a[2] + t * (b[2] - a[2]);
            assert!(
                (y * y + z * z).sqrt() < 2.7,
                "the path crosses the wall at ({y}, {z}), outside the gap"
            );
            crossings += 1;
        }
    }
    assert!(crossings >= 1, "the path crosses the wall");
}

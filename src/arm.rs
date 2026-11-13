// HYDRA-UMC-PATH-PLANNER-3D - src/arm.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// Planning for an arm in the space of its joint angles instead of the position of a point. The arm is a serial chain of revolute joints described in
// the scenario the way a URDF describes it (the origin of each joint in the frame of the one before, its roll-pitch-yaw, its axis and its limits), and
// the position of every joint comes from forward kinematics with the same convention as `fk` of HYDRA-UMC-PHYSICS-REPLICA: the frame of a joint is
// the frame of its parent, moved by `origin`, turned by `rpy` (Rz * Ry * Rx) and then by the angle about `axis`. Each link is the segment between two
// consecutive joint positions (the last one ends at `tip`, an offset in the frame of the last joint), with the radius `link_radius`, and it is tested
// against the spheres and the boxes of the scenario. Nothing is computed by the physics replica here: the chain is in the scenario, so this runs
// without the other program (the numbers can be compared with its `fk --envelope`). Collisions of the arm with itself are not checked.

use crate::geometry::Vec3;
use crate::obstacle::{BoxObstacle, Obstacle};
use crate::rng::Xorshift64Star;
use serde::Deserialize;

type Matrix = [[f64; 3]; 3];

const IDENTITY: Matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn multiply(a: &Matrix, b: &Matrix) -> Matrix {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn apply(m: &Matrix, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

/// The rotation by `angle` about `axis` (Rodrigues); the axis is normalized first.
fn about_axis(axis: Vec3, angle: f64) -> Matrix {
    let a = axis.normalized();
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [
            t * a.x * a.x + c,
            t * a.x * a.y - s * a.z,
            t * a.x * a.z + s * a.y,
        ],
        [
            t * a.x * a.y + s * a.z,
            t * a.y * a.y + c,
            t * a.y * a.z - s * a.x,
        ],
        [
            t * a.x * a.z - s * a.y,
            t * a.y * a.z + s * a.x,
            t * a.z * a.z + c,
        ],
    ]
}

/// roll about x, pitch about y, yaw about z, applied as Rz * Ry * Rx (the convention of URDF).
fn from_rpy(rpy: Vec3) -> Matrix {
    let rx = about_axis(Vec3::new(1.0, 0.0, 0.0), rpy.x);
    let ry = about_axis(Vec3::new(0.0, 1.0, 0.0), rpy.y);
    let rz = about_axis(Vec3::new(0.0, 0.0, 1.0), rpy.z);
    multiply(&rz, &multiply(&ry, &rx))
}

#[derive(Debug, Clone, Deserialize)]
pub struct ArmJoint {
    /// only a label for the person who writes the scenario
    #[serde(default)]
    #[allow(dead_code)]
    pub name: String,
    /// where the joint is, in the frame of the joint before it (the base for the first one), in metres
    pub origin: Vec3,
    /// roll, pitch and yaw of the joint frame in radians; zero when absent
    #[serde(default)]
    pub rpy: Option<Vec3>,
    pub axis: Vec3,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Arm {
    /// where the base of the arm is in the world (the origin when absent)
    #[serde(default)]
    pub base: Option<Vec3>,
    pub joints: Vec<ArmJoint>,
    /// the end of the last link, an offset in the frame of the last joint
    pub tip: Vec3,
    /// the radius of every link, in metres
    #[serde(default = "default_link_radius")]
    pub link_radius: f64,
}

fn default_link_radius() -> f64 {
    0.05
}

/// How the search in joint space works; every number is in radians except the counts.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct ArmConfig {
    /// the longest step of the tree towards a sample
    pub step_rad: f64,
    /// the largest change of any joint between two configurations that are checked on a straight move
    pub resolution_rad: f64,
    /// how near the goal a node must be to try the last move
    pub goal_threshold_rad: f64,
    pub goal_bias: f64,
    pub max_iterations: u32,
}

impl Default for ArmConfig {
    fn default() -> Self {
        ArmConfig {
            step_rad: 0.3,
            resolution_rad: 0.05,
            goal_threshold_rad: 0.3,
            goal_bias: 0.1,
            max_iterations: 5000,
        }
    }
}

/// The most iterations accepted: the tree is searched node by node.
pub const MAX_ITERATIONS_LIMIT: u32 = 100_000;

#[derive(Debug, Deserialize)]
pub struct ArmScenario {
    pub arm: Arm,
    pub start: Vec<f64>,
    pub goal: Vec<f64>,
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
    #[serde(default)]
    pub boxes: Vec<BoxObstacle>,
    #[serde(default)]
    pub config: ArmConfig,
    #[serde(default)]
    pub seed: Option<u64>,
}

#[derive(Debug, PartialEq)]
pub enum ArmError {
    /// the scenario is not usable; the text says what to fix
    Invalid(String),
    StartInCollision,
    GoalInCollision,
    NoPathFound,
}

impl ArmError {
    pub fn reason(&self) -> String {
        match self {
            ArmError::Invalid(text) => format!("invalid_input: {text}"),
            ArmError::StartInCollision => "start_in_collision".to_string(),
            ArmError::GoalInCollision => "goal_in_collision".to_string(),
            ArmError::NoPathFound => "no_path_found".to_string(),
        }
    }
}

/// The positions that bound the links: the base, every joint and the tip (so `joints + 2` points) for the joint angles `q`.
pub fn link_points(arm: &Arm, q: &[f64]) -> Vec<Vec3> {
    let mut position = arm.base.unwrap_or(Vec3::new(0.0, 0.0, 0.0));
    let mut rotation = IDENTITY;
    let mut points = vec![position];
    for (joint, angle) in arm.joints.iter().zip(q) {
        position = position.add(apply(&rotation, joint.origin));
        rotation = multiply(
            &rotation,
            &from_rpy(joint.rpy.unwrap_or(Vec3::new(0.0, 0.0, 0.0))),
        );
        rotation = multiply(&rotation, &about_axis(joint.axis, *angle));
        points.push(position);
    }
    points.push(position.add(apply(&rotation, arm.tip)));
    points
}

/// True if any link, with its radius, touches a sphere or a box of the scenario in the configuration `q`.
pub fn in_collision(arm: &Arm, q: &[f64], obstacles: &[Obstacle], boxes: &[BoxObstacle]) -> bool {
    link_points(arm, q).windows(2).any(|link| {
        obstacles
            .iter()
            .any(|o| o.intersects_segment(link[0], link[1], arm.link_radius))
            || boxes
                .iter()
                .any(|b| b.intersects_segment(link[0], link[1], arm.link_radius))
    })
}

fn validate(scenario: &ArmScenario) -> Result<(), ArmError> {
    let invalid = |text: &str| Err(ArmError::Invalid(text.to_string()));
    let arm = &scenario.arm;
    if arm.joints.is_empty() {
        return invalid("the arm has no joints");
    }
    if !(arm.link_radius.is_finite() && arm.link_radius >= 0.0) {
        return invalid("link_radius must be a finite number, zero or more");
    }
    if !arm.tip.is_finite() || arm.base.is_some_and(|b| !b.is_finite()) {
        return invalid("the base and the tip must be finite points");
    }
    for (index, joint) in arm.joints.iter().enumerate() {
        if !joint.origin.is_finite() || joint.rpy.is_some_and(|r| !r.is_finite()) {
            return Err(ArmError::Invalid(format!(
                "joint {index}: origin and rpy must be finite"
            )));
        }
        if !joint.axis.is_finite() || joint.axis.length() < 1e-9 {
            return Err(ArmError::Invalid(format!(
                "joint {index}: the axis must be a finite vector that is not zero"
            )));
        }
        if !(joint.lower.is_finite() && joint.upper.is_finite() && joint.lower <= joint.upper) {
            return Err(ArmError::Invalid(format!(
                "joint {index}: the limits must be finite, with lower not above upper"
            )));
        }
    }
    let n = arm.joints.len();
    for (name, q) in [("start", &scenario.start), ("goal", &scenario.goal)] {
        if q.len() != n {
            return Err(ArmError::Invalid(format!(
                "{name} has {} angles but the arm has {n} joints",
                q.len()
            )));
        }
        for (index, (angle, joint)) in q.iter().zip(&arm.joints).enumerate() {
            if !angle.is_finite() || *angle < joint.lower || *angle > joint.upper {
                return Err(ArmError::Invalid(format!(
                    "{name}: angle {index} is outside the limits of its joint"
                )));
            }
        }
    }
    let c = &scenario.config;
    for (name, value) in [
        ("step_rad", c.step_rad),
        ("resolution_rad", c.resolution_rad),
        ("goal_threshold_rad", c.goal_threshold_rad),
    ] {
        if !(value.is_finite() && value > 0.0) {
            return Err(ArmError::Invalid(format!(
                "{name} must be a finite number above zero"
            )));
        }
    }
    if !(c.goal_bias.is_finite() && (0.0..=1.0).contains(&c.goal_bias)) {
        return invalid("goal_bias must be between 0 and 1");
    }
    if c.max_iterations == 0 || c.max_iterations > MAX_ITERATIONS_LIMIT {
        return Err(ArmError::Invalid(format!(
            "max_iterations must be between 1 and {MAX_ITERATIONS_LIMIT}"
        )));
    }
    crate::semantics::validate_obstacles(&scenario.obstacles)
        .map_err(|r| ArmError::Invalid(format!("{r:?}")))?;
    crate::semantics::validate_boxes(&scenario.boxes)
        .map_err(|r| ArmError::Invalid(format!("{r:?}")))?;
    Ok(())
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// True if every configuration on the straight move from `a` to `b` is free (`a` is taken as free, `b` is checked).
fn move_is_free(scenario: &ArmScenario, a: &[f64], b: &[f64]) -> bool {
    let widest = a
        .iter()
        .zip(b)
        .map(|(x, y)| (y - x).abs())
        .fold(0.0, f64::max);
    let steps = (widest / scenario.config.resolution_rad).ceil().max(1.0) as usize;
    (1..=steps).all(|i| {
        let t = i as f64 / steps as f64;
        let q: Vec<f64> = a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect();
        !in_collision(&scenario.arm, &q, &scenario.obstacles, &scenario.boxes)
    })
}

/// An RRT in the space of the joint angles. The same scenario and seed give the same path.
pub fn plan_arm(scenario: &ArmScenario) -> Result<Vec<Vec<f64>>, ArmError> {
    validate(scenario)?;
    let arm = &scenario.arm;
    if in_collision(arm, &scenario.start, &scenario.obstacles, &scenario.boxes) {
        return Err(ArmError::StartInCollision);
    }
    if in_collision(arm, &scenario.goal, &scenario.obstacles, &scenario.boxes) {
        return Err(ArmError::GoalInCollision);
    }
    if move_is_free(scenario, &scenario.start, &scenario.goal) {
        return Ok(vec![scenario.start.clone(), scenario.goal.clone()]);
    }
    let config = scenario.config;
    let mut rng = Xorshift64Star::new(scenario.seed.unwrap_or(1));
    let mut nodes: Vec<(Vec<f64>, Option<usize>)> = vec![(scenario.start.clone(), None)];
    for _ in 0..config.max_iterations {
        let sample: Vec<f64> = if rng.next_f64() < config.goal_bias {
            scenario.goal.clone()
        } else {
            arm.joints
                .iter()
                .map(|j| j.lower + (j.upper - j.lower) * rng.next_f64())
                .collect()
        };
        let nearest = (0..nodes.len())
            .min_by(|&i, &j| {
                distance(&nodes[i].0, &sample).total_cmp(&distance(&nodes[j].0, &sample))
            })
            .unwrap_or(0);
        let from = nodes[nearest].0.clone();
        let gap = distance(&from, &sample);
        let new: Vec<f64> = if gap <= config.step_rad {
            sample
        } else {
            from.iter()
                .zip(&sample)
                .map(|(x, y)| x + (y - x) * config.step_rad / gap)
                .collect()
        };
        if !move_is_free(scenario, &from, &new) {
            continue;
        }
        nodes.push((new.clone(), Some(nearest)));
        let last = nodes.len() - 1;
        if distance(&new, &scenario.goal) <= config.goal_threshold_rad
            && move_is_free(scenario, &new, &scenario.goal)
        {
            let mut path = vec![scenario.goal.clone()];
            let mut at = Some(last);
            while let Some(index) = at {
                path.push(nodes[index].0.clone());
                at = nodes[index].1;
            }
            path.reverse();
            return Ok(path);
        }
    }
    Err(ArmError::NoPathFound)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planar_arm() -> Arm {
        // two links of one metre in the plane z = 0, both joints turning about z
        Arm {
            base: None,
            joints: vec![
                ArmJoint {
                    name: "shoulder".into(),
                    origin: Vec3::new(0.0, 0.0, 0.0),
                    rpy: None,
                    axis: Vec3::new(0.0, 0.0, 1.0),
                    lower: -3.0,
                    upper: 3.0,
                },
                ArmJoint {
                    name: "elbow".into(),
                    origin: Vec3::new(1.0, 0.0, 0.0),
                    rpy: None,
                    axis: Vec3::new(0.0, 0.0, 1.0),
                    lower: -3.0,
                    upper: 3.0,
                },
            ],
            tip: Vec3::new(1.0, 0.0, 0.0),
            link_radius: 0.05,
        }
    }

    fn near(a: Vec3, x: f64, y: f64, z: f64) -> bool {
        a.distance(Vec3::new(x, y, z)) < 1e-9
    }

    #[test]
    fn the_forward_kinematics_of_a_planar_arm_are_the_ones_of_the_geometry() {
        let arm = planar_arm();
        let straight = link_points(&arm, &[0.0, 0.0]);
        assert_eq!(straight.len(), 4);
        assert!(
            near(straight[1], 0.0, 0.0, 0.0)
                && near(straight[2], 1.0, 0.0, 0.0)
                && near(straight[3], 2.0, 0.0, 0.0)
        );
        let up = link_points(&arm, &[std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(
            near(up[2], 0.0, 1.0, 0.0) && near(up[3], 0.0, 2.0, 0.0),
            "{up:?}"
        );
        let bent = link_points(&arm, &[0.0, std::f64::consts::FRAC_PI_2]);
        assert!(
            near(bent[2], 1.0, 0.0, 0.0) && near(bent[3], 1.0, 1.0, 0.0),
            "{bent:?}"
        );
    }

    #[test]
    fn the_rpy_of_a_joint_turns_the_frame_of_the_links_after_it() {
        let mut arm = planar_arm();
        // the elbow frame is turned a quarter turn about x before its own angle: its link points along +x still (x is the axis), but a link along y goes to z
        arm.joints[1].rpy = Some(Vec3::new(std::f64::consts::FRAC_PI_2, 0.0, 0.0));
        arm.joints[1].axis = Vec3::new(0.0, 1.0, 0.0);
        arm.tip = Vec3::new(0.0, 1.0, 0.0);
        let points = link_points(&arm, &[0.0, 0.0]);
        assert!(near(points[3], 1.0, 0.0, 1.0), "{points:?}");
    }

    #[test]
    fn a_link_is_tested_against_the_spheres_and_the_boxes() {
        let arm = planar_arm();
        let sphere = [Obstacle::new(Vec3::new(1.5, 0.0, 0.0), 0.2)];
        assert!(in_collision(&arm, &[0.0, 0.0], &sphere, &[]));
        assert!(!in_collision(
            &arm,
            &[std::f64::consts::FRAC_PI_2, 0.0],
            &sphere,
            &[]
        ));
        let wall = [BoxObstacle {
            min: Vec3::new(1.4, -1.0, -1.0),
            max: Vec3::new(1.6, 1.0, 1.0),
        }];
        assert!(in_collision(&arm, &[0.0, 0.0], &[], &wall));
        assert!(!in_collision(
            &arm,
            &[std::f64::consts::FRAC_PI_2, 0.0],
            &[],
            &wall
        ));
    }

    fn scenario(start: [f64; 2], goal: [f64; 2], obstacles: Vec<Obstacle>) -> ArmScenario {
        ArmScenario {
            arm: planar_arm(),
            start: start.to_vec(),
            goal: goal.to_vec(),
            obstacles,
            boxes: Vec::new(),
            config: ArmConfig::default(),
            seed: Some(5),
        }
    }

    #[test]
    fn a_free_move_is_one_straight_segment_of_the_joint_space() {
        let s = scenario([0.0, 0.0], [0.5, 0.3], Vec::new());
        let path = plan_arm(&s).unwrap();
        assert_eq!(path, vec![vec![0.0, 0.0], vec![0.5, 0.3]]);
    }

    #[test]
    fn the_arm_folds_to_get_past_an_obstacle_that_blocks_the_straight_move() {
        // swinging the straight arm from 0 to a quarter turn sweeps through (1, 1); the folded arm stays inside a circle of radius 1
        let obstacle = vec![Obstacle::new(Vec3::new(1.0, 1.0, 0.0), 0.25)];
        let s = scenario([0.0, 0.0], [std::f64::consts::FRAC_PI_2, 0.0], obstacle);
        assert!(
            !move_is_free(&s, &s.start, &s.goal),
            "the straight move must be blocked"
        );
        let path = plan_arm(&s).expect("a path exists by folding the arm");
        assert_eq!(path.first().unwrap(), &s.start);
        assert_eq!(path.last().unwrap(), &s.goal);
        // every configuration on the path, sampled finely, is free and inside the limits
        for pair in path.windows(2) {
            for i in 0..=50 {
                let t = f64::from(i) / 50.0;
                let q: Vec<f64> = pair[0]
                    .iter()
                    .zip(&pair[1])
                    .map(|(a, b)| a + (b - a) * t)
                    .collect();
                assert!(!in_collision(&s.arm, &q, &s.obstacles, &s.boxes), "{q:?}");
                assert!(q.iter().all(|a| a.abs() <= 3.0));
            }
        }
        assert_eq!(
            plan_arm(&s).unwrap(),
            path,
            "the same seed gives the same path"
        );
    }

    #[test]
    fn a_start_or_goal_in_collision_and_a_wrong_scenario_are_refused() {
        let blocked = vec![Obstacle::new(Vec3::new(1.5, 0.0, 0.0), 0.2)];
        assert_eq!(
            plan_arm(&scenario([0.0, 0.0], [1.0, 0.0], blocked.clone())),
            Err(ArmError::StartInCollision)
        );
        assert_eq!(
            plan_arm(&scenario([1.0, 0.0], [0.0, 0.0], blocked)),
            Err(ArmError::GoalInCollision)
        );
        let mut short = scenario([0.0, 0.0], [1.0, 0.0], Vec::new());
        short.start = vec![0.0];
        assert!(matches!(plan_arm(&short), Err(ArmError::Invalid(t)) if t.contains("1 angles")));
        let mut outside = scenario([0.0, 0.0], [1.0, 0.0], Vec::new());
        outside.goal = vec![4.0, 0.0];
        assert!(
            matches!(plan_arm(&outside), Err(ArmError::Invalid(t)) if t.contains("outside the limits"))
        );
        let mut zero_axis = scenario([0.0, 0.0], [1.0, 0.0], Vec::new());
        zero_axis.arm.joints[0].axis = Vec3::new(0.0, 0.0, 0.0);
        assert!(matches!(plan_arm(&zero_axis), Err(ArmError::Invalid(t)) if t.contains("axis")));
        let mut bad = scenario([0.0, 0.0], [1.0, 0.0], Vec::new());
        bad.config.step_rad = 0.0;
        assert!(matches!(plan_arm(&bad), Err(ArmError::Invalid(t)) if t.contains("step_rad")));
    }

    #[test]
    fn a_goal_inside_a_box_is_refused_before_the_search() {
        // a wall that the goal arm crosses
        let mut s = scenario([0.0, 0.0], [std::f64::consts::FRAC_PI_2, 0.0], Vec::new());
        s.boxes = vec![BoxObstacle {
            min: Vec3::new(-5.0, 0.5, -5.0),
            max: Vec3::new(5.0, 0.7, 5.0),
        }];
        s.goal = vec![std::f64::consts::FRAC_PI_2, 0.0];
        // the goal arm points along +y and crosses the wall, so the goal itself is in collision
        assert_eq!(plan_arm(&s), Err(ArmError::GoalInCollision));
    }
    #[test]
    fn every_kind_of_wrong_scenario_is_refused_by_name() {
        let base = || scenario([0.0, 0.0], [1.0, 0.0], Vec::new());
        let refused = |change: &dyn Fn(&mut ArmScenario), word: &str| {
            let mut s = base();
            change(&mut s);
            match plan_arm(&s) {
                Err(ArmError::Invalid(text)) => assert!(text.contains(word), "{text}"),
                other => panic!("expected a refusal naming {word}, got {other:?}"),
            }
        };
        refused(&|s| s.arm.joints.clear(), "no joints");
        refused(&|s| s.arm.link_radius = -1.0, "link_radius");
        refused(&|s| s.arm.link_radius = f64::NAN, "link_radius");
        refused(&|s| s.arm.tip = Vec3::new(f64::INFINITY, 0.0, 0.0), "tip");
        refused(
            &|s| s.arm.base = Some(Vec3::new(f64::NAN, 0.0, 0.0)),
            "base",
        );
        refused(
            &|s| s.arm.joints[1].origin = Vec3::new(f64::NAN, 0.0, 0.0),
            "origin",
        );
        refused(
            &|s| s.arm.joints[1].rpy = Some(Vec3::new(f64::NAN, 0.0, 0.0)),
            "rpy",
        );
        refused(&|s| s.arm.joints[0].lower = 5.0, "limits");
        refused(&|s| s.arm.joints[0].upper = f64::NAN, "limits");
        refused(&|s| s.start = vec![0.0, f64::NAN], "outside the limits");
        refused(&|s| s.config.resolution_rad = -1.0, "resolution_rad");
        refused(&|s| s.config.goal_threshold_rad = 0.0, "goal_threshold_rad");
        refused(&|s| s.config.goal_bias = 1.5, "goal_bias");
        refused(&|s| s.config.max_iterations = 0, "max_iterations");
        refused(
            &|s| s.config.max_iterations = MAX_ITERATIONS_LIMIT + 1,
            "max_iterations",
        );
        refused(
            &|s| s.obstacles = vec![Obstacle::new(Vec3::new(f64::NAN, 0.0, 0.0), 0.1)],
            "NonFiniteObstacle",
        );
        refused(
            &|s| {
                s.boxes = vec![BoxObstacle {
                    min: Vec3::new(1.0, 0.0, 0.0),
                    max: Vec3::new(0.0, 1.0, 1.0),
                }]
            },
            "InvertedBox",
        );
        assert_eq!(ArmError::StartInCollision.reason(), "start_in_collision");
        assert_eq!(ArmError::GoalInCollision.reason(), "goal_in_collision");
        assert_eq!(ArmError::NoPathFound.reason(), "no_path_found");
        assert!(ArmError::Invalid("x".into())
            .reason()
            .starts_with("invalid_input"));
    }

    #[test]
    fn a_goal_behind_a_blocked_range_of_one_joint_ends_without_a_path() {
        // one joint turning the tip about the z axis; a sphere on the tip at angle zero separates the two ends of its range
        let arm = Arm {
            base: None,
            joints: vec![ArmJoint {
                name: String::new(),
                origin: Vec3::new(0.0, 0.0, 0.0),
                rpy: Some(Vec3::new(0.0, 0.0, 0.0)),
                axis: Vec3::new(0.0, 0.0, 1.0),
                lower: -1.0,
                upper: 1.0,
            }],
            tip: Vec3::new(1.0, 0.0, 0.0),
            link_radius: 0.05,
        };
        let s = ArmScenario {
            arm,
            start: vec![-1.0],
            goal: vec![1.0],
            obstacles: vec![Obstacle::new(Vec3::new(1.0, 0.0, 0.0), 0.2)],
            boxes: Vec::new(),
            config: ArmConfig {
                max_iterations: 300,
                ..ArmConfig::default()
            },
            seed: None,
        };
        assert_eq!(plan_arm(&s), Err(ArmError::NoPathFound));
    }
}

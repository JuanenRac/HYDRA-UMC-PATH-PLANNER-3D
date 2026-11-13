// HYDRA-UMC-PATH-PLANNER-3D - src/fleet.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// Several robots planned one after the other in the order of their priority. The first robot plans as usual; the path of every robot that found
// one is then kept as a row of spheres along its whole length (its robot radius, a little inflated and spaced so that the spheres leave no gap
// in the tube they stand for), and the next robots plan around them. The row stands for the robot at EVERY time at once, so a later robot never
// plans through a place an earlier one passes, even one the earlier robot passes after the later one has gone by. That is safe and it is
// conservative: a fleet whose robots could share a corridor one after the other, by timing, may get no path for a later robot. Planning with
// time as a dimension (space-time RRT) is the refinement; this module is the version without it.

use crate::geometry::Vec3;
use crate::obstacle::{BoxObstacle, Obstacle};
use crate::rrt::{plan_with_boxes, PlanError, PlannerConfig, Workspace};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct FleetRobot {
    pub id: String,
    pub start: Vec3,
    pub goal: Vec3,
    #[serde(default)]
    pub config: Option<PlannerConfig>,
    #[serde(default)]
    pub seed: Option<u64>,
}

/// The robots in the order of their priority (the first one has it), the workspace they share and the fixed obstacles.
#[derive(Debug, Deserialize)]
pub struct Fleet {
    pub robots: Vec<FleetRobot>,
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
    #[serde(default)]
    pub boxes: Vec<BoxObstacle>,
    pub workspace: Workspace,
}

/// How much the spheres of a reserved path are inflated over the robot radius, and the spacing of their centres as a fraction of that radius.
/// Spheres of radius r spaced d apart cover a tube of radius sqrt(r^2 - (d/2)^2); with d = r/2 that is 0.968 r, so the inflation of 1.05 leaves the
/// tube at least as wide as the robot.
const INFLATION: f64 = 1.05;
const SPACING: f64 = 0.5;
/// Used when the robot radius is zero, so that the row still has centres.
const MIN_SPACING: f64 = 0.01;

/// The spheres that stand for a robot of radius `radius` moving along `path`.
pub fn reserved_spheres(path: &[Vec3], radius: f64) -> Vec<Obstacle> {
    let sphere_radius = radius.max(0.0) * INFLATION + 1e-9;
    let spacing = (radius * SPACING).max(MIN_SPACING);
    let mut spheres = Vec::new();
    for (index, &a) in path.iter().enumerate() {
        spheres.push(Obstacle {
            center: a,
            radius: sphere_radius,
        });
        let Some(&b) = path.get(index + 1) else {
            continue;
        };
        let length = a.distance(b);
        let steps = (length / spacing).ceil() as usize;
        for step in 1..steps {
            let t = step as f64 / steps as f64;
            spheres.push(Obstacle {
                center: a.add(b.sub(a).scale(t)),
                radius: sphere_radius,
            });
        }
    }
    spheres
}

/// The result for every robot, in the order of the fleet.
pub fn plan_fleet(fleet: &Fleet) -> Vec<(String, Result<Vec<Vec3>, PlanError>)> {
    let mut obstacles = fleet.obstacles.clone();
    let mut results = Vec::with_capacity(fleet.robots.len());
    for robot in &fleet.robots {
        let config = robot.config.unwrap_or_default();
        let seed = robot.seed.unwrap_or(1);
        let result = plan_with_boxes(
            robot.start,
            robot.goal,
            &obstacles,
            &fleet.boxes,
            fleet.workspace,
            config,
            seed,
        );
        if let Ok(path) = &result {
            obstacles.extend(reserved_spheres(path, config.robot_radius));
        }
        results.push((robot.id.clone(), result));
    }
    results
}

#[cfg(test)]
mod tests {
    use super::*;

    fn workspace() -> Workspace {
        Workspace {
            min: Vec3::new(-5.0, -5.0, -5.0),
            max: Vec3::new(5.0, 5.0, 5.0),
        }
    }

    fn robot(id: &str, start: Vec3, goal: Vec3, seed: u64) -> FleetRobot {
        FleetRobot {
            id: id.to_string(),
            start,
            goal,
            config: None,
            seed: Some(seed),
        }
    }

    fn sampled(path: &[Vec3]) -> Vec<Vec3> {
        let mut points = Vec::new();
        for pair in path.windows(2) {
            for i in 0..=100 {
                let t = f64::from(i) / 100.0;
                points.push(pair[0].add(pair[1].sub(pair[0]).scale(t)));
            }
        }
        points
    }

    #[test]
    fn the_reserved_spheres_cover_the_tube_of_the_robot_along_its_path() {
        let path = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)];
        let spheres = reserved_spheres(&path, 0.1);
        // every point of the tube of radius 0.1 around the segment is inside some sphere
        for i in 0..=200 {
            let x = 2.0 * f64::from(i) / 200.0;
            for (dy, dz) in [(0.1, 0.0), (0.0, 0.1), (-0.1, 0.0), (0.0, 0.0)] {
                let p = Vec3::new(x, dy, dz);
                assert!(
                    spheres.iter().any(|s| s.contains_point(p, 0.0)),
                    "{p:?} is not covered"
                );
            }
        }
    }

    #[test]
    fn a_robot_that_crosses_the_path_of_the_first_goes_around_it_and_keeps_its_distance() {
        let fleet = Fleet {
            robots: vec![
                robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
                robot("b", Vec3::new(0.0, -3.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
            ],
            obstacles: Vec::new(),
            boxes: Vec::new(),
            workspace: workspace(),
        };
        let results = plan_fleet(&fleet);
        let a = results[0]
            .1
            .as_ref()
            .expect("the first robot plans as alone");
        let b = results[1]
            .1
            .as_ref()
            .expect("the second robot finds its way around");
        let (pa, pb) = (sampled(a), sampled(b));
        let radius = PlannerConfig::default().robot_radius;
        for p in &pa {
            for q in &pb {
                assert!(
                    p.distance(*q) >= 2.0 * radius,
                    "{p:?} and {q:?} are too close"
                );
            }
        }
    }

    #[test]
    fn a_robot_whose_start_is_on_the_path_of_the_first_gets_no_plan_and_the_others_go_on() {
        let fleet = Fleet {
            robots: vec![
                robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
                robot("b", Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
                robot("c", Vec3::new(-3.0, 4.0, 0.0), Vec3::new(3.0, 4.0, 0.0), 3),
            ],
            obstacles: Vec::new(),
            boxes: Vec::new(),
            workspace: workspace(),
        };
        let results = plan_fleet(&fleet);
        assert!(results[0].1.is_ok());
        assert_eq!(results[1].1, Err(PlanError::StartInsideObstacle));
        assert!(results[2].1.is_ok(), "{:?}", results[2].1);
    }

    #[test]
    fn the_same_fleet_gives_the_same_paths() {
        let make = || Fleet {
            robots: vec![
                robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
                robot("b", Vec3::new(0.0, -3.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
            ],
            obstacles: Vec::new(),
            boxes: Vec::new(),
            workspace: workspace(),
        };
        assert_eq!(plan_fleet(&make()), plan_fleet(&make()));
    }
}

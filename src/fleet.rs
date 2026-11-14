// HYDRA-UMC-PATH-PLANNER-3D - src/fleet.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// Several robots planned one after the other in the order of their priority. The first robot plans as usual; the path of every robot that found
// one is then kept as a row of spheres along its whole length (its robot radius, a little inflated and spaced so that the spheres leave no gap
// in the tube they stand for), and the next robots plan around them. The row stands for the robot at EVERY time at once, so a later robot never
// plans through a place an earlier one passes, even one the earlier robot passes after the later one has gone by. That is safe and it is
// conservative: a fleet whose robots could share a corridor one after the other, by timing, may get no path for a later robot.
//
// With a `timing` section the robots share TIME as well. Every robot moves at a constant speed along its path, waits at its start until it leaves
// and stays at its goal when it arrives, and a later robot plans as if it were alone (fixed obstacles only); then the two motions are compared in
// time, exactly (the closest approach of two points that move in straight lines is solved, not sampled). When the later robot would come too
// close to an earlier one it leaves later, in steps, up to a limit; when no departure works it goes back to planning around the whole path of
// every earlier robot, as without timing. So timing only ever finds more fleets that can be planned, and a robot that is given a plan never
// comes closer to another than the sum of their radii (the earlier one inflated as in the version without timing).

use crate::geometry::Vec3;
use crate::obstacle::{BoxObstacle, Obstacle};
use crate::rrt::{plan_with_boxes, PlanError, PlannerConfig, Workspace};
use serde::Deserialize;

fn default_speed() -> f64 {
    1.0
}
fn default_max_wait() -> f64 {
    60.0
}
fn default_wait_step() -> f64 {
    1.0
}

/// How the robots share time. `speed` is the default speed of a robot (length units per second); a robot that would meet an earlier one waits at
/// its start, in steps of `wait_step` seconds, for at most `max_wait` seconds. A value that is not positive and finite (not negative for
/// `max_wait`) is taken as its default.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Timing {
    #[serde(default = "default_speed")]
    pub speed: f64,
    #[serde(default = "default_max_wait")]
    pub max_wait: f64,
    #[serde(default = "default_wait_step")]
    pub wait_step: f64,
}

#[derive(Debug, Deserialize)]
pub struct FleetRobot {
    pub id: String,
    pub start: Vec3,
    pub goal: Vec3,
    #[serde(default)]
    pub config: Option<PlannerConfig>,
    #[serde(default)]
    pub seed: Option<u64>,
    /// Its own speed, with a `timing` section (the one of the section when absent).
    #[serde(default)]
    pub speed: Option<f64>,
    /// The earliest time it may leave, in seconds (zero when absent). Used only with a `timing` section.
    #[serde(default)]
    pub start_time: f64,
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
    /// When present the robots share time as well as space (see the top of this file).
    #[serde(default)]
    pub timing: Option<Timing>,
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

/// When a robot leaves its start and when it reaches its goal, in seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Schedule {
    pub departs_at: f64,
    pub arrives_at: f64,
}

/// The result for one robot: its path or why there is none, and its schedule when the fleet has a `timing` section.
#[derive(Debug, PartialEq)]
pub struct FleetPlan {
    pub id: String,
    pub result: Result<Vec<Vec3>, PlanError>,
    pub schedule: Option<Schedule>,
}

/// The most departures tried for one robot, whatever `max_wait` and `wait_step` say.
const MAX_DEPARTURES: usize = 10_000;

/// A robot moving at a constant speed along its path: it is at the first point until `times[0]`, at `points[i]` at `times[i]`, and at the last
/// point from the last time on.
struct Trajectory {
    points: Vec<Vec3>,
    times: Vec<f64>,
    radius: f64,
}

impl Trajectory {
    fn new(path: &[Vec3], departs_at: f64, speed: f64, radius: f64) -> Self {
        let mut times = vec![departs_at];
        for pair in path.windows(2) {
            let last = times[times.len() - 1];
            times.push(last + pair[0].distance(pair[1]) / speed);
        }
        Trajectory {
            points: path.to_vec(),
            times,
            radius,
        }
    }

    fn arrives_at(&self) -> f64 {
        self.times[self.times.len() - 1]
    }

    fn at(&self, t: f64) -> Vec3 {
        let last = self.points.len() - 1;
        if t <= self.times[0] {
            return self.points[0];
        }
        if t >= self.times[last] {
            return self.points[last];
        }
        let after = self.times.partition_point(|&x| x <= t);
        let i = after - 1;
        let span = self.times[after] - self.times[i];
        if span <= 0.0 {
            return self.points[i];
        }
        let fraction = (t - self.times[i]) / span;
        self.points[i].add(self.points[after].sub(self.points[i]).scale(fraction))
    }
}

/// The smallest length of `d0 + (d1 - d0) * s` for `s` from 0 to 1.
fn closest_approach(d0: Vec3, d1: Vec3) -> f64 {
    let v = d1.sub(d0);
    let vv = v.dot(v);
    let s = if vv <= 1e-18 {
        0.0
    } else {
        (-d0.dot(v) / vv).clamp(0.0, 1.0)
    };
    d0.add(v.scale(s)).length()
}

/// Whether two robots come closer than their radii allow (the earlier one inflated as in `reserved_spheres`) at any time, from time zero on.
/// Both stand still after their last time, so it is enough to look between the times where either changes direction.
fn in_conflict(earlier: &Trajectory, later: &Trajectory) -> bool {
    let limit = earlier.radius * INFLATION + later.radius;
    let mut marks = vec![0.0];
    marks.extend(earlier.times.iter().copied());
    marks.extend(later.times.iter().copied());
    marks.sort_by(f64::total_cmp);
    marks.dedup();
    let gap = |t: f64| earlier.at(t).sub(later.at(t));
    if marks.len() == 1 {
        return gap(marks[0]).length() < limit;
    }
    marks
        .windows(2)
        .any(|pair| closest_approach(gap(pair[0]), gap(pair[1])) < limit)
}

fn positive_or(value: f64, default: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        default
    }
}

/// The result for every robot, in the order of the fleet.
#[cfg(test)]
pub fn plan_fleet(fleet: &Fleet) -> Vec<(String, Result<Vec<Vec3>, PlanError>)> {
    plan_fleet_scheduled(fleet)
        .into_iter()
        .map(|plan| (plan.id, plan.result))
        .collect()
}

/// The result for every robot, in the order of the fleet, with the schedule of each when the fleet has a `timing` section.
pub fn plan_fleet_scheduled(fleet: &Fleet) -> Vec<FleetPlan> {
    match fleet.timing {
        Some(timing) => plan_timed(fleet, timing, plan_with_boxes),
        None => plan_untimed(fleet),
    }
}

/// What plans the path of one robot alone: the sampling planner, or a stand-in in the tests.
type PathPlanner = fn(
    Vec3,
    Vec3,
    &[Obstacle],
    &[BoxObstacle],
    Workspace,
    PlannerConfig,
    u64,
) -> Result<Vec<Vec3>, PlanError>;

fn plan_untimed(fleet: &Fleet) -> Vec<FleetPlan> {
    let mut obstacles = fleet.obstacles.clone();
    let mut plans = Vec::with_capacity(fleet.robots.len());
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
        plans.push(FleetPlan {
            id: robot.id.clone(),
            result,
            schedule: None,
        });
    }
    plans
}

fn plan_timed(fleet: &Fleet, timing: Timing, plan_path: PathPlanner) -> Vec<FleetPlan> {
    let default_speed = positive_or(timing.speed, 1.0);
    let step = positive_or(timing.wait_step, 1.0);
    let max_wait = if timing.max_wait.is_finite() && timing.max_wait >= 0.0 {
        timing.max_wait
    } else {
        0.0
    };
    let departures = ((max_wait / step).floor() as usize).min(MAX_DEPARTURES - 1) + 1;
    let mut trajectories: Vec<Trajectory> = Vec::new();
    let mut reserved: Vec<Obstacle> = Vec::new();
    let mut plans = Vec::with_capacity(fleet.robots.len());
    for robot in &fleet.robots {
        let config = robot.config.unwrap_or_default();
        let seed = robot.seed.unwrap_or(1);
        let speed = robot
            .speed
            .map_or(default_speed, |s| positive_or(s, default_speed));
        let earliest = if robot.start_time.is_finite() {
            robot.start_time.max(0.0)
        } else {
            0.0
        };
        let alone = plan_path(
            robot.start,
            robot.goal,
            &fleet.obstacles,
            &fleet.boxes,
            fleet.workspace,
            config,
            seed,
        );
        let mut chosen: Option<(Vec<Vec3>, Trajectory)> = None;
        let mut failure: Option<PlanError> = None;
        match alone {
            Err(e) => failure = Some(e),
            Ok(path) => {
                for k in 0..departures {
                    let candidate = Trajectory::new(
                        &path,
                        earliest + k as f64 * step,
                        speed,
                        config.robot_radius,
                    );
                    if !trajectories.iter().any(|t| in_conflict(t, &candidate)) {
                        chosen = Some((path.clone(), candidate));
                        break;
                    }
                }
                if chosen.is_none() {
                    // no departure works: keep clear of the whole path of every earlier robot, as without timing
                    let mut obstacles = fleet.obstacles.clone();
                    obstacles.extend(reserved.iter().copied());
                    match plan_path(
                        robot.start,
                        robot.goal,
                        &obstacles,
                        &fleet.boxes,
                        fleet.workspace,
                        config,
                        seed,
                    ) {
                        Ok(around) => {
                            let trajectory =
                                Trajectory::new(&around, earliest, speed, config.robot_radius);
                            chosen = Some((around, trajectory));
                        }
                        Err(e) => failure = Some(e),
                    }
                }
            }
        }
        match (chosen, failure) {
            (Some((path, trajectory)), _) => {
                reserved.extend(reserved_spheres(&path, config.robot_radius));
                let schedule = Schedule {
                    departs_at: trajectory.times[0],
                    arrives_at: trajectory.arrives_at(),
                };
                trajectories.push(trajectory);
                plans.push(FleetPlan {
                    id: robot.id.clone(),
                    result: Ok(path),
                    schedule: Some(schedule),
                });
            }
            (None, error) => plans.push(FleetPlan {
                id: robot.id.clone(),
                result: Err(error.unwrap_or(PlanError::NoPathFound)),
                schedule: None,
            }),
        }
    }
    plans
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
            speed: None,
            start_time: 0.0,
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
            timing: None,
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
    fn a_robot_that_starts_where_the_first_starts_gets_no_plan_and_the_others_go_on() {
        let fleet = Fleet {
            robots: vec![
                robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
                robot("b", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
                robot("c", Vec3::new(-3.0, 4.0, 0.0), Vec3::new(3.0, 4.0, 0.0), 3),
            ],
            obstacles: Vec::new(),
            boxes: Vec::new(),
            workspace: workspace(),
            timing: None,
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
            timing: None,
        };
        assert_eq!(plan_fleet(&make()), plan_fleet(&make()));
    }

    fn timed(robots: Vec<FleetRobot>, max_wait: f64) -> Fleet {
        Fleet {
            robots,
            obstacles: Vec::new(),
            boxes: Vec::new(),
            workspace: workspace(),
            timing: Some(Timing {
                speed: 1.0,
                max_wait,
                wait_step: 1.0,
            }),
        }
    }

    fn crossing() -> Vec<FleetRobot> {
        vec![
            robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
            robot("b", Vec3::new(0.0, -3.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
        ]
    }

    /// A stand-in for the sampling planner, so that where two robots meet does not depend on the random paths: a straight line when nothing is in
    /// the way, and a line that goes up and over the middle when there are obstacles (the paths of the earlier robots).
    fn straight_or_over(
        start: Vec3,
        goal: Vec3,
        obstacles: &[Obstacle],
        _boxes: &[BoxObstacle],
        _workspace: Workspace,
        _config: PlannerConfig,
        _seed: u64,
    ) -> Result<Vec<Vec3>, PlanError> {
        if obstacles.is_empty() {
            Ok(vec![start, goal])
        } else {
            let middle = start
                .add(goal.sub(start).scale(0.5))
                .add(Vec3::new(0.0, 0.0, 2.0));
            Ok(vec![start, middle, goal])
        }
    }

    fn plan_straight(fleet: &Fleet) -> Vec<FleetPlan> {
        plan_timed(fleet, fleet.timing.unwrap(), straight_or_over)
    }

    /// The smallest distance between two robots at any time, found by sampling the time finely.
    fn closest(a: &Trajectory, b: &Trajectory) -> f64 {
        let end = a.arrives_at().max(b.arrives_at());
        (0..=4000)
            .map(|i| {
                let t = end * f64::from(i) / 4000.0;
                a.at(t).distance(b.at(t))
            })
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn a_trajectory_waits_at_its_start_moves_at_its_speed_and_stays_at_its_goal() {
        let path = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 4.0, 0.0),
        ];
        let trajectory = Trajectory::new(&path, 3.0, 2.0, 0.1);
        assert_eq!(trajectory.times, vec![3.0, 4.0, 6.0]);
        assert_eq!(trajectory.at(0.0), path[0]);
        assert_eq!(trajectory.at(3.0), path[0]);
        assert_eq!(trajectory.at(3.5), Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(trajectory.at(5.0), Vec3::new(2.0, 2.0, 0.0));
        assert_eq!(trajectory.at(6.0), path[2]);
        assert_eq!(trajectory.at(100.0), path[2]);
        assert_eq!(trajectory.arrives_at(), 6.0);
    }

    #[test]
    fn two_robots_in_conflict_are_found_exactly_and_two_that_pass_one_after_the_other_are_not() {
        let along = |departs: f64, from: f64, to: f64| {
            Trajectory::new(
                &[Vec3::new(from, 0.0, 0.0), Vec3::new(to, 0.0, 0.0)],
                departs,
                1.0,
                0.1,
            )
        };
        // head on, on the same line: they meet
        assert!(in_conflict(&along(0.0, -3.0, 3.0), &along(0.0, 3.0, -3.0)));
        // two robots on the same line that never come within reach of each other
        assert!(!in_conflict(&along(0.0, 0.0, 2.0), &along(0.0, 4.0, 6.0)));
        // so close at one instant that only the exact closest approach, between two sampled times, sees it
        let a = Trajectory::new(
            &[Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
            0.0,
            1.0,
            0.1,
        );
        let b = Trajectory::new(
            &[Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)],
            0.0,
            1.0,
            0.1,
        );
        assert!(in_conflict(&a, &b));
        let late = Trajectory::new(
            &[Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)],
            1.5,
            1.0,
            0.1,
        );
        assert!(!in_conflict(&a, &late));
    }

    #[test]
    fn a_robot_that_would_cross_an_earlier_one_in_time_leaves_later_and_keeps_its_distance() {
        // straight lines that cross at the middle, where both would be at the same time
        let plans = plan_straight(&timed(crossing(), 60.0));
        let (a, b) = (&plans[0], &plans[1]);
        let (path_a, path_b) = (a.result.as_ref().unwrap(), b.result.as_ref().unwrap());
        let (schedule_a, schedule_b) = (a.schedule.unwrap(), b.schedule.unwrap());
        assert_eq!(schedule_a.departs_at, 0.0);
        assert!(schedule_b.departs_at > 0.0, "b should wait: {schedule_b:?}");
        assert!(schedule_b.arrives_at > schedule_b.departs_at);
        let radius = PlannerConfig::default().robot_radius;
        let ta = Trajectory::new(path_a, schedule_a.departs_at, 1.0, radius);
        let tb = Trajectory::new(path_b, schedule_b.departs_at, 1.0, radius);
        assert!(closest(&ta, &tb) >= 2.0 * radius);
    }

    #[test]
    fn robots_that_never_meet_leave_at_once() {
        let robots = vec![
            robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
            robot("c", Vec3::new(-3.0, 4.0, 0.0), Vec3::new(3.0, 4.0, 0.0), 3),
        ];
        let plans = plan_fleet_scheduled(&timed(robots, 60.0));
        for plan in &plans {
            assert_eq!(plan.schedule.unwrap().departs_at, 0.0);
            assert!(plan.result.is_ok());
        }
    }

    #[test]
    fn with_no_time_to_wait_the_second_robot_goes_around_the_first_as_without_timing() {
        let plans = plan_straight(&timed(crossing(), 0.0));
        let b = &plans[1];
        assert_eq!(b.schedule.unwrap().departs_at, 0.0);
        // the path planned with the whole path of the first robot in the way, as without timing
        let path_b = b.result.as_ref().unwrap();
        assert_eq!(
            path_b,
            &vec![
                Vec3::new(0.0, -3.0, 0.0),
                Vec3::new(0.0, 0.0, 2.0),
                Vec3::new(0.0, 3.0, 0.0)
            ]
        );
    }

    #[test]
    fn a_robot_that_starts_where_an_earlier_one_stands_gets_no_plan_with_timing_either() {
        let robots = vec![
            robot("a", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), 1),
            robot("b", Vec3::new(-3.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0), 2),
        ];
        let plans = plan_fleet_scheduled(&timed(robots, 60.0));
        assert!(plans[0].result.is_ok());
        assert_eq!(plans[1].result, Err(PlanError::StartInsideObstacle));
        assert_eq!(plans[1].schedule, None);
    }

    #[test]
    fn the_speed_of_a_robot_and_its_earliest_departure_are_respected() {
        let mut robots = crossing();
        robots[1].speed = Some(2.0);
        robots[1].start_time = 100.0;
        let plans = plan_fleet_scheduled(&timed(robots, 60.0));
        let schedule = plans[1].schedule.unwrap();
        assert_eq!(schedule.departs_at, 100.0);
        let length: f64 = plans[1]
            .result
            .as_ref()
            .unwrap()
            .windows(2)
            .map(|pair| pair[0].distance(pair[1]))
            .sum();
        assert!(
            (schedule.arrives_at - (100.0 + length / 2.0)).abs() < 1e-9,
            "{schedule:?}"
        );
    }

    #[test]
    fn values_that_make_no_sense_are_taken_as_their_defaults() {
        let mut robots = crossing();
        robots[0].speed = Some(-1.0);
        robots[1].start_time = f64::NAN;
        let mut fleet = timed(robots, f64::NAN);
        fleet.timing = Some(Timing {
            speed: 0.0,
            max_wait: f64::NAN,
            wait_step: -2.0,
        });
        let plans = plan_fleet_scheduled(&fleet);
        assert!(plans.iter().all(|p| p.result.is_ok()));
        assert_eq!(plans[0].schedule.unwrap().departs_at, 0.0);
        assert_eq!(plans[1].schedule.unwrap().departs_at, 0.0);
    }

    #[test]
    fn the_timing_section_has_defaults_and_is_optional() {
        let text = r#"{"workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},
          "timing":{},
          "robots":[{"id":"a","start":{"x":-3,"y":0,"z":0},"goal":{"x":3,"y":0,"z":0},"speed":2.0,"start_time":1.5}]}"#;
        let fleet: Fleet = serde_json::from_str(text).unwrap();
        let timing = fleet.timing.unwrap();
        assert_eq!(
            (timing.speed, timing.max_wait, timing.wait_step),
            (1.0, 60.0, 1.0)
        );
        assert_eq!(fleet.robots[0].speed, Some(2.0));
        assert_eq!(fleet.robots[0].start_time, 1.5);
        let plain: Fleet = serde_json::from_str(
            r#"{"workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"robots":[]}"#,
        )
        .unwrap();
        assert!(plain.timing.is_none());
    }

    #[test]
    fn the_same_timed_fleet_gives_the_same_plans() {
        assert_eq!(
            plan_fleet_scheduled(&timed(crossing(), 60.0)),
            plan_fleet_scheduled(&timed(crossing(), 60.0))
        );
    }
}

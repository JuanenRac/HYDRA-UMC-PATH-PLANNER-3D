// HYDRA-UMC-PATH-PLANNER-3D - rrt.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// The real collision-free path search: a standard RRT (Rapidly-exploring
// Random Tree) and, when `optimize_iterations` is set, RRT* (the cheapest
// parent among the neighbours of a new node, a rewiring pass, and more
// iterations after the first path to shorten it). It is not yet
// multi-robot-synchronized (plans one agent through a static
// obstacle set, does not coordinate several agents' paths against each
// other). Both are real, scoped-out future work: proving a
// single-agent planner correct came first. Nearest-neighbor lookup uses
// a real, incremental KD-tree (kdtree.rs) - that module's own tests
// verify it stays exactly equivalent (same index on every tie, not just
// "a" valid nearest neighbor) to a real independent linear scan across
// many random insertion/query sequences, which is what actually lets
// this file's own seed-determinism tests below keep passing unchanged.

use crate::geometry::Vec3;
use crate::kdtree::KdTree;
use crate::obstacle::{BoxObstacle, Obstacle, ObstacleIndex};
use crate::rng::Xorshift64Star;
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Workspace {
    pub min: Vec3,
    pub max: Vec3,
}

impl Workspace {
    /// Is a robot of radius `margin` centred on `p` entirely inside the
    /// workspace? The workspace is the volume the whole body must stay in
    /// (walls, the next cell), not only its centre: a centre touching the
    /// wall leaves half of the body outside it.
    pub fn contains_with_margin(&self, p: Vec3, margin: f64) -> bool {
        p.x >= self.min.x + margin
            && p.x <= self.max.x - margin
            && p.y >= self.min.y + margin
            && p.y <= self.max.y - margin
            && p.z >= self.min.z + margin
            && p.z <= self.max.z - margin
    }

    /// A point of the workspace a robot of radius `margin` can be centred on.
    fn sample(&self, rng: &mut Xorshift64Star, margin: f64) -> Vec3 {
        Vec3::new(
            rng.next_range(self.min.x + margin, self.max.x - margin),
            rng.next_range(self.min.y + margin, self.max.y - margin),
            rng.next_range(self.min.z + margin, self.max.z - margin),
        )
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(default)]
pub struct PlannerConfig {
    pub max_iterations: u32,
    pub step_size: f64,
    /// Probability (0.0-1.0) of sampling the goal directly instead of a
    /// random workspace point, each iteration - biases the tree to grow
    /// toward the goal instead of relying on pure chance to wander there.
    pub goal_bias: f64,
    /// A new tree node within this distance of the goal, with a clear
    /// line of sight to it, connects the path.
    pub goal_threshold: f64,
    /// Treated as the planned agent's own radius for collision checks -
    /// keeps the path clear of obstacles by more than a single
    /// dimensionless point would.
    pub robot_radius: f64,
    /// Wall-clock budget for the search, independent of
    /// `max_iterations` (which is itself capped, see
    /// `semantics::MAX_ITERATIONS_LIMIT`). `None` (the default) means no time limit -
    /// existing scenarios/tests that never set this field behave
    /// exactly as before. A real time limit matters because
    /// `max_iterations` alone cannot bound wall-clock time: a scene
    /// with many obstacles makes each iteration's collision checks
    /// proportionally slower, so the same iteration count can take
    /// wildly different real time on a pathological scene versus an
    /// open one - and this planner may run on a real-time control loop
    /// that cannot wait indefinitely for an answer.
    #[serde(default)]
    pub max_duration_ms: Option<u64>,
    /// RRT* mode. 0 (the default) is the plain RRT: the search stops at the
    /// first path, exactly as before. Above 0 every new node takes the
    /// cheapest parent within `rewire_radius`, the neighbours are rewired
    /// through it when that shortens them, and after the first path the
    /// search goes on for this many more iterations to shorten it.
    pub optimize_iterations: u32,
    /// Radius of the neighbourhood used to choose a parent and to rewire.
    /// Used only when `optimize_iterations` is above 0.
    pub rewire_radius: f64,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        PlannerConfig {
            max_iterations: 5000,
            step_size: 0.5,
            goal_bias: 0.05,
            goal_threshold: 0.5,
            robot_radius: 0.1,
            max_duration_ms: None,
            optimize_iterations: 0,
            rewire_radius: 1.0,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum PlanError {
    StartInsideObstacle,
    GoalInsideObstacle,
    StartOutsideWorkspace,
    GoalOutsideWorkspace,
    /// Genuinely honest outcome, not just "we didn't try hard enough":
    /// the search exhausted `max_iterations` without connecting start to
    /// goal. This can mean the goal is truly unreachable, or that the
    /// iteration budget was too low for this scene - the planner cannot
    /// tell the two apart, and does not pretend to.
    NoPathFound,
    /// The search hit `config.max_duration_ms` before either finding a
    /// path or exhausting `max_iterations` - distinct from `NoPathFound`
    /// so a caller on a real-time budget can tell "genuinely searched
    /// the whole budget and found nothing" apart from "ran out of wall
    /// clock, unknown whether a path exists".
    TimeLimitExceeded,
    /// the scenario's own numeric inputs are not physically
    /// valid (negative/non-finite radius, inverted workspace bound,
    /// out-of-range planner parameter) - rejected before any real
    /// geometry check runs, never silently treated as "no path found".
    InvalidInput(crate::semantics::InvalidScenarioReason),
}

struct TreeNode {
    point: Vec3,
    parent: Option<usize>,
}

/// Runs one RRT search from `start` to `goal`, avoiding every obstacle in
/// `obstacles` by at least `config.robot_radius`, within `workspace`
/// bounds. `seed` makes the search fully deterministic - the same
/// scenario and seed always produce the same path (or the same failure).
pub fn plan(
    start: Vec3,
    goal: Vec3,
    obstacles: &[Obstacle],
    workspace: Workspace,
    config: PlannerConfig,
    seed: u64,
) -> Result<Vec<Vec3>, PlanError> {
    plan_with_boxes(start, goal, obstacles, &[], workspace, config, seed)
}

/// `plan`, with the box obstacles of the scenario as well as the spheres: the path keeps the robot radius away from both.
pub fn plan_with_boxes(
    start: Vec3,
    goal: Vec3,
    obstacles: &[Obstacle],
    boxes: &[BoxObstacle],
    workspace: Workspace,
    config: PlannerConfig,
    seed: u64,
) -> Result<Vec<Vec3>, PlanError> {
    // reject a semantically invalid scenario before any real
    // geometry check runs against it - see semantics.rs's own module
    // doc for why a negative robot_radius/obstacle radius or an
    // inverted workspace bound must never be allowed to silently defeat
    // collision detection instead of producing a real, explicit error.
    crate::semantics::validate_planner_config(&config).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_workspace(&workspace).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_obstacles(obstacles).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_boxes(boxes).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_point(start).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_point(goal).map_err(PlanError::InvalidInput)?;
    crate::semantics::validate_workspace_fits_robot(&workspace, config.robot_radius)
        .map_err(PlanError::InvalidInput)?;

    if !workspace.contains_with_margin(start, config.robot_radius) {
        return Err(PlanError::StartOutsideWorkspace);
    }
    if !workspace.contains_with_margin(goal, config.robot_radius) {
        return Err(PlanError::GoalOutsideWorkspace);
    }
    if obstacles
        .iter()
        .any(|o| o.contains_point(start, config.robot_radius))
        || boxes
            .iter()
            .any(|b| b.contains_point(start, config.robot_radius))
    {
        return Err(PlanError::StartInsideObstacle);
    }
    if obstacles
        .iter()
        .any(|o| o.contains_point(goal, config.robot_radius))
        || boxes
            .iter()
            .any(|b| b.contains_point(goal, config.robot_radius))
    {
        return Err(PlanError::GoalInsideObstacle);
    }

    if start == goal {
        // already there: the path is the one waypoint
        return Ok(vec![start]);
    }

    // one hierarchy for the whole search: every extension asks "is this segment clear?"
    let index = ObstacleIndex::with_boxes(obstacles, boxes);

    if config.optimize_iterations > 0 {
        return plan_optimizing(start, goal, &index, workspace, config, seed);
    }

    let mut rng = Xorshift64Star::new(seed);
    let mut tree = vec![TreeNode {
        point: start,
        parent: None,
    }];
    let mut kd = KdTree::new();
    kd.insert(start, 0);

    let is_clear =
        |a: Vec3, b: Vec3| -> bool { !index.any_intersects_segment(a, b, config.robot_radius) };

    let deadline = config.max_duration_ms.map(Duration::from_millis);
    let search_started = Instant::now();

    for _ in 0..config.max_iterations {
        if let Some(deadline) = deadline {
            if search_started.elapsed() >= deadline {
                return Err(PlanError::TimeLimitExceeded);
            }
        }

        let sample = if rng.next_f64() < config.goal_bias {
            goal
        } else {
            workspace.sample(&mut rng, config.robot_radius)
        };

        let nearest_idx = kd.nearest(sample);
        let nearest_point = tree[nearest_idx].point;

        let to_sample = sample.sub(nearest_point);
        let dist = to_sample.length();
        let new_point = if dist <= config.step_size {
            sample
        } else {
            nearest_point.add(to_sample.normalized().scale(config.step_size))
        };

        if !is_clear(nearest_point, new_point) {
            continue;
        }

        tree.push(TreeNode {
            point: new_point,
            parent: Some(nearest_idx),
        });
        let new_idx = tree.len() - 1;
        kd.insert(new_point, new_idx);

        if new_point.distance(goal) <= config.goal_threshold && is_clear(new_point, goal) {
            // when the new node already is the goal (a goal-biased sample
            // close enough to land on it) it is the end of the path: adding
            // the goal again would end the path with a segment of length 0
            if new_point == goal {
                return Ok(reconstruct_path(&tree, new_idx));
            }
            tree.push(TreeNode {
                point: goal,
                parent: Some(new_idx),
            });
            return Ok(reconstruct_path(&tree, tree.len() - 1));
        }
    }

    Err(PlanError::NoPathFound)
}

/// RRT*: the same sampling and steering as `plan`, with a cost per node.
/// A new node takes the cheapest collision-free parent within
/// `rewire_radius`, then every neighbour that would be cheaper through it is
/// rewired to it (the cost of its whole subtree follows). After the first
/// node close enough to the goal the search runs `optimize_iterations` more
/// iterations, and the answer is the cheapest way to the goal over all the
/// nodes that see it. Costs only ever go down, so for the same seed the path
/// is never longer than the one the plain RRT returns.
fn plan_optimizing(
    start: Vec3,
    goal: Vec3,
    index: &ObstacleIndex,
    workspace: Workspace,
    config: PlannerConfig,
    seed: u64,
) -> Result<Vec<Vec3>, PlanError> {
    let mut rng = Xorshift64Star::new(seed);
    let mut tree = vec![TreeNode {
        point: start,
        parent: None,
    }];
    let mut cost: Vec<f64> = vec![0.0];
    let mut children: Vec<Vec<usize>> = vec![Vec::new()];
    let mut kd = KdTree::new();
    kd.insert(start, 0);

    let is_clear =
        |a: Vec3, b: Vec3| -> bool { !index.any_intersects_segment(a, b, config.robot_radius) };

    let deadline = config.max_duration_ms.map(Duration::from_millis);
    let search_started = Instant::now();
    let mut found = false;
    let mut iterations: u32 = 0;
    let mut extra: u32 = 0;

    loop {
        if found {
            if extra >= config.optimize_iterations {
                break;
            }
            extra += 1;
        } else {
            if iterations >= config.max_iterations {
                break;
            }
            iterations += 1;
        }
        if let Some(deadline) = deadline {
            if search_started.elapsed() >= deadline {
                if found {
                    break;
                }
                return Err(PlanError::TimeLimitExceeded);
            }
        }

        let sample = if rng.next_f64() < config.goal_bias {
            goal
        } else {
            workspace.sample(&mut rng, config.robot_radius)
        };

        let nearest_idx = kd.nearest(sample);
        let nearest_point = tree[nearest_idx].point;
        let to_sample = sample.sub(nearest_point);
        let dist = to_sample.length();
        let new_point = if dist <= config.step_size {
            sample
        } else {
            nearest_point.add(to_sample.normalized().scale(config.step_size))
        };
        if !is_clear(nearest_point, new_point) {
            continue;
        }

        // the neighbourhood, in index order so the search stays deterministic
        let mut near: Vec<usize> = (0..tree.len())
            .filter(|&i| tree[i].point.distance(new_point) <= config.rewire_radius)
            .collect();
        if !near.contains(&nearest_idx) {
            near.push(nearest_idx);
        }

        let mut parent = nearest_idx;
        let mut new_cost = cost[nearest_idx] + nearest_point.distance(new_point);
        for &n in &near {
            let through = cost[n] + tree[n].point.distance(new_point);
            if through < new_cost && is_clear(tree[n].point, new_point) {
                parent = n;
                new_cost = through;
            }
        }

        tree.push(TreeNode {
            point: new_point,
            parent: Some(parent),
        });
        cost.push(new_cost);
        children.push(Vec::new());
        let new_idx = tree.len() - 1;
        children[parent].push(new_idx);
        kd.insert(new_point, new_idx);

        for &n in &near {
            if n == parent {
                continue;
            }
            let through = new_cost + new_point.distance(tree[n].point);
            if through + 1e-12 < cost[n] && is_clear(new_point, tree[n].point) {
                if let Some(old) = tree[n].parent {
                    children[old].retain(|&c| c != n);
                }
                tree[n].parent = Some(new_idx);
                children[new_idx].push(n);
                let delta = through - cost[n];
                let mut stack = vec![n];
                while let Some(x) = stack.pop() {
                    cost[x] += delta;
                    stack.extend(children[x].iter().copied());
                }
            }
        }

        if !found && new_point.distance(goal) <= config.goal_threshold && is_clear(new_point, goal)
        {
            found = true;
        }
    }

    if !found {
        return Err(PlanError::NoPathFound);
    }

    // the cheapest way to the goal over every node that is close enough and sees it
    let mut best: Option<(usize, f64)> = None;
    for (i, node) in tree.iter().enumerate() {
        if node.point.distance(goal) > config.goal_threshold {
            continue;
        }
        let total = cost[i] + node.point.distance(goal);
        if best.is_none_or(|(_, t)| total < t) && is_clear(node.point, goal) {
            best = Some((i, total));
        }
    }
    let (end, _) = best.ok_or(PlanError::NoPathFound)?;
    let mut path = reconstruct_path(&tree, end);
    if tree[end].point != goal {
        path.push(goal);
    }
    Ok(path)
}

fn reconstruct_path(tree: &[TreeNode], mut idx: usize) -> Vec<Vec3> {
    let mut path = vec![tree[idx].point];
    while let Some(parent) = tree[idx].parent {
        path.push(tree[parent].point);
        idx = parent;
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corpus;

    fn open_workspace() -> Workspace {
        Workspace {
            min: Vec3::new(-10.0, -10.0, -10.0),
            max: Vec3::new(10.0, 10.0, 10.0),
        }
    }

    #[test]
    fn finds_a_direct_path_with_no_obstacles() {
        let start = Vec3::new(0.0, 0.0, 0.0);
        let goal = Vec3::new(5.0, 0.0, 0.0);
        let path = plan(
            start,
            goal,
            &[],
            open_workspace(),
            PlannerConfig::default(),
            1,
        )
        .expect("open workspace must always find a path");

        assert_eq!(path.first().copied(), Some(start));
        assert_eq!(path.last().copied(), Some(goal));
        assert!(path.len() >= 2);
    }

    // a negative robot_radius must reject the plan outright, not
    // let obstacle.rs's own effective-radius arithmetic go negative and
    // silently defeat every collision check the search relies on.
    #[test]
    fn plan_rejects_a_negative_robot_radius_instead_of_searching_with_defeated_collision_checks() {
        let config = PlannerConfig {
            robot_radius: -2.0,
            ..PlannerConfig::default()
        };
        let obstacles = vec![crate::obstacle::Obstacle::new(
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
        )];
        let result = plan(
            Vec3::new(-5.0, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            &obstacles,
            open_workspace(),
            config,
            7,
        );
        assert_eq!(
            result,
            Err(PlanError::InvalidInput(
                crate::semantics::InvalidScenarioReason::NegativeRobotRadius
            ))
        );
    }

    #[test]
    fn a_zero_duration_time_limit_is_hit_before_any_useful_search() {
        let config = PlannerConfig {
            max_iterations: 5000,
            max_duration_ms: Some(0),
            ..PlannerConfig::default()
        };
        let result = plan(
            corpus::start(),
            corpus::goal(),
            &corpus::no_obstacles(),
            corpus::open_workspace(),
            config,
            1,
        );
        assert_eq!(result, Err(PlanError::TimeLimitExceeded));
    }

    #[test]
    fn a_generous_time_limit_does_not_interfere_with_a_normal_search() {
        // Same scenario/seed as `same_seed_and_scenario_is_fully_deterministic`
        // below, but with an explicit (generous) time limit set - must
        // reach the exact same result as leaving it unset entirely.
        let obstacles = corpus::single_blocking_obstacle();
        let config = PlannerConfig::default();
        let unlimited = plan(
            corpus::start(),
            corpus::goal(),
            &obstacles,
            corpus::open_workspace(),
            config,
            99,
        )
        .unwrap();

        let limited_config = PlannerConfig {
            max_duration_ms: Some(5_000),
            ..config
        };
        let limited = plan(
            corpus::start(),
            corpus::goal(),
            &obstacles,
            corpus::open_workspace(),
            limited_config,
            99,
        )
        .unwrap();

        assert_eq!(unlimited, limited);
    }

    #[test]
    fn no_time_limit_by_default_matches_prior_behavior() {
        assert_eq!(PlannerConfig::default().max_duration_ms, None);
    }

    #[test]
    fn path_never_intersects_a_real_obstacle() {
        let start = Vec3::new(-5.0, 0.0, 0.0);
        let goal = Vec3::new(5.0, 0.0, 0.0);
        // A wall of obstacles directly between start and goal, forcing
        // the planner to route around it - not just find a lucky
        // straight line.
        let obstacles = vec![
            Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.5),
            Obstacle::new(Vec3::new(0.0, 2.0, 0.0), 1.5),
            Obstacle::new(Vec3::new(0.0, -2.0, 0.0), 1.5),
        ];
        let config = PlannerConfig {
            max_iterations: 20_000,
            ..PlannerConfig::default()
        };
        let path = plan(start, goal, &obstacles, open_workspace(), config, 7)
            .expect("a path must exist around the wall within workspace bounds");

        for window in path.windows(2) {
            for o in &obstacles {
                assert!(
                    !o.intersects_segment(window[0], window[1], config.robot_radius),
                    "path segment {:?}->{:?} clips obstacle at {:?}",
                    window[0],
                    window[1],
                    o.center
                );
            }
        }
    }

    #[test]
    fn same_seed_and_scenario_is_fully_deterministic() {
        let start = Vec3::new(-5.0, 0.0, 0.0);
        let goal = Vec3::new(5.0, 0.0, 0.0);
        let obstacles = vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.0)];
        let config = PlannerConfig::default();

        let path_a = plan(start, goal, &obstacles, open_workspace(), config, 99).unwrap();
        let path_b = plan(start, goal, &obstacles, open_workspace(), config, 99).unwrap();
        assert_eq!(path_a, path_b);
    }

    #[test]
    fn start_inside_obstacle_is_rejected_before_searching() {
        let obstacles = vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 2.0)];
        let result = plan(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            &obstacles,
            open_workspace(),
            PlannerConfig::default(),
            1,
        );
        assert_eq!(result, Err(PlanError::StartInsideObstacle));
    }

    #[test]
    fn goal_outside_workspace_is_rejected_before_searching() {
        let result = plan(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(999.0, 0.0, 0.0),
            &[],
            open_workspace(),
            PlannerConfig::default(),
            1,
        );
        assert_eq!(result, Err(PlanError::GoalOutsideWorkspace));
    }

    #[test]
    fn a_workspace_spanning_wall_honestly_reports_no_path() {
        // See corpus::workspace_spanning_wall's own doc comment for why
        // every continuous path is mathematically forced to cross the
        // obstacle here - there is truly no path, not just one the
        // search failed to find in time.
        let (workspace, obstacles) = corpus::workspace_spanning_wall();
        let config = PlannerConfig {
            max_iterations: 3000,
            ..PlannerConfig::default()
        };
        let result = plan(
            corpus::start(),
            corpus::goal(),
            &obstacles,
            workspace,
            config,
            3,
        );
        assert_eq!(result, Err(PlanError::NoPathFound));
    }

    #[test]
    fn a_workspace_spanning_wall_with_no_time_limit_still_honestly_reports_no_path() {
        // A tight time limit must not turn a genuine "no path exists"
        // into a misleading NoPathFound-that-was-really-a-timeout, or
        // vice versa - a real search over the same scene, this time
        // bounded by wall-clock time instead of iteration count, must
        // still reach NoPathFound (the search space here is small
        // enough to exhaust well within the limit).
        let (workspace, obstacles) = corpus::workspace_spanning_wall();
        let config = PlannerConfig {
            max_iterations: 3000,
            max_duration_ms: Some(5_000),
            ..PlannerConfig::default()
        };
        let result = plan(
            corpus::start(),
            corpus::goal(),
            &obstacles,
            workspace,
            config,
            3,
        );
        assert_eq!(result, Err(PlanError::NoPathFound));
    }

    #[test]
    fn the_body_not_only_the_centre_must_fit_inside_the_workspace() {
        let workspace = Workspace {
            min: Vec3::new(-0.9, -0.9, -0.9),
            max: Vec3::new(0.9, 0.9, 0.9),
        };
        let config = PlannerConfig {
            robot_radius: 0.5,
            ..PlannerConfig::default()
        };
        let result = plan(
            Vec3::new(-0.85, 0.0, 0.0),
            Vec3::new(0.85, 0.0, 0.0),
            &[],
            workspace,
            config,
            1,
        );
        assert_eq!(result, Err(PlanError::StartOutsideWorkspace));
        let wall_to_wall = Workspace {
            min: Vec3::new(-0.4, -5.0, -5.0),
            max: Vec3::new(0.4, 5.0, 5.0),
        };
        assert_eq!(
            plan(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                &[],
                wall_to_wall,
                config,
                1
            ),
            Err(PlanError::InvalidInput(
                crate::semantics::InvalidScenarioReason::WorkspaceNarrowerThanRobot
            ))
        );
    }

    #[test]
    fn a_sampled_goal_does_not_end_the_path_with_a_zero_length_segment() {
        let config = PlannerConfig {
            goal_bias: 1.0,
            step_size: 0.5,
            goal_threshold: 0.5,
            ..PlannerConfig::default()
        };
        let path = plan(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.3, 0.0, 0.0),
            &[],
            open_workspace(),
            config,
            1,
        )
        .unwrap();
        for pair in path.windows(2) {
            assert!(
                pair[0].distance(pair[1]) > 0.0,
                "path {path:?} repeats a waypoint"
            );
        }
        assert_eq!(path.last().copied(), Some(Vec3::new(0.3, 0.0, 0.0)));
    }

    #[test]
    fn a_start_that_is_the_goal_is_a_path_of_one_waypoint() {
        let p = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(
            plan(p, p, &[], open_workspace(), PlannerConfig::default(), 1),
            Ok(vec![p])
        );
    }

    #[test]
    fn an_absurd_iteration_count_is_refused_instead_of_running_for_hours() {
        let config = PlannerConfig {
            max_iterations: u32::MAX,
            ..PlannerConfig::default()
        };
        assert_eq!(
            plan(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(5.0, 0.0, 0.0),
                &[],
                open_workspace(),
                config,
                1
            ),
            Err(PlanError::InvalidInput(
                crate::semantics::InvalidScenarioReason::TooManyIterations
            ))
        );
    }

    fn path_length(path: &[Vec3]) -> f64 {
        path.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    fn optimizing(extra: u32) -> PlannerConfig {
        PlannerConfig {
            max_iterations: 20_000,
            optimize_iterations: extra,
            rewire_radius: 2.0,
            ..PlannerConfig::default()
        }
    }

    #[test]
    fn a_crowded_scene_is_planned_through_the_hierarchy_and_stays_clear_by_a_full_scan() {
        let mut rng = Xorshift64Star::new(21);
        let obstacles: Vec<Obstacle> = (0..400)
            .map(|_| {
                Obstacle::new(
                    Vec3::new(
                        rng.next_range(-9.0, 9.0),
                        rng.next_range(-9.0, 9.0),
                        rng.next_range(-9.0, 9.0),
                    ),
                    0.4,
                )
            })
            .collect();
        let start = Vec3::new(-9.5, -9.5, -9.5);
        let goal = Vec3::new(9.5, 9.5, 9.5);
        let config = PlannerConfig {
            max_iterations: 60_000,
            ..PlannerConfig::default()
        };
        let path = plan(start, goal, &obstacles, open_workspace(), config, 4)
            .expect("a scattered scene still has a way through");
        assert_eq!(path.first().copied(), Some(start));
        assert_eq!(path.last().copied(), Some(goal));
        for pair in path.windows(2) {
            assert!(!obstacles.iter().any(|o| o.intersects_segment(
                pair[0],
                pair[1],
                config.robot_radius
            )));
        }
    }

    #[test]
    fn rrt_star_is_off_by_default() {
        let config = PlannerConfig::default();
        assert_eq!(config.optimize_iterations, 0);
        let from_json: PlannerConfig = serde_json::from_str("{}").unwrap();
        assert_eq!(from_json.optimize_iterations, 0);
    }

    #[test]
    fn rrt_star_never_returns_a_longer_path_than_the_plain_search_for_the_same_seed() {
        let start = Vec3::new(-8.0, -8.0, 0.0);
        let goal = Vec3::new(8.0, 8.0, 0.0);
        let mut shorter = 0;
        for seed in 1..=8u64 {
            let plain = plan(
                start,
                goal,
                &[],
                open_workspace(),
                PlannerConfig {
                    max_iterations: 20_000,
                    ..PlannerConfig::default()
                },
                seed,
            )
            .expect("an open workspace has a path");
            let star = plan(start, goal, &[], open_workspace(), optimizing(1500), seed)
                .expect("an open workspace has a path");
            assert_eq!(star.first().copied(), Some(start));
            assert_eq!(star.last().copied(), Some(goal));
            let (a, b) = (path_length(&plain), path_length(&star));
            assert!(b <= a + 1e-9, "seed {seed}: {b} is longer than {a}");
            if b + 1e-6 < a {
                shorter += 1;
            }
        }
        assert!(shorter >= 1, "the rewiring never shortened a single path");
    }

    #[test]
    fn rrt_star_paths_stay_clear_of_the_obstacles() {
        let obstacles = corpus::single_blocking_obstacle();
        let config = PlannerConfig {
            max_iterations: 20_000,
            ..optimizing(800)
        };
        for seed in 1..=4u64 {
            let path = plan(
                corpus::start(),
                corpus::goal(),
                &obstacles,
                corpus::open_workspace(),
                config,
                seed,
            )
            .expect("one obstacle leaves a way round");
            for pair in path.windows(2) {
                assert!(!obstacles.iter().any(|o| o.intersects_segment(
                    pair[0],
                    pair[1],
                    config.robot_radius
                )));
            }
        }
    }

    #[test]
    fn rrt_star_is_deterministic_for_a_seed() {
        let start = Vec3::new(-5.0, 0.0, 0.0);
        let goal = Vec3::new(5.0, 3.0, 0.0);
        let a = plan(start, goal, &[], open_workspace(), optimizing(500), 11).unwrap();
        let b = plan(start, goal, &[], open_workspace(), optimizing(500), 11).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn rrt_star_still_reports_no_path_behind_a_wall() {
        let (workspace, wall) = corpus::workspace_spanning_wall();
        let config = PlannerConfig {
            max_iterations: 300,
            ..optimizing(100)
        };
        assert_eq!(
            plan(corpus::start(), corpus::goal(), &wall, workspace, config, 1),
            Err(PlanError::NoPathFound)
        );
    }

    #[test]
    fn rrt_star_with_a_zero_time_limit_reports_the_time_limit() {
        let config = PlannerConfig {
            max_duration_ms: Some(0),
            ..optimizing(100)
        };
        assert_eq!(
            plan(
                corpus::start(),
                corpus::goal(),
                &[],
                corpus::open_workspace(),
                config,
                1
            ),
            Err(PlanError::TimeLimitExceeded)
        );
    }

    #[test]
    fn rrt_star_refuses_a_rewire_radius_that_is_not_a_distance() {
        for radius in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let config = PlannerConfig {
                rewire_radius: radius,
                ..optimizing(10)
            };
            assert!(matches!(
                plan(
                    corpus::start(),
                    corpus::goal(),
                    &[],
                    corpus::open_workspace(),
                    config,
                    1
                ),
                Err(PlanError::InvalidInput(_))
            ));
        }
        // the radius is not read when the plain search is used
        let plain = PlannerConfig {
            rewire_radius: -1.0,
            ..PlannerConfig::default()
        };
        assert!(plan(
            corpus::start(),
            corpus::goal(),
            &[],
            corpus::open_workspace(),
            plain,
            1
        )
        .is_ok());
    }

    #[test]
    fn rrt_star_refuses_more_extra_iterations_than_the_limit() {
        let config = PlannerConfig {
            optimize_iterations: u32::MAX,
            ..PlannerConfig::default()
        };
        assert_eq!(
            plan(
                corpus::start(),
                corpus::goal(),
                &[],
                corpus::open_workspace(),
                config,
                1
            ),
            Err(PlanError::InvalidInput(
                crate::semantics::InvalidScenarioReason::TooManyIterations
            ))
        );
    }

    #[test]
    fn a_partial_config_takes_the_defaults_for_what_it_leaves_out() {
        let config: PlannerConfig = serde_json::from_str(r#"{"robot_radius": 0.3}"#).unwrap();
        assert_eq!(config.robot_radius, 0.3);
        assert_eq!(
            config.max_iterations,
            PlannerConfig::default().max_iterations
        );
        assert_eq!(config.step_size, PlannerConfig::default().step_size);
    }

    #[test]
    fn a_long_straight_corridor_with_a_full_goal_bias_finishes() {
        // 400 m at a 1 cm step is 40,000 nodes along one axis: it overflowed the stack
        let workspace = Workspace {
            min: Vec3::new(-1.0, -1.0, -1.0),
            max: Vec3::new(401.0, 1.0, 1.0),
        };
        let config = PlannerConfig {
            step_size: 0.01,
            goal_bias: 1.0,
            goal_threshold: 0.01,
            max_iterations: 100_000,
            ..PlannerConfig::default()
        };
        let path = plan(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(400.0, 0.0, 0.0),
            &[],
            workspace,
            config,
            1,
        )
        .expect("a straight corridor has a path");
        assert_eq!(path.last().copied(), Some(Vec3::new(400.0, 0.0, 0.0)));
    }
}

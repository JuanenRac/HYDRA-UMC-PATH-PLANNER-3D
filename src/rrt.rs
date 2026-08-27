// HYDRA-UMC-PATH-PLANNER-3D - rrt.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// The real collision-free path search: a standard RRT (Rapidly-exploring
// Random Tree), not yet RRT* (no rewiring pass for path optimality) and
// not yet multi-robot-synchronized (plans one agent through a static
// obstacle set, does not coordinate several agents' paths against each
// other). Both are real, scoped-out future work - see
// mejoras_futuras.txt for why proving a single-agent planner correct
// came first. Nearest-neighbor lookup is a linear scan over the tree,
// not a KD-tree - fine at the tree sizes a single planning call produces
// today (hundreds to a few thousand nodes), and simpler to verify
// correct than a KD-tree would have been for this first pass.

use crate::geometry::Vec3;
use crate::obstacle::Obstacle;
use crate::rng::Xorshift64Star;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Workspace {
    pub min: Vec3,
    pub max: Vec3,
}

impl Workspace {
    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    fn sample(&self, rng: &mut Xorshift64Star) -> Vec3 {
        Vec3::new(
            rng.next_range(self.min.x, self.max.x),
            rng.next_range(self.min.y, self.max.y),
            rng.next_range(self.min.z, self.max.z),
        )
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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
}

impl Default for PlannerConfig {
    fn default() -> Self {
        PlannerConfig {
            max_iterations: 5000,
            step_size: 0.5,
            goal_bias: 0.05,
            goal_threshold: 0.5,
            robot_radius: 0.1,
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
    if !workspace.contains(start) {
        return Err(PlanError::StartOutsideWorkspace);
    }
    if !workspace.contains(goal) {
        return Err(PlanError::GoalOutsideWorkspace);
    }
    if obstacles
        .iter()
        .any(|o| o.contains_point(start, config.robot_radius))
    {
        return Err(PlanError::StartInsideObstacle);
    }
    if obstacles
        .iter()
        .any(|o| o.contains_point(goal, config.robot_radius))
    {
        return Err(PlanError::GoalInsideObstacle);
    }

    let mut rng = Xorshift64Star::new(seed);
    let mut tree = vec![TreeNode {
        point: start,
        parent: None,
    }];

    let is_clear = |a: Vec3, b: Vec3| -> bool {
        !obstacles
            .iter()
            .any(|o| o.intersects_segment(a, b, config.robot_radius))
    };

    for _ in 0..config.max_iterations {
        let sample = if rng.next_f64() < config.goal_bias {
            goal
        } else {
            workspace.sample(&mut rng)
        };

        let nearest_idx = nearest(&tree, sample);
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

        if new_point.distance(goal) <= config.goal_threshold && is_clear(new_point, goal) {
            tree.push(TreeNode {
                point: goal,
                parent: Some(new_idx),
            });
            return Ok(reconstruct_path(&tree, tree.len() - 1));
        }
    }

    Err(PlanError::NoPathFound)
}

fn nearest(tree: &[TreeNode], target: Vec3) -> usize {
    tree.iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.point
                .distance(target)
                .partial_cmp(&b.point.distance(target))
                .expect("distance() never produces NaN for finite inputs")
        })
        .map(|(idx, _)| idx)
        .expect("tree always has at least the start node")
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
        // A single sphere at the origin, large enough that its y/z
        // cross-section at x=0 (radius 3.0) fully covers the workspace's
        // own y/z bounds (+-2.0, worst-case corner distance from the
        // origin is sqrt(2^2+2^2) = 2.83 < 3.0). Since start and goal
        // sit on opposite sides of x=0 and the workspace itself forbids
        // going around in y or z, every continuous path is mathematically
        // forced to cross a blocked point - there is truly no path, not
        // just one the search failed to find in time.
        let start = Vec3::new(-5.0, 0.0, 0.0);
        let goal = Vec3::new(5.0, 0.0, 0.0);
        let workspace = Workspace {
            min: Vec3::new(-8.0, -2.0, -2.0),
            max: Vec3::new(8.0, 2.0, 2.0),
        };
        let obstacles = vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 3.0)];
        let config = PlannerConfig {
            max_iterations: 3000,
            ..PlannerConfig::default()
        };
        let result = plan(start, goal, &obstacles, workspace, config, 3);
        assert_eq!(result, Err(PlanError::NoPathFound));
    }
}

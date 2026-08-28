// HYDRA-UMC-PATH-PLANNER-3D - validate.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// Real safety validation for an ALREADY-COMPUTED path. rrt::plan() only
// ever returns paths it built collision-free by construction (every
// edge is checked with is_clear() before being added to the tree), so
// there is no gap to close there - but a path handed to THIS validator
// might come from anywhere else: a cached/replayed plan, a path relayed
// from another process over the network, a hand-edited scenario file.
// The obstacle set or workspace it was computed against may have since
// changed too (a safety zone moved, a new static obstacle appeared).
// This re-checks a path against the CURRENT scenario before it gets
// handed to a robot for real execution - a fail-safe gate, not a
// second path planner.

use crate::geometry::Vec3;
use crate::obstacle::Obstacle;
use crate::rrt::Workspace;
use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum PathSafetyIssue {
    /// The path has zero waypoints - nothing to validate, and nothing a
    /// robot could execute either.
    EmptyPath,
    WaypointOutsideWorkspace { index: usize, point: Vec3 },
    WaypointInsideObstacle { index: usize, point: Vec3 },
    /// The straight-line segment between two consecutive waypoints
    /// comes within `robot_radius` of an obstacle, even if both
    /// waypoints themselves are individually clear - the case a
    /// per-waypoint-only check would miss entirely.
    SegmentIntersectsObstacle { from_index: usize, to_index: usize },
}

/// Checks every waypoint in `path` against `workspace` and every
/// obstacle in `obstacles` (inflated by `robot_radius`), and every
/// segment between consecutive waypoints for a collision a
/// waypoint-only check would miss. Returns every issue found, not just
/// the first - a caller rejecting an unsafe path can report the full
/// extent of why, not just that something was wrong.
pub fn validate_path(
    path: &[Vec3],
    obstacles: &[Obstacle],
    workspace: &Workspace,
    robot_radius: f64,
) -> Vec<PathSafetyIssue> {
    if path.is_empty() {
        return vec![PathSafetyIssue::EmptyPath];
    }

    let mut issues = Vec::new();

    for (index, &point) in path.iter().enumerate() {
        if !workspace.contains(point) {
            issues.push(PathSafetyIssue::WaypointOutsideWorkspace { index, point });
        }
        if obstacles
            .iter()
            .any(|o| o.contains_point(point, robot_radius))
        {
            issues.push(PathSafetyIssue::WaypointInsideObstacle { index, point });
        }
    }

    for (from_index, window) in path.windows(2).enumerate() {
        let (a, b) = (window[0], window[1]);
        if obstacles
            .iter()
            .any(|o| o.intersects_segment(a, b, robot_radius))
        {
            issues.push(PathSafetyIssue::SegmentIntersectsObstacle {
                from_index,
                to_index: from_index + 1,
            });
        }
    }

    issues
}

/// Convenience wrapper: true only if `validate_path` found nothing. Not
/// called from the CLI today (`run_validate` in main.rs wants the full
/// issue list to report) - kept as the real public API for a future
/// caller (e.g. HYDRA-UMC-JOB-DISPATCHER, once it integrates directly
/// against this crate) that only needs a yes/no answer.
#[allow(dead_code)]
pub fn is_path_safe(
    path: &[Vec3],
    obstacles: &[Obstacle],
    workspace: &Workspace,
    robot_radius: f64,
) -> bool {
    validate_path(path, obstacles, workspace, robot_radius).is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corpus;
    use crate::rrt::plan;

    #[test]
    fn a_real_planner_output_always_validates_as_safe() {
        let obstacles = corpus::wall_of_obstacles();
        let workspace = corpus::open_workspace();
        let config = crate::rrt::PlannerConfig {
            max_iterations: 20_000,
            ..Default::default()
        };
        let path = plan(corpus::start(), corpus::goal(), &obstacles, workspace, config, 7)
            .expect("a path must exist around the wall");

        let issues = validate_path(&path, &obstacles, &workspace, config.robot_radius);
        assert!(issues.is_empty(), "planner's own output flagged unsafe: {issues:?}");
    }

    #[test]
    fn empty_path_is_rejected() {
        let issues = validate_path(&[], &corpus::no_obstacles(), &corpus::open_workspace(), 0.1);
        assert_eq!(issues, vec![PathSafetyIssue::EmptyPath]);
    }

    #[test]
    fn waypoint_outside_workspace_is_flagged() {
        let workspace = corpus::open_workspace();
        let path = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(999.0, 0.0, 0.0)];
        let issues = validate_path(&path, &corpus::no_obstacles(), &workspace, 0.0);
        assert_eq!(
            issues,
            vec![PathSafetyIssue::WaypointOutsideWorkspace {
                index: 1,
                point: Vec3::new(999.0, 0.0, 0.0)
            }]
        );
    }

    #[test]
    fn waypoint_landing_inside_an_obstacle_is_flagged() {
        let obstacles = corpus::single_blocking_obstacle();
        let path = vec![corpus::start(), Vec3::new(0.0, 0.0, 0.0), corpus::goal()];
        let issues = validate_path(&path, &obstacles, &corpus::open_workspace(), 0.0);
        assert!(issues
            .iter()
            .any(|i| matches!(i, PathSafetyIssue::WaypointInsideObstacle { index: 1, .. })));
    }

    #[test]
    fn a_straight_line_clipping_an_obstacle_between_two_clear_waypoints_is_flagged() {
        // Both waypoints sit outside the sphere at the origin, but the
        // straight segment between them passes right through it - a
        // waypoint-only check would miss this entirely.
        let obstacles = corpus::single_blocking_obstacle();
        let path = vec![corpus::start(), corpus::goal()];
        let issues = validate_path(&path, &obstacles, &corpus::open_workspace(), 0.0);
        assert_eq!(
            issues,
            vec![PathSafetyIssue::SegmentIntersectsObstacle {
                from_index: 0,
                to_index: 1
            }]
        );
    }

    #[test]
    fn is_path_safe_matches_validate_path_emptiness() {
        let workspace = corpus::open_workspace();
        let obstacles = corpus::no_obstacles();
        let safe_path = vec![corpus::start(), corpus::goal()];
        assert!(is_path_safe(&safe_path, &obstacles, &workspace, 0.1));

        let unsafe_path = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(999.0, 0.0, 0.0)];
        assert!(!is_path_safe(&unsafe_path, &obstacles, &workspace, 0.1));
    }
}

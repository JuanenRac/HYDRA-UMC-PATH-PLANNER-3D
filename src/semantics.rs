// HYDRA-UMC-PATH-PLANNER-3D - semantics.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// PATH-01 (P0):
// obstacle.rs's own contains_point()/intersects_segment() compute an
// EFFECTIVE collision radius as `self.radius + clearance` - a negative
// robot_radius large enough in magnitude (e.g. -2 against an obstacle
// radius of 1) makes that effective radius negative, which no real,
// non-negative distance can ever satisfy. Collision detection is not
// merely weakened by a bad radius, it is silently DEFEATED: `validate`
// reported `status: "safe"` for a straight segment that visibly crosses
// the obstacle's own sphere, because `distance <= -1.0` is always false.
//
// This module is the shared semantic-validity gate both rrt::plan() and
// validate::validate_path() now run BEFORE any geometry check - a scene
// with a negative/non-finite radius, an inverted workspace bound, or an
// out-of-range planner parameter is rejected explicitly, with a real,
// structured reason, instead of silently producing a false "safe" or
// "no path found" verdict that looks like an honest planning outcome.

use crate::geometry::Vec3;
use crate::obstacle::Obstacle;
use crate::rrt::{PlannerConfig, Workspace};
use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum InvalidScenarioReason {
    NonFiniteRobotRadius,
    NegativeRobotRadius,
    NonFiniteObstacle { index: usize },
    NegativeObstacleRadius { index: usize },
    NonFiniteWorkspaceBound,
    InvertedWorkspaceBound,
    NonFinitePoint,
    NonFiniteStepSize,
    NonPositiveStepSize,
    NonFiniteGoalBias,
    GoalBiasOutOfRange,
    NonFiniteGoalThreshold,
    NegativeGoalThreshold,
    ZeroMaxIterations,
}

/// Validates a bare robot/clearance radius on its own - the exact value
/// `Obstacle::contains_point`/`intersects_segment` add onto an
/// obstacle's own radius. Used both standalone (`validate_path`, which
/// only ever receives a bare `f64`, not a full `PlannerConfig`) and as
/// part of `validate_planner_config` below.
pub fn validate_robot_radius(robot_radius: f64) -> Result<(), InvalidScenarioReason> {
    if !robot_radius.is_finite() {
        return Err(InvalidScenarioReason::NonFiniteRobotRadius);
    }
    if robot_radius < 0.0 {
        return Err(InvalidScenarioReason::NegativeRobotRadius);
    }
    Ok(())
}

/// Every obstacle's own center and radius must be finite, and radius
/// must never be negative - a negative obstacle radius would shrink the
/// effective collision sphere the same way a negative robot_radius does,
/// just from the other side of the same addition.
pub fn validate_obstacles(obstacles: &[Obstacle]) -> Result<(), InvalidScenarioReason> {
    for (index, o) in obstacles.iter().enumerate() {
        if !o.center.is_finite() || !o.radius.is_finite() {
            return Err(InvalidScenarioReason::NonFiniteObstacle { index });
        }
        if o.radius < 0.0 {
            return Err(InvalidScenarioReason::NegativeObstacleRadius { index });
        }
    }
    Ok(())
}

/// A workspace whose bounds are non-finite or inverted (`min > max` on
/// any axis) has no real, well-defined volume - `Workspace::contains`
/// would otherwise reject every point unconditionally on an inverted
/// axis, which looks identical to "start/goal outside workspace" and
/// hides the real, distinct cause.
pub fn validate_workspace(workspace: &Workspace) -> Result<(), InvalidScenarioReason> {
    if !workspace.min.is_finite() || !workspace.max.is_finite() {
        return Err(InvalidScenarioReason::NonFiniteWorkspaceBound);
    }
    if workspace.min.x > workspace.max.x
        || workspace.min.y > workspace.max.y
        || workspace.min.z > workspace.max.z
    {
        return Err(InvalidScenarioReason::InvertedWorkspaceBound);
    }
    Ok(())
}

/// A waypoint/start/goal coordinate must be finite before it means
/// anything geometrically - `validate_path` already checks this
/// per-waypoint (see `PathSafetyIssue::NonFiniteWaypoint`); `plan()`
/// uses this for its own `start`/`goal` arguments up front.
pub fn validate_point(point: Vec3) -> Result<(), InvalidScenarioReason> {
    if !point.is_finite() {
        return Err(InvalidScenarioReason::NonFinitePoint);
    }
    Ok(())
}

/// The full set of planner-specific numeric parameters `rrt::plan` needs
/// sane values for on top of the shared robot_radius/obstacle/workspace
/// checks above: a non-positive step_size would either loop forever
/// (zero) or never converge meaningfully; an out-of-range goal_bias
/// (outside `[0.0, 1.0]`) is not a real probability; a negative
/// goal_threshold can never be satisfied by a real, non-negative
/// distance, the same class of bug as the robot_radius one above.
pub fn validate_planner_config(config: &PlannerConfig) -> Result<(), InvalidScenarioReason> {
    validate_robot_radius(config.robot_radius)?;
    if !config.step_size.is_finite() {
        return Err(InvalidScenarioReason::NonFiniteStepSize);
    }
    if config.step_size <= 0.0 {
        return Err(InvalidScenarioReason::NonPositiveStepSize);
    }
    if !config.goal_bias.is_finite() {
        return Err(InvalidScenarioReason::NonFiniteGoalBias);
    }
    if !(0.0..=1.0).contains(&config.goal_bias) {
        return Err(InvalidScenarioReason::GoalBiasOutOfRange);
    }
    if !config.goal_threshold.is_finite() {
        return Err(InvalidScenarioReason::NonFiniteGoalThreshold);
    }
    if config.goal_threshold < 0.0 {
        return Err(InvalidScenarioReason::NegativeGoalThreshold);
    }
    if config.max_iterations == 0 {
        return Err(InvalidScenarioReason::ZeroMaxIterations);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn negative_robot_radius_is_rejected() {
        assert_eq!(
            validate_robot_radius(-2.0),
            Err(InvalidScenarioReason::NegativeRobotRadius)
        );
    }

    #[test]
    fn non_finite_robot_radius_is_rejected() {
        assert_eq!(
            validate_robot_radius(f64::NAN),
            Err(InvalidScenarioReason::NonFiniteRobotRadius)
        );
        assert_eq!(
            validate_robot_radius(f64::INFINITY),
            Err(InvalidScenarioReason::NonFiniteRobotRadius)
        );
    }

    #[test]
    fn zero_robot_radius_is_a_real_valid_value() {
        assert_eq!(validate_robot_radius(0.0), Ok(()));
    }

    #[test]
    fn negative_obstacle_radius_is_rejected() {
        let obstacles = vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), -1.0)];
        assert_eq!(
            validate_obstacles(&obstacles),
            Err(InvalidScenarioReason::NegativeObstacleRadius { index: 0 })
        );
    }

    #[test]
    fn inverted_workspace_bound_is_rejected() {
        let workspace = Workspace {
            min: Vec3::new(10.0, 0.0, 0.0),
            max: Vec3::new(-10.0, 0.0, 0.0),
        };
        assert_eq!(
            validate_workspace(&workspace),
            Err(InvalidScenarioReason::InvertedWorkspaceBound)
        );
    }

    #[test]
    fn non_positive_step_size_is_rejected() {
        let config = PlannerConfig {
            step_size: 0.0,
            ..Default::default()
        };
        assert_eq!(
            validate_planner_config(&config),
            Err(InvalidScenarioReason::NonPositiveStepSize)
        );
    }

    #[test]
    fn goal_bias_out_of_range_is_rejected() {
        let config = PlannerConfig {
            goal_bias: 1.5,
            ..Default::default()
        };
        assert_eq!(
            validate_planner_config(&config),
            Err(InvalidScenarioReason::GoalBiasOutOfRange)
        );
    }

    #[test]
    fn default_planner_config_is_valid() {
        assert_eq!(validate_planner_config(&PlannerConfig::default()), Ok(()));
    }
}

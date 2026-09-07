// HYDRA-UMC-PATH-PLANNER-3D - corpus.rs (test-only)
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// A reusable corpus of obstacle/workspace scenarios shared by rrt.rs's
// and validate.rs's own tests - one canonical source of "open
// workspace", "single blocking obstacle", "wall forcing a detour", and
// "workspace-spanning wall with truly no path" fixtures, instead of
// each test module re-deriving its own ad-hoc obstacle placement. Only
// compiled into test builds (see the `#[cfg(test)] mod corpus;`
// declaration in main.rs) so none of this exists in a release binary.

use crate::geometry::Vec3;
use crate::obstacle::Obstacle;
use crate::rrt::Workspace;

pub fn open_workspace() -> Workspace {
    Workspace {
        min: Vec3::new(-10.0, -10.0, -10.0),
        max: Vec3::new(10.0, 10.0, 10.0),
    }
}

pub fn no_obstacles() -> Vec<Obstacle> {
    Vec::new()
}

pub fn single_blocking_obstacle() -> Vec<Obstacle> {
    vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.0)]
}

/// A wall of three spheres directly between `start()` and `goal()`,
/// forcing a real detour rather than a lucky straight line.
pub fn wall_of_obstacles() -> Vec<Obstacle> {
    vec![
        Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.5),
        Obstacle::new(Vec3::new(0.0, 2.0, 0.0), 1.5),
        Obstacle::new(Vec3::new(0.0, -2.0, 0.0), 1.5),
    ]
}

/// A workspace + single sphere combination where every continuous path
/// from `start()` to `goal()` is mathematically forced to cross the
/// obstacle: the sphere's radius (3.0) exceeds the workspace's own
/// worst-case y/z corner distance from the origin (sqrt(2^2+2^2) =
/// 2.83), and start/goal sit on opposite sides of x=0 with no way
/// around in y or z. There is truly no path here, not just one a
/// search failed to find in time.
pub fn workspace_spanning_wall() -> (Workspace, Vec<Obstacle>) {
    let workspace = Workspace {
        min: Vec3::new(-8.0, -2.0, -2.0),
        max: Vec3::new(8.0, 2.0, 2.0),
    };
    let obstacles = vec![Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 3.0)];
    (workspace, obstacles)
}

pub fn start() -> Vec3 {
    Vec3::new(-5.0, 0.0, 0.0)
}

pub fn goal() -> Vec3 {
    Vec3::new(5.0, 0.0, 0.0)
}

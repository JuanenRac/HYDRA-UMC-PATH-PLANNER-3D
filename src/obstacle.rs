// HYDRA-UMC-PATH-PLANNER-3D - obstacle.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// Obstacles are spheres - the simplest 3D collision primitive that is
// still real and correct (not a bounding-box stand-in for something
// else). An octree/BVH for large obstacle counts is real future work
// (see mejoras_futuras.txt) once there's a scene with enough obstacles
// for brute-force checking to actually matter - today's swarm cell is a
// handful of arms and static safety zones, not thousands of colliders.

use crate::geometry::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Obstacle {
    pub center: Vec3,
    pub radius: f64,
}

impl Obstacle {
    // Only called from tests today (main.rs builds `Obstacle` values via
    // serde's derived `Deserialize` from a scenario file, not this
    // constructor) - kept as the real public API for anything that
    // builds scenarios in code rather than JSON, e.g. a future test
    // harness or the CLI's own scenario generator.
    #[allow(dead_code)]
    pub fn new(center: Vec3, radius: f64) -> Self {
        Obstacle { center, radius }
    }

    /// True if `point` is inside this obstacle, inflated by
    /// `clearance` (typically the planning robot's own radius).
    pub fn contains_point(&self, point: Vec3, clearance: f64) -> bool {
        self.center.distance(point) <= self.radius + clearance
    }

    /// True if the line segment `a`-`b` comes within `clearance` of this
    /// obstacle at any point along it, using the closest point on the
    /// segment to the sphere's center (standard point-to-segment
    /// projection, clamped to the segment's own extent).
    pub fn intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        let ab = b.sub(a);
        let ab_len_sq = ab.dot(ab);
        let closest = if ab_len_sq < 1e-12 {
            // Degenerate (zero-length) segment - the closest point is
            // just `a` itself.
            a
        } else {
            let t = (self.center.sub(a).dot(ab) / ab_len_sq).clamp(0.0, 1.0);
            a.add(ab.scale(t))
        };
        self.contains_point(closest, clearance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contains_point_respects_clearance() {
        let o = Obstacle::new(Vec3::new(0.0, 0.0, 0.0), 1.0);
        assert!(o.contains_point(Vec3::new(0.5, 0.0, 0.0), 0.0));
        assert!(!o.contains_point(Vec3::new(2.0, 0.0, 0.0), 0.0));
        // Just outside the sphere, but within clearance of a robot with
        // radius 1.5 (1.0 + 1.5 = 2.5 >= distance 2.0).
        assert!(o.contains_point(Vec3::new(2.0, 0.0, 0.0), 1.5));
    }

    #[test]
    fn segment_passing_through_sphere_is_detected() {
        let o = Obstacle::new(Vec3::new(5.0, 0.0, 0.0), 1.0);
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(10.0, 0.0, 0.0);
        assert!(o.intersects_segment(a, b, 0.0));
    }

    #[test]
    fn segment_missing_sphere_entirely_is_not_detected() {
        let o = Obstacle::new(Vec3::new(5.0, 10.0, 0.0), 1.0);
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(10.0, 0.0, 0.0);
        assert!(!o.intersects_segment(a, b, 0.0));
    }

    #[test]
    fn segment_endpoint_projection_is_clamped_not_extrapolated() {
        // The sphere is "behind" point a relative to the segment
        // direction - the closest point on the FULL LINE would be
        // outside [a,b], so this must clamp to `a` and correctly report
        // no intersection given the sphere is far from `a` too.
        let o = Obstacle::new(Vec3::new(-5.0, 0.0, 0.0), 1.0);
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(10.0, 0.0, 0.0);
        assert!(!o.intersects_segment(a, b, 0.0));
    }
}

// HYDRA-UMC-PATH-PLANNER-3D - geometry.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// Minimal 3D vector math - only what the RRT planner and its collision
// checks actually need. Deliberately not pulling in a general-purpose
// linear algebra crate (nalgebra/glam) for 6 operations on a 3-tuple.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    pub fn sub(self, other: Vec3) -> Vec3 {
        Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    pub fn add(self, other: Vec3) -> Vec3 {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    pub fn scale(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }

    pub fn dot(self, other: Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn distance(self, other: Vec3) -> f64 {
        self.sub(other).length()
    }

    /// Returns the zero vector if `self` is (numerically) the zero
    /// vector, rather than producing NaN - callers that would otherwise
    /// divide by a near-zero length (e.g. two RRT samples landing on
    /// the same point) get a defined, inert direction instead of a
    /// silently corrupted tree.
    pub fn normalized(self) -> Vec3 {
        let len = self.length();
        if len < 1e-9 {
            Vec3::new(0.0, 0.0, 0.0)
        } else {
            self.scale(1.0 / len)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_is_symmetric_and_correct() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(3.0, 4.0, 0.0);
        assert!((a.distance(b) - 5.0).abs() < 1e-9);
        assert!((b.distance(a) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn normalized_has_unit_length() {
        let v = Vec3::new(2.0, 0.0, 0.0);
        let n = v.normalized();
        assert!((n.length() - 1.0).abs() < 1e-9);
        assert_eq!(n, Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn normalized_zero_vector_stays_zero_not_nan() {
        let v = Vec3::new(0.0, 0.0, 0.0);
        let n = v.normalized();
        assert_eq!(n, Vec3::new(0.0, 0.0, 0.0));
    }
}

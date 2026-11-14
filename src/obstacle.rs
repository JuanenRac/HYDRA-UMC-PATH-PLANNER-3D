// HYDRA-UMC-PATH-PLANNER-3D - obstacle.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// Obstacles are spheres - the simplest 3D collision primitive that is
// still real and correct (not a bounding-box stand-in for something
// else). `ObstacleIndex` below is a bounding-volume hierarchy over the
// spheres, so a segment check on a scene with many obstacles visits only
// the few whose boxes the segment crosses; it answers exactly what the
// brute-force scan over `Obstacle::intersects_segment` answers (the leaves
// run that same function), and the independent safety validator in
// validate.rs keeps scanning every obstacle on purpose.

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

/// An axis-aligned box obstacle (a zone with flat sides, a shelf, a table): `min` and `max` are its opposite corners, in metres. The robot is a
/// sphere, so the box is inflated by the clearance on every side; the rounded corners the sphere would leave are counted as filled, which keeps
/// the check on the safe side.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoxObstacle {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoxObstacle {
    /// True if `point` is inside this box inflated by `clearance`.
    pub fn contains_point(&self, point: Vec3, clearance: f64) -> bool {
        point.x >= self.min.x - clearance
            && point.x <= self.max.x + clearance
            && point.y >= self.min.y - clearance
            && point.y <= self.max.y + clearance
            && point.z >= self.min.z - clearance
            && point.z <= self.max.z + clearance
    }

    /// True if the segment `a`-`b` touches this box inflated by `clearance` (the slab test: the interval of the segment that is inside the
    /// slab of each axis must not be empty).
    pub fn intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        let mut t_enter = 0.0f64;
        let mut t_leave = 1.0f64;
        let axes = [
            (
                a.x,
                b.x - a.x,
                self.min.x - clearance,
                self.max.x + clearance,
            ),
            (
                a.y,
                b.y - a.y,
                self.min.y - clearance,
                self.max.y + clearance,
            ),
            (
                a.z,
                b.z - a.z,
                self.min.z - clearance,
                self.max.z + clearance,
            ),
        ];
        for (start, delta, low, high) in axes {
            if delta.abs() < 1e-12 {
                if start < low || start > high {
                    return false;
                }
            } else {
                let mut t1 = (low - start) / delta;
                let mut t2 = (high - start) / delta;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                t_enter = t_enter.max(t1);
                t_leave = t_leave.min(t2);
                if t_enter > t_leave {
                    return false;
                }
            }
        }
        true
    }
}

/// Euler angles in degrees, as the zones files of HYDRA-UMC-SAFETY-ZONES write them: the local axes are rotated by `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
/// A missing angle is 0.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Rotation {
    #[serde(default)]
    pub roll: f64,
    #[serde(default)]
    pub pitch: f64,
    #[serde(default)]
    pub yaw: f64,
}

impl Rotation {
    /// The matrix of `R`, row by row (it takes a vector of the box frame to the world frame).
    fn matrix(&self) -> [[f64; 3]; 3] {
        let (sr, cr) = self.roll.to_radians().sin_cos();
        let (sp, cp) = self.pitch.to_radians().sin_cos();
        let (sy, cy) = self.yaw.to_radians().sin_cos();
        [
            [cy * cp, -sy * cr + cy * sp * sr, sy * sr + cy * sp * cr],
            [sy * cp, cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr],
            [-sp, cp * sr, cp * cr],
        ]
    }

    pub fn is_finite(&self) -> bool {
        self.roll.is_finite() && self.pitch.is_finite() && self.yaw.is_finite()
    }
}

/// An oriented box (a fence that is not aligned with the axes, a tilted table): its `center`, its `half_extents` (each at least 0, in metres) and its
/// `rotation` in degrees. Like the aligned box it is inflated by the clearance on every side, so the rounded corners the sphere would leave are counted
/// as filled and the check stays on the safe side.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ObbObstacle {
    pub center: Vec3,
    pub half_extents: Vec3,
    #[serde(default)]
    pub rotation_deg: Rotation,
}

impl ObbObstacle {
    /// `point` in the frame of the box (`R` transposed times the offset from the centre).
    fn in_local_frame(&self, point: Vec3, m: &[[f64; 3]; 3]) -> Vec3 {
        let d = point.sub(self.center);
        Vec3::new(
            m[0][0] * d.x + m[1][0] * d.y + m[2][0] * d.z,
            m[0][1] * d.x + m[1][1] * d.y + m[2][1] * d.z,
            m[0][2] * d.x + m[1][2] * d.y + m[2][2] * d.z,
        )
    }

    /// The same box in its own frame, centred on the origin: a box aligned with the axes.
    fn local_box(&self) -> BoxObstacle {
        BoxObstacle {
            min: self.half_extents.scale(-1.0),
            max: self.half_extents,
        }
    }

    pub fn contains_point(&self, point: Vec3, clearance: f64) -> bool {
        let m = self.rotation_deg.matrix();
        self.local_box()
            .contains_point(self.in_local_frame(point, &m), clearance)
    }

    /// True if the segment `a`-`b` touches this box inflated by `clearance`: the segment is taken to the frame of the box and the slab test of the
    /// aligned box is applied there (a rotation keeps the distances, so the inflation is the same).
    pub fn intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        let m = self.rotation_deg.matrix();
        self.local_box().intersects_segment(
            self.in_local_frame(a, &m),
            self.in_local_frame(b, &m),
            clearance,
        )
    }
}

/// A capsule (a link of a robot, a conveyor edge, a pole): every point within `radius` of the segment `start`-`end`. Equal ends make a sphere.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CapsuleObstacle {
    pub start: Vec3,
    pub end: Vec3,
    pub radius: f64,
}

impl CapsuleObstacle {
    pub fn contains_point(&self, point: Vec3, clearance: f64) -> bool {
        segment_distance(point, point, self.start, self.end) <= self.radius + clearance
    }

    /// True if the segment `a`-`b` comes within `radius + clearance` of the axis of the capsule (the distance between two segments).
    pub fn intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        segment_distance(a, b, self.start, self.end) <= self.radius + clearance
    }
}

/// The shortest distance between the segments `p1`-`q1` and `p2`-`q2` (a point is a segment of zero length; two parallel segments are handled).
fn segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> f64 {
    const EPS: f64 = 1e-12;
    let d1 = q1.sub(p1);
    let d2 = q2.sub(p2);
    let r = p1.sub(p2);
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    if a <= EPS && e <= EPS {
        return p1.distance(p2);
    }
    // the parameters of the closest points along each segment, both in 0..1
    let (s, t) = if a <= EPS {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= EPS {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let s0 = if denom > EPS {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t0 > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s0, t0)
            }
        }
    };
    p1.add(d1.scale(s)).distance(p2.add(d2.scale(t)))
}

/// A solid obstacle other than a sphere, as a scenario lists it in `boxes`: an aligned box (`min`, `max`), an oriented box (`center`,
/// `half_extents`, `rotation_deg`) or a capsule (`start`, `end`, `radius`). Each is told apart by its own fields, so a list written for the aligned
/// boxes alone reads as before.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Solid {
    Capsule(CapsuleObstacle),
    Obb(ObbObstacle),
    Aabb(BoxObstacle),
}

impl Solid {
    pub fn contains_point(&self, point: Vec3, clearance: f64) -> bool {
        match self {
            Solid::Aabb(s) => s.contains_point(point, clearance),
            Solid::Obb(s) => s.contains_point(point, clearance),
            Solid::Capsule(s) => s.contains_point(point, clearance),
        }
    }

    pub fn intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        match self {
            Solid::Aabb(s) => s.intersects_segment(a, b, clearance),
            Solid::Obb(s) => s.intersects_segment(a, b, clearance),
            Solid::Capsule(s) => s.intersects_segment(a, b, clearance),
        }
    }
}

/// Boxes are padded by this much so that rounding can never make the
/// hierarchy skip an obstacle the exact check would report.
const BOX_MARGIN: f64 = 1e-9;
/// A node with this many obstacles or fewer is a leaf.
const LEAF_SIZE: usize = 4;

#[derive(Debug, Clone, Copy)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn of(o: &Obstacle) -> Self {
        Aabb {
            min: Vec3::new(
                o.center.x - o.radius,
                o.center.y - o.radius,
                o.center.z - o.radius,
            ),
            max: Vec3::new(
                o.center.x + o.radius,
                o.center.y + o.radius,
                o.center.z + o.radius,
            ),
        }
    }

    fn union(self, other: Aabb) -> Aabb {
        Aabb {
            min: Vec3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Vec3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }

    /// Does the segment `a`-`b` touch this box grown by `grow` on every side? (slab test)
    fn touches_segment(&self, a: Vec3, b: Vec3, grow: f64) -> bool {
        let mut t_min = 0.0_f64;
        let mut t_max = 1.0_f64;
        let axes = [
            (a.x, b.x - a.x, self.min.x - grow, self.max.x + grow),
            (a.y, b.y - a.y, self.min.y - grow, self.max.y + grow),
            (a.z, b.z - a.z, self.min.z - grow, self.max.z + grow),
        ];
        for (start, delta, low, high) in axes {
            if delta.abs() < 1e-300 {
                if start < low || start > high {
                    return false;
                }
            } else {
                let mut t1 = (low - start) / delta;
                let mut t2 = (high - start) / delta;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                t_min = t_min.max(t1);
                t_max = t_max.min(t2);
                if t_min > t_max {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Debug)]
enum Node {
    Leaf {
        bounds: Aabb,
        first: usize,
        count: usize,
    },
    Inner {
        bounds: Aabb,
        left: usize,
        right: usize,
    },
}

/// A bounding-volume hierarchy over a fixed set of spherical obstacles.
#[derive(Debug)]
pub struct ObstacleIndex {
    boxes: Vec<Solid>,
    obstacles: Vec<Obstacle>,
    nodes: Vec<Node>,
}

impl ObstacleIndex {
    /// Builds the hierarchy: the set is split at the median centre along the
    /// widest axis until a node holds `LEAF_SIZE` obstacles or fewer.
    pub fn new(obstacles: &[Obstacle]) -> Self {
        let mut ordered = obstacles.to_vec();
        let mut nodes = Vec::new();
        if !ordered.is_empty() {
            let len = ordered.len();
            Self::build(&mut ordered, 0, len, &mut nodes);
        }
        ObstacleIndex {
            boxes: Vec::new(),
            obstacles: ordered,
            nodes,
        }
    }

    /// The same hierarchy, plus the boxes of the scenario (tested one by one: a scene has few of them).
    pub fn with_boxes(obstacles: &[Obstacle], boxes: &[Solid]) -> Self {
        let mut index = Self::new(obstacles);
        index.boxes = boxes.to_vec();
        index
    }

    fn build(items: &mut [Obstacle], first: usize, end: usize, nodes: &mut Vec<Node>) -> usize {
        let bounds = items[first..end]
            .iter()
            .map(Aabb::of)
            .reduce(Aabb::union)
            .expect("a node is never empty");
        let index = nodes.len();
        if end - first <= LEAF_SIZE {
            nodes.push(Node::Leaf {
                bounds,
                first,
                count: end - first,
            });
            return index;
        }
        let widths = [
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
        ];
        let axis = if widths[0] >= widths[1] && widths[0] >= widths[2] {
            0
        } else if widths[1] >= widths[2] {
            1
        } else {
            2
        };
        let key = |o: &Obstacle| match axis {
            0 => o.center.x,
            1 => o.center.y,
            _ => o.center.z,
        };
        items[first..end].sort_by(|p, q| key(p).total_cmp(&key(q)));
        let middle = first + (end - first) / 2;
        // reserve the slot so the children come after their parent
        nodes.push(Node::Leaf {
            bounds,
            first,
            count: 0,
        });
        let left = Self::build(items, first, middle, nodes);
        let right = Self::build(items, middle, end, nodes);
        nodes[index] = Node::Inner {
            bounds,
            left,
            right,
        };
        index
    }

    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.obstacles.len()
    }

    /// True if the segment `a`-`b` comes within `clearance` of any obstacle;
    /// the same answer as `obstacles.iter().any(|o| o.intersects_segment(..))`.
    pub fn any_intersects_segment(&self, a: Vec3, b: Vec3, clearance: f64) -> bool {
        // the boxes of a scenario are few: each one is tested on its own
        if self
            .boxes
            .iter()
            .any(|b_box| b_box.intersects_segment(a, b, clearance))
        {
            return true;
        }
        if self.nodes.is_empty() {
            return false;
        }
        let grow = clearance + BOX_MARGIN;
        let mut stack = vec![0usize];
        while let Some(at) = stack.pop() {
            match &self.nodes[at] {
                Node::Leaf {
                    bounds,
                    first,
                    count,
                } => {
                    if bounds.touches_segment(a, b, grow)
                        && self.obstacles[*first..*first + *count]
                            .iter()
                            .any(|o| o.intersects_segment(a, b, clearance))
                    {
                        return true;
                    }
                }
                Node::Inner {
                    bounds,
                    left,
                    right,
                } => {
                    if bounds.touches_segment(a, b, grow) {
                        stack.push(*left);
                        stack.push(*right);
                    }
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Xorshift64Star;

    fn scatter(count: usize, seed: u64) -> Vec<Obstacle> {
        let mut rng = Xorshift64Star::new(seed);
        (0..count)
            .map(|_| {
                Obstacle::new(
                    Vec3::new(
                        rng.next_range(-20.0, 20.0),
                        rng.next_range(-20.0, 20.0),
                        rng.next_range(-20.0, 20.0),
                    ),
                    rng.next_range(0.0, 1.5),
                )
            })
            .collect()
    }

    fn brute(obstacles: &[Obstacle], a: Vec3, b: Vec3, clearance: f64) -> bool {
        obstacles
            .iter()
            .any(|o| o.intersects_segment(a, b, clearance))
    }

    #[test]
    fn the_index_answers_exactly_what_the_scan_answers() {
        let mut rng = Xorshift64Star::new(99);
        for (count, seed) in [(1usize, 1u64), (3, 2), (4, 3), (5, 4), (40, 5), (300, 6)] {
            let obstacles = scatter(count, seed);
            let index = ObstacleIndex::new(&obstacles);
            assert_eq!(index.len(), count);
            let mut hits = 0;
            for _ in 0..1500 {
                let a = Vec3::new(
                    rng.next_range(-22.0, 22.0),
                    rng.next_range(-22.0, 22.0),
                    rng.next_range(-22.0, 22.0),
                );
                let b = a.add(Vec3::new(
                    rng.next_range(-6.0, 6.0),
                    rng.next_range(-6.0, 6.0),
                    rng.next_range(-6.0, 6.0),
                ));
                let clearance = rng.next_range(0.0, 1.0);
                let expected = brute(&obstacles, a, b, clearance);
                assert_eq!(index.any_intersects_segment(a, b, clearance), expected);
                if expected {
                    hits += 1;
                }
            }
            assert!(
                hits > 0 || count < 5,
                "the queries never hit anything with {count} obstacles"
            );
        }
    }

    #[test]
    fn an_empty_index_never_reports_a_hit() {
        let index = ObstacleIndex::new(&[]);
        assert_eq!(index.len(), 0);
        assert!(!index.any_intersects_segment(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.5
        ));
    }

    #[test]
    fn a_zero_length_segment_and_axis_aligned_segments_are_handled() {
        let obstacles = scatter(60, 8);
        let index = ObstacleIndex::new(&obstacles);
        let mut rng = Xorshift64Star::new(5);
        for _ in 0..800 {
            let a = Vec3::new(
                rng.next_range(-20.0, 20.0),
                rng.next_range(-20.0, 20.0),
                rng.next_range(-20.0, 20.0),
            );
            for b in [
                a,
                a.add(Vec3::new(4.0, 0.0, 0.0)),
                a.add(Vec3::new(0.0, -4.0, 0.0)),
                a.add(Vec3::new(0.0, 0.0, 4.0)),
            ] {
                assert_eq!(
                    index.any_intersects_segment(a, b, 0.2),
                    brute(&obstacles, a, b, 0.2)
                );
            }
        }
    }

    #[test]
    fn a_segment_that_only_touches_the_clearance_is_found() {
        // the sphere surface is 1.0 from the line: a clearance of 1.0 reaches it
        let obstacles = vec![Obstacle::new(Vec3::new(5.0, 1.5, 0.0), 0.5)];
        let index = ObstacleIndex::new(&obstacles);
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(10.0, 0.0, 0.0);
        assert!(index.any_intersects_segment(a, b, 1.0));
        assert!(!index.any_intersects_segment(a, b, 0.9));
    }

    #[test]
    fn many_identical_obstacles_do_not_break_the_build() {
        let same = vec![Obstacle::new(Vec3::new(1.0, 1.0, 1.0), 0.5); 100];
        let index = ObstacleIndex::new(&same);
        assert!(index.any_intersects_segment(
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(2.0, 1.0, 1.0),
            0.0
        ));
        assert!(!index.any_intersects_segment(
            Vec3::new(5.0, 5.0, 5.0),
            Vec3::new(6.0, 5.0, 5.0),
            0.0
        ));
    }

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

#[cfg(test)]
mod box_tests {
    use super::*;

    fn wall() -> BoxObstacle {
        BoxObstacle {
            min: Vec3::new(-0.5, -1.0, -1.0),
            max: Vec3::new(0.5, 1.0, 1.0),
        }
    }

    #[test]
    fn a_point_is_inside_the_box_inflated_by_the_clearance() {
        let b = wall();
        assert!(b.contains_point(Vec3::new(0.0, 0.0, 0.0), 0.0));
        assert!(
            b.contains_point(Vec3::new(0.5, 1.0, 1.0), 0.0),
            "the faces count as inside"
        );
        assert!(!b.contains_point(Vec3::new(0.6, 0.0, 0.0), 0.0));
        assert!(b.contains_point(Vec3::new(0.6, 0.0, 0.0), 0.1));
        assert!(!b.contains_point(Vec3::new(0.0, 1.2, 0.0), 0.1));
    }

    #[test]
    fn a_segment_through_the_box_or_grazing_its_inflated_face_is_a_hit() {
        let b = wall();
        assert!(b.intersects_segment(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 0.0));
        assert!(!b.intersects_segment(Vec3::new(-2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0), 0.0));
        assert!(b.intersects_segment(Vec3::new(-2.0, 1.05, 0.0), Vec3::new(2.0, 1.05, 0.0), 0.1));
        assert!(!b.intersects_segment(Vec3::new(-2.0, 1.2, 0.0), Vec3::new(2.0, 1.2, 0.0), 0.1));
    }

    #[test]
    fn a_segment_that_stops_short_or_lies_beside_the_box_is_not_a_hit() {
        let b = wall();
        assert!(!b.intersects_segment(Vec3::new(-3.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), 0.0));
        assert!(b.intersects_segment(Vec3::new(-3.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.0));
        // a segment parallel to an axis, outside the slab of that axis
        assert!(!b.intersects_segment(Vec3::new(0.0, 0.0, 2.0), Vec3::new(0.0, 0.0, 3.0), 0.0));
        // a zero-length segment is a point
        assert!(b.intersects_segment(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.0));
        assert!(!b.intersects_segment(Vec3::new(2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 0.0));
    }

    #[test]
    fn the_segment_test_agrees_with_dense_sampling() {
        let b = wall();
        let mut state = 12345u64;
        let mut next = || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) * 6.0 - 3.0
        };
        for _ in 0..400 {
            let a = Vec3::new(next(), next(), next());
            let c = Vec3::new(next(), next(), next());
            let sampled = (0..=2000).any(|i| {
                let t = f64::from(i) / 2000.0;
                b.contains_point(a.add(c.sub(a).scale(t)), 0.05)
            });
            if sampled {
                assert!(b.intersects_segment(a, c, 0.05), "{a:?} -> {c:?}");
            }
        }
    }

    #[test]
    fn the_index_with_boxes_reports_a_box_even_when_there_is_no_sphere() {
        let index = ObstacleIndex::with_boxes(&[], &[Solid::Aabb(wall())]);
        assert!(index.any_intersects_segment(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            0.1
        ));
        assert!(!index.any_intersects_segment(
            Vec3::new(-2.0, 3.0, 0.0),
            Vec3::new(2.0, 3.0, 0.0),
            0.1
        ));
        let plain = ObstacleIndex::new(&[]);
        assert!(!plain.any_intersects_segment(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            0.1
        ));
    }
}

#[cfg(test)]
mod solid_tests {
    use super::*;
    use crate::rng::Xorshift64Star;
    use crate::rrt::{plan_with_boxes, PlanError, PlannerConfig, Workspace};
    use crate::semantics::{validate_boxes, InvalidScenarioReason};
    use crate::validate::validate_path_with_boxes;

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    fn obb(center: Vec3, half: Vec3, roll: f64, pitch: f64, yaw: f64) -> ObbObstacle {
        ObbObstacle {
            center,
            half_extents: half,
            rotation_deg: Rotation { roll, pitch, yaw },
        }
    }

    fn capsule(start: Vec3, end: Vec3, radius: f64) -> CapsuleObstacle {
        CapsuleObstacle { start, end, radius }
    }

    #[test]
    fn an_oriented_box_without_rotation_answers_like_the_aligned_box() {
        let mut rng = Xorshift64Star::new(2024);
        for _ in 0..400 {
            let center = v(
                rng.next_range(-3.0, 3.0),
                rng.next_range(-3.0, 3.0),
                rng.next_range(-3.0, 3.0),
            );
            let half = v(
                rng.next_range(0.1, 2.0),
                rng.next_range(0.1, 2.0),
                rng.next_range(0.1, 2.0),
            );
            let aligned = BoxObstacle {
                min: center.sub(half),
                max: center.add(half),
            };
            let oriented = obb(center, half, 0.0, 0.0, 0.0);
            let clearance = rng.next_range(0.0, 0.5);
            let a = v(
                rng.next_range(-8.0, 8.0),
                rng.next_range(-8.0, 8.0),
                rng.next_range(-8.0, 8.0),
            );
            let b = v(
                rng.next_range(-8.0, 8.0),
                rng.next_range(-8.0, 8.0),
                rng.next_range(-8.0, 8.0),
            );
            assert_eq!(
                aligned.contains_point(a, clearance),
                oriented.contains_point(a, clearance)
            );
            assert_eq!(
                aligned.intersects_segment(a, b, clearance),
                oriented.intersects_segment(a, b, clearance)
            );
        }
    }

    #[test]
    fn a_quarter_turn_about_z_swaps_the_long_side() {
        let long = obb(v(0.0, 0.0, 0.0), v(2.0, 0.5, 0.5), 0.0, 0.0, 90.0);
        assert!(
            long.contains_point(v(0.0, 1.5, 0.0), 0.0),
            "the long side now lies along y"
        );
        assert!(
            !long.contains_point(v(1.5, 0.0, 0.0), 0.0),
            "and no longer along x"
        );
        let flat = obb(v(0.0, 0.0, 0.0), v(2.0, 0.5, 0.5), 0.0, 0.0, 0.0);
        assert!(flat.contains_point(v(1.5, 0.0, 0.0), 0.0));
        assert!(!flat.contains_point(v(0.0, 1.5, 0.0), 0.0));
    }

    #[test]
    fn a_diagonal_box_holds_the_diagonal_and_not_the_other_corner() {
        let fence = obb(v(0.0, 0.0, 0.0), v(1.0, 0.1, 0.1), 0.0, 0.0, 45.0);
        assert!(fence.contains_point(v(0.7, 0.7, 0.0), 0.0));
        assert!(!fence.contains_point(v(0.7, -0.7, 0.0), 0.0));
        // a segment across the diagonal hits it, a segment parallel and apart does not
        assert!(fence.intersects_segment(v(0.7, -0.7, 0.0), v(-0.7, 0.7, 0.0), 0.0));
        assert!(!fence.intersects_segment(v(0.7, -0.7, 0.0), v(1.5, -0.7, 0.0), 0.0));
    }

    #[test]
    fn roll_and_pitch_tilt_the_box_and_a_tilted_one_is_not_where_the_aligned_one_is() {
        let slab = obb(v(0.0, 0.0, 0.0), v(2.0, 2.0, 0.1), 0.0, 90.0, 0.0);
        // a pitch of 90 degrees stands the slab up: its thin side is now along x
        assert!(slab.contains_point(v(0.0, 1.5, 1.5), 0.0));
        assert!(!slab.contains_point(v(1.5, 0.0, 0.0), 0.0));
        let rolled = obb(v(0.0, 0.0, 0.0), v(0.1, 2.0, 2.0), 90.0, 0.0, 0.0);
        assert!(
            rolled.contains_point(v(0.0, 1.5, 1.5), 0.0),
            "a roll of 90 degrees keeps a square section"
        );
        let long = obb(v(0.0, 0.0, 0.0), v(0.1, 2.0, 0.1), 90.0, 0.0, 0.0);
        assert!(
            long.contains_point(v(0.0, 0.0, 1.5), 0.0),
            "a roll of 90 degrees turns y into z"
        );
        assert!(!long.contains_point(v(0.0, 1.5, 0.0), 0.0));
    }

    #[test]
    fn the_clearance_inflates_an_oriented_box_and_touching_counts_as_inside() {
        let b = obb(v(1.0, 1.0, 0.0), v(1.0, 0.5, 0.5), 0.0, 0.0, 90.0);
        // after the quarter turn the box spans x 0.5..1.5 and y 0..2
        assert!(
            b.contains_point(v(1.5, 1.0, 0.0), 0.0),
            "the face counts as inside"
        );
        assert!(!b.contains_point(v(1.6, 1.0, 0.0), 0.0));
        assert!(b.contains_point(v(1.6, 1.0, 0.0), 0.1 + 1e-9));
    }

    #[test]
    fn a_zero_size_oriented_box_is_a_point_and_a_segment_inside_the_box_hits() {
        let point = obb(v(1.0, 1.0, 1.0), v(0.0, 0.0, 0.0), 10.0, 20.0, 30.0);
        assert!(point.contains_point(v(1.0, 1.0, 1.0), 0.0));
        assert!(!point.contains_point(v(1.1, 1.0, 1.0), 0.0));
        let big = obb(v(0.0, 0.0, 0.0), v(3.0, 3.0, 3.0), 15.0, 25.0, 35.0);
        assert!(
            big.intersects_segment(v(-0.5, 0.0, 0.0), v(0.5, 0.0, 0.0), 0.0),
            "wholly inside"
        );
        assert!(
            big.intersects_segment(v(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0), 0.0),
            "a zero-length segment inside"
        );
        assert!(
            !big.intersects_segment(v(9.0, 9.0, 9.0), v(9.0, 9.0, 9.0), 0.0),
            "and one outside"
        );
    }

    #[test]
    fn a_capsule_holds_the_points_within_its_radius_of_the_axis_ends_included() {
        let pole = capsule(v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0), 0.5);
        assert!(
            pole.contains_point(v(2.0, 0.5, 0.0), 0.0),
            "touching counts"
        );
        assert!(!pole.contains_point(v(2.0, 0.51, 0.0), 0.0));
        assert!(
            pole.contains_point(v(4.4, 0.0, 0.0), 0.0),
            "the rounded end"
        );
        assert!(!pole.contains_point(v(4.6, 0.0, 0.0), 0.0));
        assert!(
            !pole.contains_point(v(4.4, 0.4, 0.0), 0.0),
            "a corner of the box around it is not in the capsule"
        );
        assert!(
            pole.contains_point(v(2.0, 0.7, 0.0), 0.2 + 1e-9),
            "the clearance inflates it"
        );
    }

    #[test]
    fn a_segment_crossing_or_near_the_axis_of_a_capsule_hits_it() {
        let pole = capsule(v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0), 0.5);
        assert!(
            pole.intersects_segment(v(2.0, -3.0, 0.0), v(2.0, 3.0, 0.0), 0.0),
            "across"
        );
        // a skew segment one metre above the axis: it hits only when the clearance closes the gap
        assert!(!pole.intersects_segment(v(2.0, -3.0, 1.0), v(2.0, 3.0, 1.0), 0.0));
        assert!(pole.intersects_segment(v(2.0, -3.0, 1.0), v(2.0, 3.0, 1.0), 0.5 + 1e-9));
        // parallel and apart
        assert!(!pole.intersects_segment(v(0.0, 2.0, 0.0), v(4.0, 2.0, 0.0), 0.0));
        assert!(pole.intersects_segment(v(0.0, 2.0, 0.0), v(4.0, 2.0, 0.0), 1.5 + 1e-9));
        // collinear and overlapping, and collinear beyond the end
        assert!(pole.intersects_segment(v(3.0, 0.0, 0.0), v(8.0, 0.0, 0.0), 0.0));
        assert!(!pole.intersects_segment(v(6.0, 0.0, 0.0), v(8.0, 0.0, 0.0), 0.0));
        assert!(pole.intersects_segment(v(6.0, 0.0, 0.0), v(8.0, 0.0, 0.0), 1.5 + 1e-9));
    }

    #[test]
    fn a_capsule_with_equal_ends_is_a_sphere_and_a_point_segment_is_handled() {
        let ball = capsule(v(1.0, 1.0, 1.0), v(1.0, 1.0, 1.0), 0.5);
        let sphere = Obstacle::new(v(1.0, 1.0, 1.0), 0.5);
        let mut rng = Xorshift64Star::new(77);
        for _ in 0..300 {
            let a = v(
                rng.next_range(-3.0, 4.0),
                rng.next_range(-3.0, 4.0),
                rng.next_range(-3.0, 4.0),
            );
            let b = v(
                rng.next_range(-3.0, 4.0),
                rng.next_range(-3.0, 4.0),
                rng.next_range(-3.0, 4.0),
            );
            let clearance = rng.next_range(0.0, 0.4);
            assert_eq!(
                ball.contains_point(a, clearance),
                sphere.contains_point(a, clearance)
            );
            assert_eq!(
                ball.intersects_segment(a, b, clearance),
                sphere.intersects_segment(a, b, clearance)
            );
        }
        let pole = capsule(v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0), 0.5);
        assert!(
            pole.intersects_segment(v(2.0, 0.3, 0.0), v(2.0, 0.3, 0.0), 0.0),
            "a zero-length segment near the axis"
        );
        assert!(!pole.intersects_segment(v(2.0, 3.0, 0.0), v(2.0, 3.0, 0.0), 0.0));
    }

    #[test]
    fn the_distance_between_segments_is_symmetric_and_never_above_the_distance_of_any_two_of_their_points(
    ) {
        let mut rng = Xorshift64Star::new(5);
        for _ in 0..300 {
            let mut p = || {
                v(
                    rng.next_range(-4.0, 4.0),
                    rng.next_range(-4.0, 4.0),
                    rng.next_range(-4.0, 4.0),
                )
            };
            let (a, b, c, d) = (p(), p(), p(), p());
            let forward = segment_distance(a, b, c, d);
            let backward = segment_distance(c, d, a, b);
            assert!((forward - backward).abs() < 1e-9, "{forward} {backward}");
            assert!(
                forward <= a.distance(c) + 1e-9
                    && forward <= b.distance(d) + 1e-9
                    && forward <= a.distance(d) + 1e-9
            );
            assert!(forward >= 0.0);
        }
    }

    #[test]
    fn a_scenario_list_of_aligned_boxes_still_reads_and_the_new_shapes_read_next_to_them() {
        let text = r#"[
            {"min":{"x":0,"y":0,"z":0},"max":{"x":1,"y":1,"z":1}},
            {"shape":"aabb","min":{"x":2,"y":0,"z":0},"max":{"x":3,"y":1,"z":1}},
            {"center":{"x":0,"y":0,"z":0},"half_extents":{"x":1,"y":2,"z":3},"rotation_deg":{"yaw":30}},
            {"center":{"x":0,"y":0,"z":0},"half_extents":{"x":1,"y":2,"z":3}},
            {"start":{"x":0,"y":0,"z":0},"end":{"x":0,"y":0,"z":1},"radius":0.2}]"#;
        let solids: Vec<Solid> = serde_json::from_str(text).unwrap();
        assert!(matches!(solids[0], Solid::Aabb(_)));
        assert!(
            matches!(solids[1], Solid::Aabb(_)),
            "a stray shape key is ignored as before"
        );
        assert!(matches!(solids[2], Solid::Obb(b) if b.rotation_deg.yaw == 30.0));
        assert!(matches!(solids[3], Solid::Obb(b) if b.rotation_deg.yaw == 0.0));
        assert!(matches!(solids[4], Solid::Capsule(b) if b.radius == 0.2));
        assert!(serde_json::from_str::<Vec<Solid>>(r#"[{"min":{"x":0,"y":0,"z":0}}]"#).is_err());
        assert!(serde_json::from_str::<Vec<Solid>>(r#"[{"radius":1}]"#).is_err());
    }

    #[test]
    fn the_index_reports_an_oriented_box_and_a_capsule_even_when_there_is_no_sphere() {
        let solids = [
            Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, 0.2, 0.2), 0.0, 0.0, 90.0)),
            Solid::Capsule(capsule(v(5.0, 0.0, 0.0), v(5.0, 0.0, 3.0), 0.3)),
        ];
        let index = ObstacleIndex::with_boxes(&[], &solids);
        assert!(
            index.any_intersects_segment(v(0.0, -2.0, 0.0), v(0.0, 2.0, 0.0), 0.0),
            "across the turned box"
        );
        assert!(!index.any_intersects_segment(v(-2.0, -2.0, 0.0), v(-2.0, 2.0, 0.0), 0.0));
        assert!(
            index.any_intersects_segment(v(4.0, 0.0, 1.0), v(6.0, 0.0, 1.0), 0.0),
            "across the pole"
        );
        assert!(
            !index.any_intersects_segment(v(4.0, 0.0, 4.0), v(6.0, 0.0, 4.0), 0.0),
            "above the pole"
        );
    }

    #[test]
    fn a_solid_that_is_not_finite_or_is_upside_down_is_an_invalid_scenario() {
        let finite = Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0), 0.0, 0.0, 0.0));
        assert!(validate_boxes(&[finite]).is_ok());
        let nan_center = Solid::Obb(obb(v(f64::NAN, 0.0, 0.0), v(1.0, 1.0, 1.0), 0.0, 0.0, 0.0));
        assert!(matches!(
            validate_boxes(&[finite, nan_center]),
            Err(InvalidScenarioReason::NonFiniteBox { index: 1 })
        ));
        let nan_angle = Solid::Obb(obb(
            v(0.0, 0.0, 0.0),
            v(1.0, 1.0, 1.0),
            0.0,
            f64::INFINITY,
            0.0,
        ));
        assert!(matches!(
            validate_boxes(&[nan_angle]),
            Err(InvalidScenarioReason::NonFiniteBox { index: 0 })
        ));
        let negative = Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, -1.0, 1.0), 0.0, 0.0, 0.0));
        assert!(matches!(
            validate_boxes(&[negative]),
            Err(InvalidScenarioReason::InvertedBox { index: 0 })
        ));
        let flat = Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, 0.0, 1.0), 0.0, 0.0, 0.0));
        assert!(
            validate_boxes(&[flat]).is_ok(),
            "a half extent of 0 is a plate"
        );
        let bad_radius = Solid::Capsule(capsule(v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), -0.1));
        assert!(matches!(
            validate_boxes(&[bad_radius]),
            Err(InvalidScenarioReason::InvertedBox { index: 0 })
        ));
        let nan_end = Solid::Capsule(capsule(v(0.0, 0.0, 0.0), v(f64::NAN, 0.0, 0.0), 0.1));
        assert!(matches!(
            validate_boxes(&[nan_end]),
            Err(InvalidScenarioReason::NonFiniteBox { index: 0 })
        ));
        let sphere_like = Solid::Capsule(capsule(v(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0), 0.0));
        assert!(
            validate_boxes(&[sphere_like]).is_ok(),
            "a radius of 0 on equal ends is a point"
        );
    }

    fn open_workspace() -> Workspace {
        Workspace {
            min: v(-10.0, -10.0, -10.0),
            max: v(10.0, 10.0, 10.0),
        }
    }

    fn config() -> PlannerConfig {
        PlannerConfig {
            max_iterations: 20_000,
            ..PlannerConfig::default()
        }
    }

    #[test]
    fn a_plan_goes_around_a_turned_wall_and_a_pole_and_the_independent_check_agrees() {
        let start = v(-6.0, 0.0, 0.0);
        let goal = v(6.0, 0.0, 0.0);
        let solids = [
            // a wall across the way, turned 20 degrees, wider than the gap a straight line would need
            Solid::Obb(obb(v(0.0, 0.0, 0.0), v(0.3, 3.0, 10.0), 0.0, 0.0, 20.0)),
            // and a pole beyond it
            Solid::Capsule(capsule(v(3.0, 3.0, -10.0), v(3.0, 3.0, 10.0), 1.0)),
        ];
        let path = plan_with_boxes(start, goal, &[], &solids, open_workspace(), config(), 7)
            .expect("there is a way around a wall that does not span the workspace");
        let radius = config().robot_radius;
        for window in path.windows(2) {
            for solid in &solids {
                assert!(
                    !solid.intersects_segment(window[0], window[1], radius),
                    "{:?}->{:?} clips {solid:?}",
                    window[0],
                    window[1]
                );
            }
        }
        let issues = validate_path_with_boxes(&path, &[], &solids, &open_workspace(), radius);
        assert!(issues.is_empty(), "{issues:?}");
    }

    #[test]
    fn a_start_or_a_goal_inside_an_oriented_box_or_a_capsule_is_refused_before_the_search() {
        let solids = [
            Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0), 0.0, 0.0, 45.0)),
            Solid::Capsule(capsule(v(5.0, 0.0, 0.0), v(7.0, 0.0, 0.0), 0.5)),
        ];
        let inside_box = plan_with_boxes(
            v(0.0, 0.0, 0.0),
            v(9.0, 9.0, 9.0),
            &[],
            &solids,
            open_workspace(),
            config(),
            1,
        );
        assert_eq!(inside_box, Err(PlanError::StartInsideObstacle));
        let inside_capsule = plan_with_boxes(
            v(-9.0, -9.0, -9.0),
            v(6.0, 0.0, 0.0),
            &[],
            &solids,
            open_workspace(),
            config(),
            1,
        );
        assert_eq!(inside_capsule, Err(PlanError::GoalInsideObstacle));
    }

    #[test]
    fn the_independent_check_finds_a_path_through_a_turned_box_and_a_capsule() {
        let solids = [
            Solid::Obb(obb(v(0.0, 0.0, 0.0), v(1.0, 0.2, 5.0), 0.0, 0.0, 45.0)),
            Solid::Capsule(capsule(v(0.0, 5.0, -5.0), v(0.0, 5.0, 5.0), 0.5)),
        ];
        let through_box = [v(-3.0, -3.0, 0.0), v(3.0, 3.0, 0.0)];
        let issues = validate_path_with_boxes(&through_box, &[], &solids, &open_workspace(), 0.1);
        assert_eq!(issues.len(), 1, "{issues:?}");
        let through_pole = [v(-3.0, 5.0, 0.0), v(3.0, 5.0, 0.0)];
        assert!(
            !validate_path_with_boxes(&through_pole, &[], &solids, &open_workspace(), 0.1)
                .is_empty()
        );
        let clear = [v(-3.0, 3.0, 0.0), v(3.0, 3.0, 0.0)];
        assert!(validate_path_with_boxes(&clear, &[], &solids, &open_workspace(), 0.1).is_empty());
    }
}

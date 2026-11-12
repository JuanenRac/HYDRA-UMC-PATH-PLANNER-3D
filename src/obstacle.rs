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
            obstacles: ordered,
            nodes,
        }
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

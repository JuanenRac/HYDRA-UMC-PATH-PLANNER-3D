// HYDRA-UMC-PATH-PLANNER-3D - kdtree.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// A real, incremental 3D KD-tree for RRT's own nearest-neighbor lookup -
// rrt.rs's own linear scan was a deliberate, documented v0 choice ("fine
// at the tree sizes a single planning call produces today... simpler to
// verify correct than a KD-tree would have been for this first pass").
// This is exactly that follow-up: axis alternates x/y/z by depth,
// insertion-only (no rebalancing - the RRT tree only ever grows during a
// single plan() call, never removes a node, so a simple insertion-order
// tree is a real, correct, much smaller piece of code than a
// self-balancing variant, at the real cost of degrading toward O(n) in a
// pathological insertion order rather than guaranteeing O(log n)).
//
// Exactness, not just "a valid nearest neighbor", matters here: rrt.rs's
// own tests assert bit-for-bit identical paths for a given seed, so this
// tree's nearest() must return the EXACT SAME index the old linear scan
// (`Iterator::min_by`, "first element wins on a tie") would have -
// verified directly in this file's own tests by comparing against a
// real, independent linear scan across many random insertion/query
// sequences, not just "the distance looks right".

use crate::geometry::Vec3;

struct KdNode {
    point: Vec3,
    idx: usize,
    axis: u8, // 0=x, 1=y, 2=z - the coordinate this node splits its children on
    left: Option<Box<KdNode>>,
    right: Option<Box<KdNode>>,
}

fn axis_value(p: Vec3, axis: u8) -> f64 {
    match axis {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    }
}

/// An incremental KD-tree over `Vec3` points, each carrying an external
/// `idx` (the caller's own index into its real node storage - this tree
/// never owns or reconstructs a path itself, only accelerates "which of
/// my points is nearest to this query point").
pub struct KdTree {
    root: Option<Box<KdNode>>,
}

impl KdTree {
    pub fn new() -> Self {
        KdTree { root: None }
    }

    pub fn insert(&mut self, point: Vec3, idx: usize) {
        Self::insert_at(&mut self.root, point, idx, 0);
    }

    fn insert_at(slot: &mut Option<Box<KdNode>>, point: Vec3, idx: usize, depth: usize) {
        match slot {
            None => {
                *slot = Some(Box::new(KdNode {
                    point,
                    idx,
                    axis: (depth % 3) as u8,
                    left: None,
                    right: None,
                }));
            }
            Some(node) => {
                if axis_value(point, node.axis) < axis_value(node.point, node.axis) {
                    Self::insert_at(&mut node.left, point, idx, depth + 1);
                } else {
                    Self::insert_at(&mut node.right, point, idx, depth + 1);
                }
            }
        }
    }

    /// Returns the real index of the nearest inserted point to `target` -
    /// panics if the tree is empty, same contract rrt.rs's own prior
    /// linear `nearest()` had ("tree always has at least the start
    /// node").
    pub fn nearest(&self, target: Vec3) -> usize {
        let mut best: Option<(f64, usize)> = None;
        if let Some(root) = &self.root {
            Self::search(root, target, &mut best);
        }
        best.expect("KdTree::nearest called on an empty tree").1
    }

    fn search(node: &KdNode, target: Vec3, best: &mut Option<(f64, usize)>) {
        let d = node.point.distance(target);
        // Strict-improvement + explicit lowest-index tie-break, mirroring
        // Iterator::min_by's own "first element wins a tie" semantics -
        // real KD-tree traversal order does NOT match insertion order, so
        // this explicit index comparison is what actually keeps this
        // exactly equivalent to the old linear scan on a tie, not an
        // accident of tree shape.
        let better = match best {
            None => true,
            Some((best_d, best_idx)) => d < *best_d || (d == *best_d && node.idx < *best_idx),
        };
        if better {
            *best = Some((d, node.idx));
        }

        let diff = axis_value(target, node.axis) - axis_value(node.point, node.axis);
        let (near, far) = if diff < 0.0 {
            (&node.left, &node.right)
        } else {
            (&node.right, &node.left)
        };

        if let Some(n) = near {
            Self::search(n, target, best);
        }
        // Only the far subtree needs pruning: every point in it is at
        // least `diff.abs()` away from `target` along this one axis alone,
        // so it can only contain a real match if that alone doesn't
        // already rule it out. `<=` (not `<`) deliberately never risks
        // skipping a real tie sitting exactly on the splitting plane.
        let must_check_far = match best {
            Some((best_d, _)) => diff.abs() <= *best_d,
            None => true,
        };
        if must_check_far {
            if let Some(f) = far {
                Self::search(f, target, best);
            }
        }
    }
}

impl Default for KdTree {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Xorshift64Star;

    /// The real, independent reference implementation these tests check
    /// the KD-tree against - deliberately NOT rrt.rs's own private
    /// nearest(), so this test module has zero dependency on rrt.rs and
    /// can't accidentally validate the tree against a buggy copy of
    /// itself.
    fn linear_nearest(points: &[(Vec3, usize)], target: Vec3) -> usize {
        points
            .iter()
            .min_by(|(a, _), (b, _)| {
                a.distance(target)
                    .partial_cmp(&b.distance(target))
                    .expect("distance() never produces NaN for finite inputs")
            })
            .map(|(_, idx)| *idx)
            .expect("points must be non-empty")
    }

    fn random_point(rng: &mut Xorshift64Star) -> Vec3 {
        Vec3::new(
            rng.next_range(-50.0, 50.0),
            rng.next_range(-50.0, 50.0),
            rng.next_range(-50.0, 50.0),
        )
    }

    #[test]
    fn single_point_is_always_nearest() {
        let mut tree = KdTree::new();
        tree.insert(Vec3::new(1.0, 2.0, 3.0), 0);
        assert_eq!(tree.nearest(Vec3::new(100.0, -100.0, 5.0)), 0);
    }

    #[test]
    fn agrees_with_linear_scan_across_many_random_insertion_and_query_sequences() {
        // Real tree sizes rrt.rs's own module doc names: "hundreds to a
        // few thousand nodes" - swept across several real seeds so this
        // isn't just one lucky insertion order.
        for seed in 0..20u64 {
            let mut rng = Xorshift64Star::new(seed + 1);
            let mut tree = KdTree::new();
            let mut points: Vec<(Vec3, usize)> = Vec::new();

            for idx in 0..500 {
                let p = random_point(&mut rng);
                tree.insert(p, idx);
                points.push((p, idx));

                // Check agreement incrementally (not just once at the
                // end) - a real bug in the incremental insert path could
                // easily only manifest at specific tree sizes/shapes.
                if idx % 37 == 0 {
                    for _ in 0..5 {
                        let target = random_point(&mut rng);
                        assert_eq!(
                            tree.nearest(target),
                            linear_nearest(&points, target),
                            "seed={seed} tree_size={} target={target:?}",
                            points.len()
                        );
                    }
                }
            }

            for _ in 0..20 {
                let target = random_point(&mut rng);
                assert_eq!(
                    tree.nearest(target),
                    linear_nearest(&points, target),
                    "seed={seed} final tree_size={} target={target:?}",
                    points.len()
                );
            }
        }
    }

    #[test]
    fn agrees_with_linear_scan_when_the_query_point_exactly_matches_an_inserted_point() {
        // A real degenerate case: distance 0.0 from more than one node is
        // impossible unless two inserted points are literally identical,
        // but querying exactly AT an inserted point (distance 0 from
        // exactly one real node) is a real, common case worth its own
        // explicit check - `<=` in the pruning decision must not skip a
        // real exact match sitting on a splitting plane.
        let mut rng = Xorshift64Star::new(7);
        let mut tree = KdTree::new();
        let mut points: Vec<(Vec3, usize)> = Vec::new();
        for idx in 0..200 {
            let p = random_point(&mut rng);
            tree.insert(p, idx);
            points.push((p, idx));
        }
        for (p, _) in &points {
            assert_eq!(tree.nearest(*p), linear_nearest(&points, *p));
        }
    }
}

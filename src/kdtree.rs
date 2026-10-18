// HYDRA-UMC-PATH-PLANNER-3D - kdtree.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// A real, incremental 3D KD-tree for RRT's own nearest-neighbor lookup -
// rrt.rs's own linear scan was a deliberate, documented v0 choice ("fine
// at the tree sizes a single planning call produces today... simpler to
// verify correct than a KD-tree would have been for this first pass").
// This is exactly that follow-up: axis alternates x/y/z by depth,
// insertion-only (the RRT tree only ever grows during a single plan()
// call, never removes a node). Nodes live in one vector and both insert
// and search walk it with a loop and an explicit stack, so the depth of
// the tree can never overflow the call stack. An insertion order that is
// monotonic along an axis (a long straight corridor, a goal bias of 1)
// would make a plain insertion-order tree a linked list; the tree is
// therefore rebuilt balanced each time it doubles in size, which keeps
// every query near O(log n) at an amortised O(log n) per insert.
//
// Exactness, not just "a valid nearest neighbor", matters here: rrt.rs's
// own tests assert bit-for-bit identical paths for a given seed, so this
// tree's nearest() must return the EXACT SAME index the old linear scan
// (`Iterator::min_by`, "first element wins on a tie") would have -
// verified directly in this file's own tests by comparing against a
// real, independent linear scan across many random insertion/query
// sequences, not just "the distance looks right".

use crate::geometry::Vec3;

const NO_CHILD: usize = usize::MAX;

/// Below this many points a rebuild is not worth its cost.
const MIN_REBUILD_SIZE: usize = 64;

struct KdNode {
    point: Vec3,
    idx: usize,
    axis: u8, // 0=x, 1=y, 2=z - the coordinate this node splits its children on
    left: usize,
    right: usize,
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
    nodes: Vec<KdNode>,
    root: usize,
    /// every point ever inserted, in insertion order, for the rebuilds
    points: Vec<(Vec3, usize)>,
    /// the size at which the tree is rebuilt next
    rebuild_at: usize,
}

impl KdTree {
    pub fn new() -> Self {
        KdTree {
            nodes: Vec::new(),
            root: NO_CHILD,
            points: Vec::new(),
            rebuild_at: MIN_REBUILD_SIZE,
        }
    }

    pub fn insert(&mut self, point: Vec3, idx: usize) {
        self.points.push((point, idx));
        if self.points.len() >= self.rebuild_at {
            self.rebuild();
            self.rebuild_at = self.points.len() * 2;
            return;
        }
        self.insert_node(point, idx);
    }

    fn insert_node(&mut self, point: Vec3, idx: usize) {
        let new_index = self.nodes.len();
        if self.root == NO_CHILD {
            self.nodes.push(KdNode {
                point,
                idx,
                axis: 0,
                left: NO_CHILD,
                right: NO_CHILD,
            });
            self.root = new_index;
            return;
        }
        let mut current = self.root;
        let mut depth = 0usize;
        loop {
            let node = &self.nodes[current];
            let go_left = axis_value(point, node.axis) < axis_value(node.point, node.axis);
            let next = if go_left { node.left } else { node.right };
            depth += 1;
            if next == NO_CHILD {
                self.nodes.push(KdNode {
                    point,
                    idx,
                    axis: (depth % 3) as u8,
                    left: NO_CHILD,
                    right: NO_CHILD,
                });
                if go_left {
                    self.nodes[current].left = new_index;
                } else {
                    self.nodes[current].right = new_index;
                }
                return;
            }
            current = next;
        }
    }

    /// Replaces the tree with a balanced one over every point so far. The
    /// result answers every query exactly as the unbalanced tree did:
    /// `nearest` breaks ties by the lowest index, whatever the shape.
    fn rebuild(&mut self) {
        let mut work: Vec<(Vec3, usize)> = self.points.clone();
        self.nodes.clear();
        self.nodes.reserve(work.len());
        self.root = Self::build(&mut self.nodes, &mut work, 0);
    }

    /// Median split on the axis of this depth. The depth of the recursion is
    /// about log2(n) (times at most 3 when whole axes are constant), so it
    /// cannot exhaust the stack.
    fn build(nodes: &mut Vec<KdNode>, points: &mut [(Vec3, usize)], depth: usize) -> usize {
        if points.is_empty() {
            return NO_CHILD;
        }
        let axis = (depth % 3) as u8;
        points.sort_by(|a, b| {
            axis_value(a.0, axis)
                .partial_cmp(&axis_value(b.0, axis))
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.1.cmp(&b.1))
        });
        // The first position holding the median value: everything before
        // it is strictly smaller, as insert_node's "< goes left" requires.
        let mut mid = points.len() / 2;
        let median_value = axis_value(points[mid].0, axis);
        while mid > 0 && axis_value(points[mid - 1].0, axis) == median_value {
            mid -= 1;
        }
        let (left_part, rest) = points.split_at_mut(mid);
        let (pivot, right_part) = rest
            .split_first_mut()
            .expect("rest holds at least the pivot");
        let index = nodes.len();
        nodes.push(KdNode {
            point: pivot.0,
            idx: pivot.1,
            axis,
            left: NO_CHILD,
            right: NO_CHILD,
        });
        let left = Self::build(nodes, left_part, depth + 1);
        let right = Self::build(nodes, right_part, depth + 1);
        nodes[index].left = left;
        nodes[index].right = right;
        index
    }

    /// Returns the real index of the nearest inserted point to `target` -
    /// panics if the tree is empty, same contract rrt.rs's own prior
    /// linear `nearest()` had ("tree always has at least the start
    /// node").
    pub fn nearest(&self, target: Vec3) -> usize {
        let mut best: Option<(f64, usize)> = None;
        // (node, distance along the parent's splitting axis when this entry
        // is the far side of it, so it can be pruned when it is reached)
        let mut stack: Vec<(usize, Option<f64>)> = Vec::new();
        if self.root != NO_CHILD {
            stack.push((self.root, None));
        }
        while let Some((current, plane_gap)) = stack.pop() {
            if let (Some(gap), Some((best_d, _))) = (plane_gap, best) {
                // Only the far subtree needs pruning: every point in it is
                // at least `gap` away from `target` along this one axis
                // alone. `>` (not `>=`) deliberately never risks skipping a
                // real tie sitting exactly on the splitting plane.
                if gap > best_d {
                    continue;
                }
            }
            let node = &self.nodes[current];
            let d = node.point.distance(target);
            // Strict-improvement + explicit lowest-index tie-break,
            // mirroring Iterator::min_by's own "first element wins a tie"
            // semantics - real KD-tree traversal order does NOT match
            // insertion order, so this explicit index comparison is what
            // actually keeps this exactly equivalent to the old linear scan
            // on a tie, not an accident of tree shape.
            let better = match best {
                None => true,
                Some((best_d, best_idx)) => d < best_d || (d == best_d && node.idx < best_idx),
            };
            if better {
                best = Some((d, node.idx));
            }
            let diff = axis_value(target, node.axis) - axis_value(node.point, node.axis);
            let (near, far) = if diff < 0.0 {
                (node.left, node.right)
            } else {
                (node.right, node.left)
            };
            if far != NO_CHILD {
                stack.push((far, Some(diff.abs())));
            }
            if near != NO_CHILD {
                stack.push((near, None));
            }
        }
        best.expect("KdTree::nearest called on an empty tree").1
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

    #[test]
    fn a_monotonic_insertion_order_neither_overflows_the_stack_nor_degrades() {
        // Every point has a larger x than the last and y = z = 0: the case
        // that made an insertion-order tree a linked list 40,000 deep.
        let mut tree = KdTree::new();
        let mut points: Vec<(Vec3, usize)> = Vec::new();
        for idx in 0..40_000usize {
            let p = Vec3::new(idx as f64 * 0.01, 0.0, 0.0);
            tree.insert(p, idx);
            points.push((p, idx));
        }
        let started = std::time::Instant::now();
        for q in 0..300 {
            let target = Vec3::new(q as f64 * 0.3 + 0.004, 0.2, -0.1);
            assert_eq!(
                tree.nearest(target),
                linear_nearest(&points[..points.len()], target)
            );
        }
        assert!(
            started.elapsed() < std::time::Duration::from_secs(60),
            "300 queries over 40,000 collinear points took {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn identical_points_resolve_to_the_lowest_index() {
        let mut tree = KdTree::new();
        for idx in 0..200usize {
            tree.insert(Vec3::new(1.0, 1.0, 1.0), idx);
        }
        assert_eq!(tree.nearest(Vec3::new(0.0, 0.0, 0.0)), 0);
    }
}

// HYDRA-UMC-PATH-PLANNER-3D - rng.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0 - see LICENSE
//
// A tiny, deterministic, dependency-free PRNG (xorshift64*) for RRT's
// random sampling. Not cryptographically secure, and not meant to be -
// planning does not need that, and a hand-rolled generator keeps this
// crate's dependency surface to just serde/serde_json (JSON I/O, not
// reasonably hand-written) instead of also pulling in the `rand` crate
// and its own dependency tree for one PRNG. Determinism from a fixed
// seed is a deliberate feature, not a limitation: it is what makes the
// RRT planner's own tests reproducible instead of flaky.

pub struct Xorshift64Star {
    state: u64,
}

impl Xorshift64Star {
    pub fn new(seed: u64) -> Self {
        // xorshift64* is undefined for a zero state (it would stay zero
        // forever) - fall back to a fixed nonzero constant so a caller
        // passing seed=0 still gets a real, working generator.
        Xorshift64Star {
            state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }

    /// Returns a float uniformly distributed in `[0.0, 1.0)`.
    pub fn next_f64(&mut self) -> f64 {
        // Use the top 53 bits (a f64 mantissa's worth of entropy) so the
        // result is uniformly spaced in [0, 1) rather than biased by a
        // naive `as f64 / u64::MAX as f64` cast.
        let bits = self.next_u64() >> 11;
        bits as f64 / (1u64 << 53) as f64
    }

    /// Returns a float uniformly distributed in `[min, max)`.
    pub fn next_range(&mut self, min: f64, max: f64) -> f64 {
        min + self.next_f64() * (max - min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_produces_same_sequence() {
        let mut a = Xorshift64Star::new(42);
        let mut b = Xorshift64Star::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Xorshift64Star::new(1);
        let mut b = Xorshift64Star::new(2);
        let seq_a: Vec<u64> = (0..10).map(|_| a.next_u64()).collect();
        let seq_b: Vec<u64> = (0..10).map(|_| b.next_u64()).collect();
        assert_ne!(seq_a, seq_b);
    }

    #[test]
    fn next_f64_stays_within_unit_interval() {
        let mut rng = Xorshift64Star::new(7);
        for _ in 0..10_000 {
            let v = rng.next_f64();
            assert!((0.0..1.0).contains(&v), "value {v} out of [0,1)");
        }
    }

    #[test]
    fn zero_seed_does_not_produce_a_stuck_generator() {
        let mut rng = Xorshift64Star::new(0);
        let a = rng.next_u64();
        let b = rng.next_u64();
        assert_ne!(a, 0);
        assert_ne!(a, b);
    }

    #[test]
    fn next_range_respects_bounds() {
        let mut rng = Xorshift64Star::new(123);
        for _ in 0..10_000 {
            let v = rng.next_range(-5.0, 5.0);
            assert!((-5.0..5.0).contains(&v));
        }
    }
}

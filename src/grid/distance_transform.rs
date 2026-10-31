//! Exact Euclidean distance transform.
//!
//! The implementation is the separable parabola-envelope algorithm of
//! Felzenszwalb and Huttenlocher, "Distance Transforms of Sampled Functions"
//! (*Theory of Computing*, 2012). For a one-dimensional sampled function `f`
//! it computes
//!
//! ```text
//! D(p) = min_q  (p - q)² + f(q)
//! ```
//!
//! in linear time by building the lower envelope of the parabolas
//! `q ↦ (p - q)² + f(q)`. The two-dimensional squared Euclidean distance
//! transform is that one-dimensional transform applied to each column and then
//! each row.
//!
//! Occupied sites are the zeros of `f`. Empty sites are `+∞`. The result at an
//! occupied site is `0`. If the grid contains no occupied site every result is
//! `+∞`.

use crate::errors::PathPlannerError;

/// Squared Euclidean distance, in cell units, from every cell to the nearest
/// occupied cell.
///
/// `occupied` is row-major (`index = y * width + x`) and must have length
/// `width * height`. Both dimensions must be non-zero. The squared distance is
/// measured between cell indices, so a cell one step east of an obstacle has
/// distance `1` and a diagonal neighbor has distance `2`.
pub fn squared_distance_transform(
    occupied: &[bool],
    width: usize,
    height: usize,
) -> Result<Vec<f64>, PathPlannerError> {
    let Some(cells) = width.checked_mul(height) else {
        return Err(PathPlannerError::InvalidConfiguration(
            "grid dimensions overflow".into(),
        ));
    };
    if width == 0 || height == 0 || occupied.len() != cells {
        return Err(PathPlannerError::InvalidConfiguration(
            "distance transform size does not match the occupancy buffer".into(),
        ));
    }

    // Column transform. `partial[x * height + y]` is the squared distance
    // considering only the column `x`, stored column-major so the following
    // row pass reads contiguous rows... we instead keep row-major and gather
    // columns into a scratch buffer.
    let mut column = vec![0.0; height];
    let mut column_dt = vec![0.0; height];
    let mut partial = vec![0.0; cells];
    for x in 0..width {
        for y in 0..height {
            column[y] = if occupied[y * width + x] {
                0.0
            } else {
                f64::INFINITY
            };
        }
        distance_transform_1d(&column, &mut column_dt);
        for y in 0..height {
            partial[y * width + x] = column_dt[y];
        }
    }

    let mut row = vec![0.0; width];
    let mut row_dt = vec![0.0; width];
    let mut out = vec![0.0; cells];
    for y in 0..height {
        let offset = y * width;
        row.copy_from_slice(&partial[offset..offset + width]);
        distance_transform_1d(&row, &mut row_dt);
        out[offset..offset + width].copy_from_slice(&row_dt);
    }
    Ok(out)
}

/// Lower envelope of parabolas. `out[q] = min_p (q - p)² + f[p]`.
fn distance_transform_1d(f: &[f64], out: &mut [f64]) {
    let n = f.len();
    debug_assert_eq!(out.len(), n);
    if n == 0 {
        return;
    }
    // `v[k]` is the site (parabola index) of the k-th segment of the envelope.
    // `z[k]` is the leftmost position where that parabola is minimal.
    let mut v = vec![0usize; n];
    let mut z = vec![0.0; n + 1];
    let mut k: isize = 0;
    v[0] = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;

    for q in 1..n {
        let mut s = intersect(f, q, v[k as usize]);
        while s <= z[k as usize] {
            k -= 1;
            if k < 0 {
                break;
            }
            s = intersect(f, q, v[k as usize]);
        }
        k += 1;
        v[k as usize] = q;
        z[k as usize] = s;
        z[k as usize + 1] = f64::INFINITY;
    }

    k = 0;
    for (q, slot) in out.iter_mut().enumerate() {
        while z[k as usize + 1] < q as f64 {
            k += 1;
        }
        let site = v[k as usize];
        let delta = q as f64 - site as f64;
        let base = f[site];
        *slot = if base.is_infinite() {
            f64::INFINITY
        } else {
            delta * delta + base
        };
    }
}

/// Horizontal position where the parabola rooted at `q` drops below the one
/// rooted at `v`.
fn intersect(f: &[f64], q: usize, v: usize) -> f64 {
    let fq = f[q];
    let fv = f[v];
    if fq.is_infinite() {
        // This site is empty. It cannot improve on a real site, and two empty
        // sites do not define a useful crossing.
        return f64::INFINITY;
    }
    if fv.is_infinite() {
        return f64::NEG_INFINITY;
    }
    let qf = q as f64;
    let vf = v as f64;
    ((fq + qf * qf) - (fv + vf * vf)) / (2.0 * qf - 2.0 * vf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    fn brute_force(occupied: &[bool], width: usize, height: usize) -> Vec<f64> {
        let mut sites = Vec::new();
        for y in 0..height {
            for x in 0..width {
                if occupied[y * width + x] {
                    sites.push((x as f64, y as f64));
                }
            }
        }
        let mut out = vec![f64::INFINITY; width * height];
        if sites.is_empty() {
            return out;
        }
        for y in 0..height {
            for x in 0..width {
                let mut best = f64::INFINITY;
                for &(sx, sy) in &sites {
                    let dx = x as f64 - sx;
                    let dy = y as f64 - sy;
                    best = best.min(dx * dx + dy * dy);
                }
                out[y * width + x] = best;
            }
        }
        out
    }

    fn assert_close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (i, (a, b)) in actual.iter().zip(expected).enumerate() {
            if a.is_infinite() || b.is_infinite() {
                assert!(a.is_infinite() && b.is_infinite(), "index {i}: {a} vs {b}");
            } else {
                assert!((a - b).abs() < 1e-6, "index {i}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn single_site_and_empty_and_full() {
        let mut occ = vec![false; 25];
        occ[3 * 5 + 2] = true;
        let dt = squared_distance_transform(&occ, 5, 5).unwrap();
        assert_close(&dt, &brute_force(&occ, 5, 5));

        let empty = vec![false; 12];
        let dt = squared_distance_transform(&empty, 4, 3).unwrap();
        assert!(dt.iter().all(|d| d.is_infinite()));

        let full = vec![true; 9];
        let dt = squared_distance_transform(&full, 3, 3).unwrap();
        assert!(dt.iter().all(|d| *d == 0.0));
    }

    #[test]
    fn matches_brute_force_on_random_grids() {
        let mut rng = StdRng::seed_from_u64(7);
        for trial in 0..30 {
            let width = rng.random_range(1..18);
            let height = rng.random_range(1..15);
            let mut occ = vec![false; width * height];
            for cell in &mut occ {
                *cell = rng.random_bool(0.15);
            }
            if trial % 5 == 0 {
                occ[0] = true;
            }
            let dt = squared_distance_transform(&occ, width, height).unwrap();
            assert_close(&dt, &brute_force(&occ, width, height));
        }
    }
}

//! Total order over the costs accepted by Dijkstra and A*.
//!
//! The searches store costs in a binary heap, so they need a total order.
//! Integer costs already have one. Floating-point costs do not, because `NaN`
//! is incomparable. [`SearchCost`] is the contract the planners actually need:
//! non-negative edge weights, a checked addition that refuses `NaN`, and a
//! total order.
//!
//! `NaN` is never a valid cost. Infinities are allowed and mean "no finite
//! path"; they compare in the usual way (`-∞ < finite < +∞`).

use std::cmp::Ordering;
use std::fmt::Debug;

/// Cost type for [`crate::Dijkstra`] and [`crate::AStar`].
///
/// Implement this for a custom non-negative weight. The primitive integers and
/// both floating-point types already implement it.
pub trait SearchCost: Copy + Debug {
    /// The cost of an empty path.
    fn zero() -> Self;

    /// `true` when `self` is strictly less than zero.
    ///
    /// Edge weights must not be negative. A negative heuristic is allowed and
    /// is the caller's responsibility (it can make A* inadmissible).
    fn is_negative(self) -> bool;

    /// `false` only for `NaN`. Infinities are finite costs in the sense that
    /// they still participate in the order.
    fn is_finite_cost(self) -> bool;

    /// `self + rhs`, or `None` when the sum is not a usable cost.
    ///
    /// Integer implementations use checked addition and return `None` on
    /// overflow instead of panicking or wrapping. Floating-point implementations
    /// return `None` when either side is `NaN` or the sum is `NaN`
    /// (`∞ + -∞`).
    fn checked_add(self, rhs: Self) -> Option<Self>;

    /// Total order used by the open set.
    ///
    /// For integers this is the ordinary order. For floats, `NaN` compares
    /// equal to `NaN` and greater than every non-`NaN` value so a stray `NaN`
    /// cannot poison the heap; callers should still treat `NaN` as
    /// [`crate::PathPlannerError::InvalidCost`] before insertion.
    fn cmp_cost(self, other: &Self) -> Ordering;
}

macro_rules! impl_int_cost {
    (unsigned $($t:ty),* $(,)?) => {$(
        impl SearchCost for $t {
            #[inline]
            fn zero() -> Self { 0 }
            #[inline]
            fn is_negative(self) -> bool { false }
            #[inline]
            fn is_finite_cost(self) -> bool { true }
            #[inline]
            fn checked_add(self, rhs: Self) -> Option<Self> {
                let (sum, overflow) = self.overflowing_add(rhs);
                if overflow { None } else { Some(sum) }
            }
            #[inline]
            fn cmp_cost(self, other: &Self) -> Ordering { self.cmp(other) }
        }
    )*};
    (signed $($t:ty),* $(,)?) => {$(
        impl SearchCost for $t {
            #[inline]
            fn zero() -> Self { 0 }
            #[inline]
            fn is_negative(self) -> bool { self < 0 }
            #[inline]
            fn is_finite_cost(self) -> bool { true }
            #[inline]
            fn checked_add(self, rhs: Self) -> Option<Self> {
                let (sum, overflow) = self.overflowing_add(rhs);
                if overflow { None } else { Some(sum) }
            }
            #[inline]
            fn cmp_cost(self, other: &Self) -> Ordering { self.cmp(other) }
        }
    )*};
}

impl_int_cost!(unsigned u8, u16, u32, u64, u128, usize);
impl_int_cost!(signed i8, i16, i32, i64, i128, isize);

fn cmp_float(a: f64, b: f64) -> Ordering {
    match a.partial_cmp(&b) {
        Some(order) => order,
        None => a.is_nan().cmp(&b.is_nan()),
    }
}

impl SearchCost for f64 {
    #[inline]
    fn zero() -> Self {
        0.0
    }

    #[inline]
    fn is_negative(self) -> bool {
        self < 0.0
    }

    #[inline]
    fn is_finite_cost(self) -> bool {
        !self.is_nan()
    }

    #[inline]
    fn checked_add(self, rhs: Self) -> Option<Self> {
        if !self.is_finite_cost() || !rhs.is_finite_cost() {
            return None;
        }
        let sum = self + rhs;
        if sum.is_nan() { None } else { Some(sum) }
    }

    #[inline]
    fn cmp_cost(self, other: &Self) -> Ordering {
        cmp_float(self, *other)
    }
}

impl SearchCost for f32 {
    #[inline]
    fn zero() -> Self {
        0.0
    }

    #[inline]
    fn is_negative(self) -> bool {
        self < 0.0
    }

    #[inline]
    fn is_finite_cost(self) -> bool {
        !self.is_nan()
    }

    #[inline]
    fn checked_add(self, rhs: Self) -> Option<Self> {
        if !self.is_finite_cost() || !rhs.is_finite_cost() {
            return None;
        }
        let sum = self + rhs;
        if sum.is_nan() { None } else { Some(sum) }
    }

    #[inline]
    fn cmp_cost(self, other: &Self) -> Ordering {
        cmp_float(f64::from(self), f64::from(*other))
    }
}

/// Validate an edge weight and add it to a path cost.
pub(crate) fn add_edge<C: SearchCost>(g: C, edge: C) -> Result<C, crate::errors::PathPlannerError> {
    use crate::errors::PathPlannerError;
    if !edge.is_finite_cost() {
        return Err(PathPlannerError::InvalidCost);
    }
    if edge.is_negative() {
        return Err(PathPlannerError::NegativeCost);
    }
    g.checked_add(edge).ok_or(PathPlannerError::InvalidCost)
}

/// Add a heuristic to a path cost. Negative heuristics are permitted.
pub(crate) fn add_heuristic<C: SearchCost>(
    g: C,
    h: C,
) -> Result<C, crate::errors::PathPlannerError> {
    use crate::errors::PathPlannerError;
    if !h.is_finite_cost() {
        return Err(PathPlannerError::InvalidCost);
    }
    g.checked_add(h).ok_or(PathPlannerError::InvalidCost)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn float_order_is_total_and_rejects_nan_addition() {
        assert_eq!(1.5_f64.cmp_cost(&1.5), Ordering::Equal);
        assert_eq!(1.0_f64.cmp_cost(&2.0), Ordering::Less);
        assert_eq!(f64::INFINITY.cmp_cost(&1.0), Ordering::Greater);
        assert_eq!(f64::NAN.cmp_cost(&f64::NAN), Ordering::Equal);
        assert!(1.0_f64.checked_add(2.5).unwrap() < 4.0);
        assert!(1.0_f64.checked_add(f64::NAN).is_none());
        assert!(f64::INFINITY.checked_add(f64::NEG_INFINITY).is_none());
        assert!(1.0_f64.checked_add(f64::INFINITY).unwrap().is_infinite());
    }

    #[test]
    fn integer_overflow_is_reported() {
        assert!(u32::MAX.checked_add(1).is_none());
        assert_eq!(3_u32.checked_add(4), Some(7));
        assert!(!10_u32.is_negative());
        assert!((-1_i32).is_negative());
    }
}

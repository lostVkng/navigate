//! Fast hash maps shared by the graph searches.
//!
//! [`FxIndexMap`] keeps insertion order (so parent indices stay stable) and uses
//! the `rustc_hash` hasher, which is quicker than the default hasher for the
//! small integer and string keys these planners use.

use std::hash::BuildHasherDefault;

use indexmap::IndexMap;
use rustc_hash::FxHasher;

/// Insertion-ordered map with a fast non-cryptographic hasher.
pub(crate) type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

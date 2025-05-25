use std::hash::BuildHasherDefault;
use indexmap::{IndexMap};
use rustc_hash::FxHasher;


/// Use indexmap for fast lookups and rustc_hash for fast hashing
pub(crate) type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;
//! Union-Find (Disjoint Set Union) data structure.
//!
//! This module provides a generic Union-Find implementation with path compression
//! for efficient grouping operations. Used by route grouping algorithms.

use std::collections::{BTreeMap, HashMap};
use std::hash::Hash;

/// Union-Find data structure with path compression.
///
/// Provides near-constant time operations for:
/// - Finding the representative (root) of a set
/// - Unioning two sets together
///
/// # Example
/// ```
/// use tracematch::union_find::UnionFind;
///
/// let mut uf: UnionFind<i32> = UnionFind::new();
/// uf.make_set(1);
/// uf.make_set(2);
/// uf.make_set(3);
///
/// uf.union(&1, &2);
/// assert_eq!(uf.find(&1), uf.find(&2));
/// assert_ne!(uf.find(&1), uf.find(&3));
/// ```
#[derive(Debug, Clone)]
pub struct UnionFind<T: Eq + Hash + Clone + Ord> {
    parent: HashMap<T, T>,
    rank: HashMap<T, usize>,
}

impl<T: Eq + Hash + Clone + Ord> Default for UnionFind<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Eq + Hash + Clone + Ord> UnionFind<T> {
    /// Create a new empty Union-Find structure.
    pub fn new() -> Self {
        Self {
            parent: HashMap::new(),
            rank: HashMap::new(),
        }
    }

    /// Create a Union-Find with pre-allocated capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            parent: HashMap::with_capacity(capacity),
            rank: HashMap::with_capacity(capacity),
        }
    }

    /// Add a new element as its own set.
    pub fn make_set(&mut self, item: T) {
        if !self.parent.contains_key(&item) {
            self.parent.insert(item.clone(), item.clone());
            self.rank.insert(item, 0);
        }
    }

    /// Find the representative (root) of the set containing `item`.
    /// Uses path compression for efficiency.
    ///
    /// Returns the item itself if not in the structure (auto-creates set).
    pub fn find(&mut self, item: &T) -> T {
        // Auto-create if not exists
        if !self.parent.contains_key(item) {
            self.parent.insert(item.clone(), item.clone());
            self.rank.insert(item.clone(), 0);
            return item.clone();
        }

        let current = self.parent.get(item).cloned().unwrap();
        if &current == item {
            return item.clone();
        }

        // Path compression: recursively find root and update parent
        let root = self.find(&current);
        self.parent.insert(item.clone(), root.clone());
        root
    }

    /// Union the sets containing `a` and `b`.
    /// Uses union by rank for efficiency.
    ///
    /// Returns true if the sets were different (union performed),
    /// false if they were already in the same set.
    pub fn union(&mut self, a: &T, b: &T) -> bool {
        let root_a = self.find(a);
        let root_b = self.find(b);

        if root_a == root_b {
            return false;
        }

        // Union by rank: attach smaller tree under larger tree
        let rank_a = *self.rank.get(&root_a).unwrap_or(&0);
        let rank_b = *self.rank.get(&root_b).unwrap_or(&0);

        if rank_a < rank_b {
            self.parent.insert(root_a, root_b);
        } else if rank_a > rank_b {
            self.parent.insert(root_b, root_a);
        } else {
            self.parent.insert(root_b, root_a.clone());
            self.rank.insert(root_a, rank_a + 1);
        }

        true
    }

    /// Check if two elements are in the same set.
    pub fn connected(&mut self, a: &T, b: &T) -> bool {
        self.find(a) == self.find(b)
    }

    /// Get all unique groups as a map from root -> members.
    ///
    /// Ordered by root, and members within each group are sorted, so a caller
    /// that folds or enumerates the result gets the same answer on every run.
    /// A `HashMap` here would carry a per-construction hash seed into whatever
    /// the caller derives from the order.
    pub fn groups(&mut self) -> BTreeMap<T, Vec<T>> {
        let mut items: Vec<T> = self.parent.keys().cloned().collect();
        items.sort();
        let mut groups: BTreeMap<T, Vec<T>> = BTreeMap::new();

        for item in items {
            let root = self.find(&item);
            groups.entry(root).or_default().push(item);
        }

        for members in groups.values_mut() {
            members.sort();
        }

        groups
    }

    /// Get the number of elements in the structure.
    pub fn len(&self) -> usize {
        self.parent.len()
    }

    /// Check if the structure is empty.
    pub fn is_empty(&self) -> bool {
        self.parent.is_empty()
    }
}

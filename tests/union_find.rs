//! Tests for union_find module

use tracematch::union_find::{self, UnionFind};

#[test]
fn test_basic_operations() {
    let mut uf: UnionFind<i32> = UnionFind::new();

    uf.make_set(1);
    uf.make_set(2);
    uf.make_set(3);

    assert!(!uf.connected(&1, &2));

    uf.union(&1, &2);
    assert!(uf.connected(&1, &2));
    assert!(!uf.connected(&1, &3));
}

#[test]
fn test_path_compression() {
    let mut uf: UnionFind<i32> = UnionFind::new();

    // Create chain: 1 -> 2 -> 3 -> 4
    uf.make_set(1);
    uf.make_set(2);
    uf.make_set(3);
    uf.make_set(4);

    uf.union(&1, &2);
    uf.union(&2, &3);
    uf.union(&3, &4);

    // After find, all should point to same root
    let root = uf.find(&1);
    assert_eq!(uf.find(&2), root);
    assert_eq!(uf.find(&3), root);
    assert_eq!(uf.find(&4), root);
}

#[test]
fn test_groups() {
    let mut uf: UnionFind<String> = UnionFind::new();

    uf.make_set("a".to_string());
    uf.make_set("b".to_string());
    uf.make_set("c".to_string());
    uf.make_set("d".to_string());

    uf.union(&"a".to_string(), &"b".to_string());
    uf.union(&"c".to_string(), &"d".to_string());

    let groups = uf.groups();
    assert_eq!(groups.len(), 2);
}

#[test]
fn test_string_helpers() {
    let ids = vec!["route-1", "route-2", "route-3"];
    let mut uf = union_find::string_uf::from_ids(ids);

    union_find::string_uf::union(&mut uf, "route-1", "route-2");

    assert_eq!(
        union_find::string_uf::find(&mut uf, "route-1"),
        union_find::string_uf::find(&mut uf, "route-2")
    );
}

#[test]
fn test_groups_deterministic() {
    // Run multiple times - results should be identical
    let results: Vec<_> = (0..5)
        .map(|_| {
            let mut uf: UnionFind<String> = UnionFind::new();

            // Add in different order internally (HashMap iteration is random)
            uf.make_set("d".to_string());
            uf.make_set("a".to_string());
            uf.make_set("c".to_string());
            uf.make_set("b".to_string());

            uf.union(&"a".to_string(), &"b".to_string());
            uf.union(&"c".to_string(), &"d".to_string());

            uf.groups()
        })
        .collect();

    // All results should be identical
    for i in 1..results.len() {
        // Check same number of groups
        assert_eq!(
            results[0].len(),
            results[i].len(),
            "Different group counts on run {i}"
        );

        // Check each group has same members (sorted)
        for (root, members) in &results[0] {
            let other_members = results[i].get(root);
            assert!(other_members.is_some(), "Missing group {root} on run {i}");
            assert_eq!(
                members,
                other_members.unwrap(),
                "Different members for group {root} on run {i}"
            );
        }
    }
}

#[test]
fn test_groups_members_sorted() {
    let mut uf: UnionFind<String> = UnionFind::new();

    // Add in reverse alphabetical order
    uf.make_set("z".to_string());
    uf.make_set("m".to_string());
    uf.make_set("a".to_string());

    uf.union(&"z".to_string(), &"a".to_string());
    uf.union(&"z".to_string(), &"m".to_string());

    let groups = uf.groups();
    assert_eq!(groups.len(), 1);

    // Get the single group's members
    let members = groups.values().next().unwrap();

    // Members should be sorted
    let mut sorted = members.clone();
    sorted.sort();
    assert_eq!(
        members, &sorted,
        "Members should be sorted, got {:?}",
        members
    );
}

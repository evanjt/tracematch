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

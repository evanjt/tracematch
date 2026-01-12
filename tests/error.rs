//! Tests for error module

use tracematch::error::{OptionExt, RouteMatchError};

#[test]
fn test_error_display() {
    let err = RouteMatchError::InsufficientPoints {
        activity_id: "test-1".to_string(),
        point_count: 1,
        minimum_required: 2,
    };
    assert!(err.to_string().contains("test-1"));
    assert!(err.to_string().contains("1 points"));
}

#[test]
fn test_option_ext() {
    let none: Option<i32> = None;
    let result = none.ok_or_insufficient_points("test", 0, 2);
    assert!(matches!(
        result,
        Err(RouteMatchError::InsufficientPoints { .. })
    ));
}

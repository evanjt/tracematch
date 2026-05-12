import tracematch


def make_line(start_lat, start_lon, count=20, step=0.001):
    return [tracematch.GpsPoint(start_lat + i * step, start_lon + i * step) for i in range(count)]


def test_gps_point_repr():
    p = tracematch.GpsPoint(51.5, -0.1)
    assert "51.5" in repr(p)
    assert p.lat == 51.5
    assert p.lon == -0.1
    assert p.elevation is None


def test_gps_point_with_elevation():
    p = tracematch.GpsPoint(51.5, -0.1, elevation=100.0)
    assert p.elevation == 100.0


def test_signature_creation():
    points = make_line(51.5, -0.1)
    sig = tracematch.RouteSignature.from_points("test-1", points)
    assert sig.activity_id == "test-1"
    assert sig.total_distance > 0
    assert sig.num_points >= 2


def test_signature_insufficient_points():
    try:
        tracematch.RouteSignature.from_points("test", [tracematch.GpsPoint(0, 0)])
        assert False, "Should have raised ValueError"
    except ValueError:
        pass


def test_compare_identical_routes():
    points = make_line(51.5, -0.1)
    sig1 = tracematch.RouteSignature.from_points("a", points)
    sig2 = tracematch.RouteSignature.from_points("b", points)
    result = tracematch.compare_routes(sig1, sig2)
    assert result is not None
    assert result.match_percentage == 100.0
    assert result.direction == "same"


def test_compare_different_routes():
    sig1 = tracematch.RouteSignature.from_points("a", make_line(51.5, -0.1))
    sig2 = tracematch.RouteSignature.from_points("b", make_line(40.7, -74.0))
    result = tracematch.compare_routes(sig1, sig2)
    assert result is None


def test_group_routes():
    london1 = tracematch.RouteSignature.from_points("l1", make_line(51.5, -0.1))
    london2 = tracematch.RouteSignature.from_points("l2", make_line(51.5, -0.1))
    nyc = tracematch.RouteSignature.from_points("n1", make_line(40.7, -74.0))

    groups = tracematch.group_routes([london1, london2, nyc])
    assert len(groups) == 2

    sizes = sorted([len(g.activity_ids) for g in groups])
    assert sizes == [1, 2]


def test_match_config_defaults():
    config = tracematch.MatchConfig()
    sig1 = tracematch.RouteSignature.from_points("a", make_line(51.5, -0.1), config)
    assert sig1 is not None


def test_match_config_custom():
    config = tracematch.MatchConfig(min_match_percentage=90.0)
    sig1 = tracematch.RouteSignature.from_points("a", make_line(51.5, -0.1), config)
    sig2 = tracematch.RouteSignature.from_points("b", make_line(51.5, -0.1), config)
    result = tracematch.compare_routes(sig1, sig2, config)
    assert result is not None

"""Renumbering a later section dump onto an earlier dump's ids.

Run from the tracematch crate root:

    python3 -m unittest discover -s tests -p 'test_section_renumber.py'
"""
import math
import unittest

from section_renumber import pair

METRES_PER_DEGREE = math.radians(1) * 6371000.0


def point(north_m, east_m):
    return [north_m / METRES_PER_DEGREE, east_m / METRES_PER_DEGREE]


def east(start_m, length_m, vertices):
    """A line running east along the equator, vertices spread evenly over length_m."""
    step = length_m / (vertices - 1)
    return [point(0, start_m + k * step) for k in range(vertices)]


def riders(prefix, n):
    return [f'{prefix}{k}' for k in range(1, n + 1)]


def section(sid, members, polyline, line_hash, length_m, sport='Paddle'):
    return {
        'id': sid,
        'sport': sport,
        'line_hash': line_hash,
        'polyline': polyline,
        'members': members,
        'distance_m': length_m,
    }


class CarryIdByLineGeometry(unittest.TestCase):
    def test_line_takes_the_id_of_the_line_it_lies_on_not_of_the_members_it_shares(self):
        # Scenario: drift before the later dump thinned A to 10 members, and the
        # later A' lies on A while taking 13 of B's members. B' lies on B, 1.3 km away.
        before = [
            section('sec_paddle_1', riders('a', 10), east(0, 1083, 10), 'hash-a', 1083),
            section('sec_paddle_2', riders('b', 28), east(2400, 198, 5), 'hash-b', 198),
        ]
        after = [
            section('sec_paddle_1', riders('a', 7) + riders('b', 13), east(0, 1357, 20), 'hash-a2', 1357),
            section('sec_paddle_2', riders('b', 26) + ['x1', 'x2'], east(2400, 198, 5), 'hash-b2', 198),
        ]

        records, notes = pair(before, after)

        self.assertEqual([r['id'] for r in records], ['sec_paddle_1', 'sec_paddle_2'])
        self.assertIn('sec_paddle_1 -> sec_paddle_1 by line overlap 1.00', notes)
        self.assertIn('sec_paddle_2 -> sec_paddle_2 by line overlap 1.00', notes)

    def test_line_matching_no_earlier_line_takes_a_fresh_id_despite_shared_members(self):
        before = [section('sec_paddle_1', riders('c', 10), east(0, 900, 10), 'hash-c', 900)]
        after = [
            section('sec_paddle_1', riders('c', 8) + ['d1', 'd2'], east(5000, 900, 10), 'hash-d', 900),
        ]

        records, notes = pair(before, after)

        self.assertEqual(records[0]['id'], 'sec_paddle_2')
        self.assertIn('sec_paddle_1 -> sec_paddle_2 (new line, 10 members)', notes)
        self.assertIn('sec_paddle_1 dropped (10 members, 900 m)', notes)

    def test_half_of_a_line_on_the_other_is_enough_to_carry_its_id(self):
        before = [section('sec_paddle_1', riders('e', 10), east(0, 900, 10), 'hash-e', 900)]
        # Two of the four vertices lie on the earlier line, two a kilometre north of it.
        half = [point(0, 0), point(0, 100), point(1100, 100), point(1100, 0)]
        after = [section('sec_paddle_1', ['f1', 'f2', 'f3', 'f4'], half, 'hash-f', 1100)]

        records, notes = pair(before, after)

        self.assertEqual(records[0]['id'], 'sec_paddle_1')
        self.assertIn('sec_paddle_1 -> sec_paddle_1 by line overlap 0.50', notes)

    def test_under_half_of_a_line_on_the_other_takes_a_fresh_id(self):
        before = [section('sec_paddle_1', riders('e', 10), east(0, 900, 10), 'hash-e', 900)]
        # One of the four vertices lies on the earlier line.
        sliver = [point(0, 0), point(1100, 0), point(1100, 100), point(1100, 200)]
        after = [section('sec_paddle_1', ['f1', 'f2', 'f3', 'f4'], sliver, 'hash-g', 1100)]

        records, notes = pair(before, after)

        self.assertEqual(records[0]['id'], 'sec_paddle_2')

    def test_empty_later_dump_drops_every_earlier_line(self):
        before = [section('sec_paddle_1', riders('e', 10), east(0, 900, 10), 'hash-e', 900)]

        records, notes = pair(before, [])

        self.assertEqual(records, [])
        self.assertEqual(notes, ['sec_paddle_1 dropped (10 members, 900 m)'])


if __name__ == '__main__':
    unittest.main()

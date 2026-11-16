"""Direction agreement of a dump's member rows with the detector's fresh pass count.

Run from the tracematch crate root:

    python3 -m unittest discover -s tests -p 'test_section_spatial.py'
"""
import os
import tempfile
import unittest

from section_spatial import direction_line, row_directions, write_summary


class RowDirections(unittest.TestCase):
    def test_row_whose_direction_matches_its_fresh_pass_agrees(self):
        # Scenario: a pin row reads Same over indices 10..20, and the detector's
        # pass over the stored line covers the same stretch Same.
        self.assertEqual(row_directions([(10, 20, 'Same')], [(10, 20, 'Same', 40.0)]), ['agree'])

    def test_row_whose_direction_differs_from_its_fresh_pass_differs(self):
        self.assertEqual(row_directions([(10, 20, 'Same')], [(10, 20, 'Reverse', 40.0)]), ['differ'])

    def test_extent_difference_alone_is_not_a_direction_difference(self):
        # The fold keeps rows over the pre-clip line, so the pin row can reach
        # further than the fresh pass without disagreeing in direction.
        self.assertEqual(row_directions([(10, 20, 'Reverse')], [(12, 25, 'Reverse', 30.0)]), ['agree'])

    def test_row_with_no_overlapping_pass_has_no_verdict(self):
        self.assertEqual(row_directions([(10, 20, 'Same')], [(30, 40, 'Same', 12.0)]), [None])

    def test_row_with_no_passes_at_all_has_no_verdict(self):
        self.assertEqual(row_directions([(10, 20, 'Same')], []), [None])

    def test_row_is_judged_by_the_pass_it_overlaps_most(self):
        # The short Reverse pass grazes the row's start, the Same pass covers
        # most of it, so the row agrees with the Same pass.
        rows = [(10, 30, 'Same')]
        passes = [(0, 11, 'Reverse', 5.0), (12, 30, 'Same', 18.0)]
        self.assertEqual(row_directions(rows, passes), ['agree'])

    def test_pass_sharing_only_an_end_index_overlaps_the_row(self):
        self.assertEqual(row_directions([(10, 20, 'Same')], [(20, 30, 'Reverse', 10.0)]), ['differ'])

    def test_each_row_is_judged_on_its_own(self):
        rows = [(0, 9, 'Same'), (10, 19, 'Reverse'), (30, 39, 'Same')]
        passes = [(0, 9, 'Same', 9.0), (10, 19, 'Same', 9.0)]
        self.assertEqual(row_directions(rows, passes), ['agree', 'differ', None])


class DirectionLine(unittest.TestCase):
    def test_all_rows_agreeing_reads_zero_differing(self):
        self.assertEqual(
            direction_line(['agree', 'agree']),
            'direction against the fresh pass count: 0 of 2 member rows differ (2 agree, 0 with no overlapping pass)',
        )

    def test_no_rows_reads_zero_of_zero(self):
        self.assertEqual(
            direction_line([]),
            'direction against the fresh pass count: 0 of 0 member rows differ (0 agree, 0 with no overlapping pass)',
        )

    def test_differing_and_unmatched_rows_are_counted_apart(self):
        self.assertEqual(
            direction_line(['agree', 'differ', 'differ', None]),
            'direction against the fresh pass count: 2 of 4 member rows differ (1 agree, 1 with no overlapping pass)',
        )


class SummaryLine(unittest.TestCase):
    def test_summary_file_carries_the_direction_line(self):
        with tempfile.TemporaryDirectory() as out_dir:
            write_summary([], [], [], out_dir, 0, 0, [], ['agree', 'differ', 'agree'])
            with open(os.path.join(out_dir, 'summary.txt')) as f:
                text = f.read()
        self.assertIn(
            'direction against the fresh pass count: 1 of 3 member rows differ (2 agree, 0 with no overlapping pass)\n',
            text,
        )


if __name__ == '__main__':
    unittest.main()

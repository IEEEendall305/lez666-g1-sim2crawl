"""Offline tests for crawl_sequencer.py (no simulator needed)."""
import unittest
from crawl_sequencer import CrawlSequencer, SEQ_CONFIG, CRAWL, IDLE


class T(unittest.TestCase):
    def test_crawl_waits_then_switches_when_settled(self):
        s = CrawlSequencer()
        self.assertEqual(s.start_crawl(0.0), "policy:" + SEQ_CONFIG["crawl_start_policy"])
        self.assertIsNone(s.update(1.0, 0.2, 0.0))          # too early even if settled
        self.assertIsNone(s.update(5.0, 0.7, 0.0))          # still tall
        self.assertIsNone(s.update(5.0, 0.3, 2.0))          # still moving
        self.assertEqual(s.update(5.0, 0.3, 0.1), "policy:" + SEQ_CONFIG["crawl_policy"])
        self.assertEqual(s.state, CRAWL)

    def test_crawl_timeout(self):
        s = CrawlSequencer(); s.start_crawl(0.0)
        self.assertEqual(s.update(13.0, 0.9, 9.0), "policy:" + SEQ_CONFIG["crawl_policy"])

    def test_stand_sequence(self):
        s = CrawlSequencer()
        self.assertEqual(s.start_stand(0.0), "mode:default_pos")
        self.assertTrue(s.update(0.1, 0.3, 0.0).startswith("ramp_rate:"))
        self.assertIsNone(s.update(5.0, 0.3, 0.0))
        self.assertEqual(s.update(13.0, 0.3, 0.0), "done")
        self.assertEqual(s.state, IDLE)


if __name__ == "__main__":
    unittest.main()

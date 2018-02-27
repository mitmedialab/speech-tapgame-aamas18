# -*- coding: utf-8 -*-

from .. import TapGameFSM

import unittest


class BasicTestSuite(unittest.TestCase):
    """Basic test cases."""

    def test_absolute_truth_and_meaning(self):
        assert True

    def test_imports(self):
        assert (not TapGameFSM is None)



unittest.main()
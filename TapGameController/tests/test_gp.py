# -*- coding: utf-8 -*-

from ..StudentWordModel import StudentWordModel
from ..AgentModel import ActionSpace

from GameUtils import Curriculum
import numpy as np

import unittest


class GPTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_GP(self):
        my_GP = StudentWordModel()
        valid_words = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]
        self.assertEqual(my_GP.get_next_best_word(ActionSpace.RING_ANSWER_CORRECT) in valid_words, True)


    def test_kernel(self):
        my_GP = StudentWordModel()

        self.assertGreater(my_GP.get_word_cov('BOAT', 'GOAT'), my_GP.get_word_cov('BOAT', 'BROOM'))

    def graph_test(self):
        my_GP = StudentWordModel()
        Xtrain = ['BEE', 'SNAKE', 'TIGER']  # these numbers are just labels
        Ytrain = [.66, .66, .66]  # these numbers correspond to 'Correct' demonstrations

        my_GP.train_and_compute_posterior(Xtrain, Ytrain)
        my_GP.plot_curricular_distro()

        Xtrain = ['BEE', 'SNAKE', 'TIGER']  # these numbers are just labels
        Ytrain = [1, 1, 1]  # these numbers correspond to 'Correct' demonstrations

        my_GP.train_and_compute_posterior(Xtrain, Ytrain)
        my_GP.plot_curricular_distro()




unittest.main()

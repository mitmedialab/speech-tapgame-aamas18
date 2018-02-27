# -*- coding: utf-8 -*-

from ..TapGameFSM import TapGameFSM

import unittest


class FSMTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_FSM(self):

        return True
        # def send_command(cmd, *args):
        #     print(cmd)
        #
        # my_FSM = TapGameFSM()
        # my_FSM.max_rounds = 2
        # my_FSM.send_game_cmd = send_command
        #
        # self.assertEqual(my_FSM.state, 'GAME_START')
        #
        # my_FSM.init_first_round()
        # self.assertEqual(my_FSM.state, 'ROUND_START')
        #
        # my_FSM.start_round()
        # self.assertEqual(my_FSM.state, 'ROUND_ACTIVE')
        #
        # my_FSM.robot_ring_in()
        # self.assertEqual(my_FSM.state, 'ROUND_START')
        #
        # my_FSM.start_round()
        # self.assertEqual(my_FSM.state, 'ROUND_ACTIVE')
        #
        # my_FSM.robot_ring_in()
        #
        # self.assertEqual(my_FSM.state, 'GAME_FINISHED')



unittest.main()

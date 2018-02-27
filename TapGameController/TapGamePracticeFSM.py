"""
This is the main FSM / Game Logic class for the Tap Game
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
import time
import _thread as thread
from random import randint

from transitions import Machine

from GameUtils import GlobalSettings
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
from GameUtils.AudioRecorder import AudioRecorder
from .AgentModel import ActionSpace
from .AgentModel import AgentModel
from .ROSNodeMgr import ROSNodeMgr
from .StudentWordModel import StudentWordModel
from .RobotBehaviorList import RobotBehaviors

if GlobalSettings.USE_ROS:
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand

RECORD_TIME_MS = 3500
SHOW_RESULTS_TIME_MS = 2500
WAIT_TO_BUZZ_TIME_MS = 1500 #note, game currently waits 3000ms after receiving message
SIMULATED_ROBOT_RESULTS_TIME_MS = 2500 # time to wait while we "process" robot speech (should be close to SpeechAce roundtrip time)

PASSING_SCORE_THRESHOLD = .65

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN,
                    TapGameLog.PLAYER_RING_IN, TapGameLog.END_ROUND_DONE,
                    TapGameLog.RESET_NEXT_ROUND_DONE, TapGameLog.SHOW_GAME_END_DONE,
                    TapGameLog.PLAYER_BEAT_ROBOT, TapGameLog.RESTART_GAME]


class TapGamePracticeFSM: # pylint: disable=no-member, too-many-instance-attributes
    """
    An FSM for the Tap Game. Contains Game Logic and some nodes for interacting w the Unity "View"
    """

    round_index = 0
    max_rounds = 6 #game ends after this many rounds

    player_score = 0
    robot_score = 0

    agent_model = AgentModel()
    pronunciation_utils = PronunciationUtils()
    ros_node_mgr = ROSNodeMgr()
    current_round_word = ""
    current_round_action = None

    audio_file = None

    letters = None # the graphemes/letters under consideration this rd
    passed = None # whether each letter corresponded to a phoneme above the threshold
    scores = None # the actual score of the phoneme each letter corresponds to

    player_won_round_tap = None # did the player buzz in first
    player_passed_round = None # was the players score high enough to pass the round


    states = ['GAME_START', 'ROUND_START', 'ROUND_ACTIVE',
              'PLAYER_PRONOUNCE', 'ROBOT_PRONOUNCE', 'SHOW_RESULTS', 'ROUND_RESOLVE',
              'GAME_FINISHED']

    # Scripted Sequence of Actions and Words to use in Game
    practice_actions = [ActionSpace.DONT_RING, ActionSpace.DONT_RING, ActionSpace.DONT_RING,
                        ActionSpace.RING_ANSWER_CORRECT, ActionSpace.RING_ANSWER_CORRECT, ActionSpace.DONT_RING,
                        ActionSpace.RING_ANSWER_CORRECT]              

    practice_words = ["FORK", "FROG", "FISH", "DUCK", "SUN", "RABBIT", "DAD"]


    transitions = [
        {'trigger': 'init_first_round',
         'source': 'GAME_START',
         'dest': 'ROUND_START',
         'after': 'on_init_first_round'},

        {'trigger': 'start_round',
         'source': 'ROUND_START',
         'dest': 'ROUND_ACTIVE',
         'after': 'on_start_round'},

        {'trigger': 'robot_ring_in',
         'source': 'ROUND_ACTIVE',
         'dest': 'ROBOT_PRONOUNCE',
         'after': 'on_robot_ring_in'},

        {'trigger': 'player_ring_in',
         'source': 'ROUND_ACTIVE',
         'dest': 'PLAYER_PRONOUNCE',
         'after': 'on_player_ring_in'},

        {'trigger': 'player_pronounce_eval',
         'source': 'PLAYER_PRONOUNCE',
         'dest': 'SHOW_RESULTS',
         'after': 'on_player_pronounce_eval'},

        {'trigger': 'robot_pronounce_eval',
         'source': 'ROBOT_PRONOUNCE',
         'dest': 'SHOW_RESULTS',
         'after': 'on_robot_pronounce_eval'},

         {'trigger': 'resolve_round',
         'source': 'SHOW_RESULTS',
         'dest': 'ROUND_RESOLVE',         
         'after': 'on_round_resolve'},

        {'trigger': 'handle_round_end',
         'source': 'ROUND_RESOLVE',
         'dest': 'ROUND_START',
         'conditions': 'is_not_last_round',
         'after': 'on_reset_round'},

        {'trigger': 'handle_round_end',
         'source': 'ROUND_RESOLVE',
         'dest': 'GAME_FINISHED',
         'conditions': 'is_last_round',
         'after': 'on_game_finished'},

        {'trigger': 'replay_game',
         'source': 'GAME_FINISHED',
         'dest': 'GAME_START',
         'after': 'on_game_replay'},
    ]

    def __init__(self, participant_id, experimenter_name, experiment_phase):

        self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
                                     initial='GAME_START')

        self.participant_id = participant_id
        self.experimenter_name = experimenter_name
        self.experiment_phase = experiment_phase

        if not self.experiment_phase == 'practice':
            print(str(self.experiment_phase) + " was not 'practice'")
            exit()

        # Initializes a new audio recorder object if one hasn't been created
        self.recorder = AudioRecorder(self.participant_id, self.experimenter_name, self.experiment_phase)

        # Tell robot to look at Tablet
        self.ros_node_mgr.init_ros_node()

    def on_init_first_round(self):
        """
        Called when the game registers with the controller
        Should send msg to Unity game telling it the word to load for the first round
        """
        print("got to init_first round!")
        # get the next robot action
        self.current_round_action = self.practice_actions[self.round_index]


        #send message every 2s in case it gets dropped
        def send_msg_til_received():
            while(self.state == "ROUND_START"):
                self.current_round_word = self.practice_words[self.round_index]
                self.ros_node_mgr.send_game_cmd(TapGameCommand.INIT_ROUND,
                                        json.dumps(self.current_round_word))
                print('sent command!')
                print(self.state)
                time.sleep(5)

        thread.start_new_thread(send_msg_til_received, ())
        

    def on_start_round(self):
        """
        Called when the game registers that the round initialization is done
        Should send msg to Unity game telling it to begin countdown and make buzzers active
        """
        print('got to start round cb')
        self.ros_node_mgr.send_game_cmd(TapGameCommand.START_ROUND)

        if self.current_round_action == ActionSpace.RING_ANSWER_CORRECT:
           time.sleep(WAIT_TO_BUZZ_TIME_MS / 1000.0)
           self.ros_node_mgr.send_robot_cmd(RobotBehaviors.RING_ANSWER_CORRECT)
           self.ros_node_mgr.send_game_cmd(TapGameCommand.ROBOT_RING_IN)

    def on_robot_ring_in(self):
        """
        Called when the game registers that the robot 'buzzed in'
        Should send msg to Unity game telling it to load robot pronunciation screen
        """
        print('got to robot ring in cb')
        self.player_won_round_tap = False
        self.audio_file = 'None'

        # Send message to robot telling it to pronounce

        # Wait a few seconds, pronounce word, then wait again
        time.sleep((RECORD_TIME_MS / 2) / 1000.0)
        if self.current_round_action == ActionSpace.RING_ANSWER_CORRECT:
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.PRONOUNCE_CORRECT, self.current_round_word)
            self.letters = list(self.current_round_word)
            self.passed = ['1'] * len(self.letters) #robot always gets it right if intentional ring
            self.scores = [1] * len(self.letters) #robot always gets it right if intentional ring

        elif self.current_round_action == ActionSpace.LATE_RING:
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.PRONOUNCE_WRONG_SOUND)
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.PRONOUNCE_WRONG_SPEECH)
            self.letters = list(self.current_round_word)
            self.passed = ['0'] * len(self.letters) #robot always gets it wrong if late ring
            self.scores = [0] * len(self.letters) #robot always gets it right if intentional ring

        time.sleep((RECORD_TIME_MS / 2) / 1000.0)

        # Move to evaluation phase
        self.robot_pronounce_eval()

    def player_beat_robot(self):
        """
        This function details what happens when the robot wants to buzz, but gets beat by the human
        """        

        # print("PLAYER BEAT ROBOT TO THE PUNCH!")

        # tmp = [int(x) for x in self.passed]
        # passed_ratio = (sum(tmp) / len(tmp)) #TODO: do this over phonemes, not letters!
        # print("ROUND PASSED RATIO WAS" + str(passed_ratio))



    def on_player_ring_in(self):
        """
        Called when the human player has tapped their buzzer to ring in
        Should send msg to Unity game telling it to load the pronunciation screen
        And also start recording from the phone for 5 seconds + writing to wav
        """
        print('got to player ring in cb')
        self.player_won_round_tap = True
        

        #SEND SHOW_PRONUNCIATION_PAGE MSG
        self.recorder.start_recording(self.current_round_word, self.round_index, RECORD_TIME_MS)
        #time.sleep(RECORD_TIME_MS / 1000.0)
        self.recorder.stop_recording()

        ##Evaluates the action message

        ## If given a word to evaluate and done recording send the information to speechace
        if self.current_round_word and \
           self.recorder.has_recorded % 2 == 0 and\
           self.recorder.has_recorded != 0:

           # If you couldn't find the android audio topic, automatically pass
            # instead of using the last audio recording
            if not self.recorder.valid_recording:
                self.letters = list(self.current_round_word)
                self.passed = ['0'] * len(self.letters)
                self.scores = [0] * len(self.letters)

                print ("NO RECORDING SO YOU AUTOMATICALLY FAIL")
                self.audio_file = 'None'
            else: 
                self.audio_file = self.recorder.WAV_OUTPUT_FILENAME_PREFIX + self.current_round_word + '_' + str(self.recorder.recording_index) + '.wav'
                print("SENDING TO SPEECHACE")
                word_score_list = self.recorder.speechace(self.audio_file)
                print("WORD SCORE LIST")
                print(word_score_list)

                # if we didn't record, there will be no word score list
                if word_score_list:
                    for word_results in word_score_list:
                        print("Message for ROS")
                        self.letters, self.passed, self.scores = \
                            self.pronunciation_utils.process_speechace_word_results(word_results)
                        print(self.letters)
                        print(self.passed)
                        print(self.scores)
                else:
                    self.letters = list(self.current_round_word)
                    self.passed = ['0'] * len(self.letters)
                    self.scores = [0] * len(self.letters)
                    print('NO RECORDING, SO YOU AUTO-FAIL!!')

            self.player_pronounce_eval()
        else:
            print('THIS SHOULD NEVER HAPPEN')

    def on_player_pronounce_eval(self):
        """
        Called after the human player has pronounced their buzzer to ring in
        send wav from previous step to speech ace, get results, update model, and
        send message to game to display results
        """
        print('got to player pronounce eval cb')
        # Get the actual results here

        avg_phone_score = (sum(self.scores) / len(self.scores)) #TODO: do this over phonemes, not letters!
        print("AVG PHONE SCORE WAS" + str(avg_phone_score))        

        results_params = {}
        results_params['letters'] = self.letters
        results_params['passed'] = self.passed
        results_params['scores'] = [str(x) for x in self.scores] #convert to string before sending over json (ask Sam why)

        self.ros_node_mgr.send_game_cmd(TapGameCommand.SHOW_RESULTS, json.dumps(results_params))
        time.sleep(SHOW_RESULTS_TIME_MS / 1000.0)
        self.resolve_round()

    def on_robot_pronounce_eval(self):
        """
        Called after the robot has 'pronounced' a word. Should send mesage to Game telling it
        to show results
        handle_round_end() to transition to next round
        """

        time.sleep(SIMULATED_ROBOT_RESULTS_TIME_MS / 1000.0)

        results_params = {}
        results_params['letters'] = self.letters
        results_params['passed'] = self.passed
        results_params['scores'] = [str(x) for x in self.scores] #convert to string before sending over json (ask Sam why)

        self.ros_node_mgr.send_game_cmd(TapGameCommand.SHOW_RESULTS, json.dumps(results_params))

        time.sleep(SHOW_RESULTS_TIME_MS / 1000.0)
        self.resolve_round()

    def on_round_resolve(self):
        """
        Called after finishing a round in the game
        Should optionally send command for robot to react to result, then send cmd to game to reset for the next round
        """
        print('got to round reset')
        self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_CENTER)
        time.sleep(.5) #wait for half a second so the lookat can go through

        #ROBOT REACTION LOGIC  - consider using fidget text?      
        
        avg_phoneme_score = (sum(self.scores) / len(self.scores))
        print('AVG PHONEME SCORE WAS')
        print(avg_phoneme_score)

        if self.player_won_round_tap: #reactions to player winning tap
            if (not self.current_round_action == ActionSpace.DONT_RING): # if the robot intended to ring but got beaten,
                if avg_phoneme_score >= PASSING_SCORE_THRESHOLD: #if the robot was beaten and the child got the word right
                    self.player_score += 1
                    self.player_passed_round = True
                    self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_TO_BEAT_CORRECT)
                else:
                    self.player_passed_round = False
                    self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_TO_BEAT_WRONG)

            else: #regular response to human ring-in
                if avg_phoneme_score >= PASSING_SCORE_THRESHOLD: #if child got the word right
                    self.player_score += 1
                    self.player_passed_round = True
                    self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_PLAYER_CORRECT)
                else:
                    self.player_passed_round = False
                    self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_PLAYER_WRONG)

        else: #robot rang in
            if avg_phoneme_score >= PASSING_SCORE_THRESHOLD: #if robot got the word right
                self.robot_score += 1
                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_ROBOT_CORRECT)
                self.player_passed_round = False
            else:
                self.player_passed_round = False
                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_ROBOT_WRONG)                

        self.handle_round_end()       
        

    def on_reset_round(self):
        self.ros_node_mgr.send_game_cmd(TapGameCommand.RESET_NEXT_ROUND)

    def on_game_finished(self):
        """
        Called when we have completed 'max_rounds' rounds in a game.
        Sends msg to the Unity game to load the game end screen
        """
        print('got to game finished')
        time.sleep(1)
        if self.player_score >= self.robot_score:
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOSE_MOTION)
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOSE_SPEECH)
        else:
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.WIN_MOTION)
            self.ros_node_mgr.send_robot_cmd(RobotBehaviors.WIN_SPEECH)


        self.ros_node_mgr.send_game_cmd(TapGameCommand.SHOW_GAME_END)



    def on_game_replay(self):
        """
        Called when the player wants to replay the game after finishing.
        Sends msg to the Unity game to reset the game and start over
        """

        # reset all state variables (rounds, score)
        self.player_score = 0
        self.robot_score = 0
        self.init_first_round()

        #reset student model here if needed

        #self.ros_node_mgr.send_game_cmd(TapGameCommand.RESTART_GAME) #START GAME OVER
        #self.ros_node_mgr.send_game_cmd()


    def on_log_received(self, data):
        """
        Rospy Callback for when we get log messages
        """
        if data.message in FSM_LOG_MESSAGES:

            if data.message == TapGameLog.CHECK_IN:
                print('Game Checked in!')
                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_CENTER)
                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.SAY_HI)

            if data.message == TapGameLog.GAME_START_PRESSED:
                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_AT_TABLET)
                time.sleep(500 / 1000.0)
                self.init_first_round()  # makes state transition + calls self.on_init_first_round()

            if data.message == TapGameLog.INIT_ROUND_DONE:
                print('done initializing')
                self.start_round()

            if data.message == TapGameLog.START_ROUND_DONE:
                print('I heard Start Round DONE. Waiting for player input')                
                
            if data.message == TapGameLog.PLAYER_RING_IN:
                print('Player Rang in!')
                self.player_ring_in()

            if data.message == TapGameLog.ROBOT_RING_IN:
                print('Robot Rang in!')
                self.robot_ring_in()

            if data.message == TapGameLog.PLAYER_BEAT_ROBOT:
                self.player_beat_robot = True

            if data.message == TapGameLog.RESET_NEXT_ROUND_DONE:
                print('Game Done Resetting Round! Now initing new round')

                self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_AT_TABLET)
                #self.ros_node_mgr.publish_round_summary(self.round_index, self.current_round_action, self.current_round_word,
                #                                         self.player_won_round_tap, self.player_passed_round, self.audio_file, self.letters, self.scores,
                #                                         self.passed,self.player_score, self.robot_score, self.practice_words, 
                #                                         [.5 * len(self.practice_words)],[.3 * len(self.practice_words)])

                self.round_index += 1
                self.player_beat_robot = False
                self.current_round_action = self.practice_actions[self.round_index]
                self.current_round_word = self.practice_words[self.round_index]

                self.ros_node_mgr.send_game_cmd(TapGameCommand.INIT_ROUND, json.dumps(self.current_round_word))

            if data.message == TapGameLog.SHOW_GAME_END_DONE:
                print('GAME OVER! WAIT FOR RESET SINAL')

            if data.message == TapGameLog.RESTART_GAME:
                self.replay_game()
        else:
            print('NOT A REAL MESSAGE?!?!?!?')


    def is_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return (self.round_index >= self.max_rounds)

    def is_not_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return not self.is_last_round()
            


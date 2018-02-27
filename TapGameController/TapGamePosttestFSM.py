"""
This is the main FSM / Game Logic class for the Tap Game
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
import time
import pronouncing
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
from .StudentPhonemeModel import StudentPhonemeModel
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


class TapGamePosttestFSM: # pylint: disable=no-member, too-many-instance-attributes
    """
    An FSM for the Tap Game. Contains Game Logic and some nodes for interacting w the Unity "View"
    """

    round_index = 0   

    player_score = 0
    robot_score = 0

    student_word_model = StudentWordModel()
    student_phoneme_model = StudentPhonemeModel()
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

        if not  self.experiment_phase == 'posttest':
            print(str(self.experiment_phase) + " was not 'posttest'")
            exit()

        # Initializes a new audio recorder object if one hasn't been created
        self.recorder = AudioRecorder(self.participant_id, self.experimenter_name, self.experiment_phase)

        # Tell robot to look at Tablet
        self.ros_node_mgr.init_ros_node()
        #self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_AT_TABLET)

    def on_init_first_round(self):
        """
        Called when the game registers with the controller
        Should send msg to Unity game telling it the word to load for the first round
        """
        print("got to init_first round!")
        # get the next robot action
        self.current_round_action = ActionSpace.DONT_RING


        #send message every 2s in case it gets dropped
        def send_msg_til_received():
            while(self.state == "ROUND_START"):
                self.current_round_word = self.student_word_model.curriculum[self.round_index]
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
        means, variances = self.student_word_model.train_and_compute_posterior([self.current_round_word],
                                                                          [avg_phone_score / 100.0])        
        

        phonemes_raw = pronouncing.phones_for_word(self.current_round_word.lower())[0].split(' ')
        arpabet_phonemes = [''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
        print("ARPABET PHONEME FOR ROUND")
        print(arpabet_phonemes)
        print(self.scores)

        self.student_phoneme_model.words_so_far.append(self.current_round_word) #tell the phoneme model which word for active learning
        phoneme_means, phoneme_vars = self.student_phoneme_model.train_and_compute_posterior(arpabet_phonemes, [x / 100.0 for x in self.scores])

        print("LATEST MEANS / VARS")
        print(self.student_word_model.curriculum)
        print(means)
        print(variances)

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
        time.sleep(.5) #wait for half a second so the lookat can go through

        #ROBOT REACTION LOGIC  - consider using fidget text?      
        
        avg_phoneme_score = (sum(self.scores) / len(self.scores))
        print('AVG PHONEME SCORE WAS')
        print(avg_phoneme_score)
        if avg_phoneme_score >= PASSING_SCORE_THRESHOLD: #if child got the word right
            self.player_score += 1
            self.player_passed_round = True            
        else:
            self.player_passed_round = False            
         
        self.handle_round_end()       
        

    def on_reset_round(self):
        self.ros_node_mgr.send_game_cmd(TapGameCommand.RESET_NEXT_ROUND)

    def on_game_finished(self):
        """
        Called when we have completed 'max_rounds' rounds in a game.
        Sends msg to the Unity game to load the game end screen
        """
        print('got to game finished')        
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

            if data.message == TapGameLog.GAME_START_PRESSED:                
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
                self.ros_node_mgr.publish_round_summary(self.round_index, self.current_round_action, self.current_round_word,
                                                                        self.player_won_round_tap, self.player_passed_round, self.audio_file, self.letters, self.scores,
                                                                        self.passed, self.player_score, self.robot_score, self.student_word_model.curriculum, 
                                                                        self.student_word_model.means, self.student_word_model.variances, self.student_phoneme_model.curriculum, 
                                                                        self.student_phoneme_model.means, self.student_phoneme_model.variances)

                self.round_index += 1
                self.player_beat_robot = False
                self.current_round_word = self.student_word_model.curriculum[self.round_index]

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
        return (self.round_index == len(self.student_word_model.curriculum) - 1)

    def is_not_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return not self.is_last_round()
            

        

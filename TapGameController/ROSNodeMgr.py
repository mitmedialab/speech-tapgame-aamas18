"""
This is the main class that manages the creation / parsing of ROS Node Communication
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name

import time
from GameUtils import GlobalSettings
from .JiboBehaviors import JiboBehaviors
from .TegaBehaviors import TegaBehaviors


if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from unity_game_msgs.msg import TapGameRoundSummary
    from r1d1_msgs.msg import TegaAction
    from r1d1_msgs.msg import Vec3
    from jibo_msgs.msg import JiboAction
else:
    TapGameLog = GlobalSettings.TapGameLog  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    TegaAction = GlobalSettings.TegaAction
    JiboAction = GlobalSettings.JiboAction

ROSCORE_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROSCORE_TOPIC = '/tap_game_to_ros'
ROUND_SUMMARY_TOPIC = '/tap_game_round_summary'

ROSCORE_TO_JIBO_TOPIC = '/jibo'
ROSCORE_TO_TEGA_TOPIC = '/tega'

class ROSNodeMgr:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Manager for the ROS Communication in the Tap Game. Contains nodes for interacting
    w the Unity "View" and also the Robot
    """

    game_commander = None
    robot_commander = None
    round_summary_logger = None
    log_listener = None



    def __init__(self):
        pass


    def init_ros_node(self): #pylint: disable=no-self-use
        """
        Start up the game connection ROS node
        """
        rospy.init_node('FSM_Listener_Controller', anonymous=True)

    def start_log_listener(self, on_log_callback):
        """
        Start up the Game Log Subscriber node
        """
        print('Sub Node started')
        self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROSCORE_TOPIC, TapGameLog,
                                             on_log_callback)

    def start_cmd_publisher(self):
        """
        Starts up the command publisher node
        """
        print('GameCmd Pub Node started')
        self.game_commander = rospy.Publisher(ROSCORE_TO_TAP_GAME_TOPIC,
                                              TapGameCommand, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers
        # rospy.spin()

    def start_robot_publisher(self):
        """
        Starts up the robot publisher node
        """
        print('Robot Pub Node started')

        if GlobalSettings.USE_TEGA:
            msg_type = TegaAction
            msg_topic = ROSCORE_TO_TEGA_TOPIC
        else:
            msg_type = JiboAction
            msg_topic = ROSCORE_TO_JIBO_TOPIC

        self.robot_commander = rospy.Publisher(msg_topic, msg_type, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers

    def start_round_publisher(self):
        """
        Starts up the gume summary publisher node
        """
        print('Game Summary Pub Node started')

        self.round_summary_logger = rospy.Publisher(ROUND_SUMMARY_TOPIC, TapGameRoundSummary, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers  

    def publish_round_summary(self, round_index, current_round_action, current_round_word,
                                    player_won_round_tap, player_passed_round, audio_file, letters, scores, passed,
                                    player_score, robot_score, word_curriculum, word_means, word_variances, phoneme_curriculum, 
                                    phoneme_means, phoneme_variances):
        # send message to tablet game
        if self.round_summary_logger is None:
            self.start_round_publisher()
            time.sleep(.5)

        msg = TapGameRoundSummary()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # fill in command and any params:
        msg.round_index = round_index
        msg.current_round_action = current_round_action
        msg.current_round_word = current_round_word

        msg.player_won_round_tap = player_won_round_tap
        msg.player_passed_round = player_passed_round
        msg.audio_file = audio_file
        #msg.speechace_result = speechace_result

        msg.letters = letters
        msg.scores = scores
        msg.passed = passed

        msg.player_score = player_score
        msg.robot_score = robot_score

        msg.word_curriculum = word_curriculum
        msg.word_means = word_means
        msg.word_variances = word_variances

        msg.phoneme_curriculum = phoneme_curriculum
        msg.phoneme_means = phoneme_means
        msg.phoneme_variances = phoneme_variances

        print(round_index)
        print(current_round_action)
        print(current_round_word)
        print(player_won_round_tap)
        print(audio_file)
        print(letters)
        print(scores)
        print(passed)

        self.round_summary_logger.publish(msg)
        rospy.loginfo(msg)



    def send_game_cmd(self, command, *args):
        """
        send a TapGameCommand to game
        Args are optional parameters
        """

        # send message to tablet game
        if self.game_commander is None:
            self.start_cmd_publisher()
            time.sleep(.5)

        msg = TapGameCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # fill in command and any params:
        msg.command = command
        if len(args) > 0:
            msg.params = args[0]
        self.game_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_cmd(self, command, *args):
        """
        send a command to the robot (action space or "cosmetic")
        This function maps actions from the ActionSpace into actual ROS Msgs
        """

        if self.robot_commander is None:
            self.start_robot_publisher()
            time.sleep(.5)

        # choose which platform
        if GlobalSettings.USE_TEGA:
            msg = TegaBehaviors.get_msg_from_behavior(command, args)
        else:
            msg = JiboBehaviors.get_msg_from_behavior(command, args)

        # add header
        self.robot_commander.publish(msg)  # would be nice to guarantee message performance here
        rospy.loginfo(msg)

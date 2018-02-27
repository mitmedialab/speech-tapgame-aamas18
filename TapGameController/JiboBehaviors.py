"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name

from .RobotBehaviorList import RobotBehaviors
from random import randint
from GameUtils import GlobalSettings

if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import TegaAction
    from r1d1_msgs.msg import Vec3
    from jibo_msgs.msg import JiboAction
else:
    TapGameLog = GlobalSettings.TapGameLog  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    JiboAction = GlobalSettings.JiboAction


class JiboBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """
    @staticmethod
    def get_msg_from_behavior(command, *args): #pylint: disable=too-many-branches, too-many-statements

        msg = JiboAction()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        if command == RobotBehaviors.LOOK_AT_TABLET:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.LOOK_DOWN

        elif command == RobotBehaviors.LOOK_CENTER:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.DEFAULT
        
        elif command == RobotBehaviors.SAY_HI:
            msg.do_sound_playback = True
            msg.audio_filename = "SSA_hello.wav"

        elif command == RobotBehaviors.RING_ANSWER_CORRECT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.RING_IN_ANIM

        elif command == RobotBehaviors.LATE_RING:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.RING_IN_ANIM

        elif command == RobotBehaviors.PRONOUNCE_CORRECT:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_lookat = False
            msg.tts_text = args[0][0]
            print(args[0][0])

        elif command == RobotBehaviors.PRONOUNCE_WRONG_SOUND:
            msg.do_motion = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = True

            num = randint(0, 2)
            if num == 0:
                msg.audio_filename = "SSA_disappointed.m4a"
            elif num == 1:
                msg.audio_filename = "SSA_wrong.m4a"
            elif num == 2:
                msg.audio_filename = "SSA_wompwomp.m4a"
            

        elif command == RobotBehaviors.PRONOUNCE_WRONG_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_lookat = False
            msg.do_sound_playback = False

            num = randint(0, 3)
            if num == 0:
                msg.tts_text = "gee I don't know"
            elif num == 1:
                msg.tts_text = "wow I'm not sure about this one"
            elif num == 2:
                msg.tts_text = "I don't know this one"
            elif num == 3:
                msg.tts_text = "I don't know this one"
            
        elif command == RobotBehaviors.REACT_ROBOT_CORRECT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False

            num = randint(0, 1)
            if num == 0:
                msg.motion = "Misc/Eye_to_Happy_01.keys"
            elif num == 1:
                msg.motion = "Misc/Eye_to_Happy_02.keys"
            

        elif command == RobotBehaviors.REACT_ROBOT_WRONG:
            msg.do_sound_playback = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_motion = True

            num = randint(0, 2)
            if num == 0:
                msg.motion = "Misc/Eye_Sad_03_01.keys"
            elif num == 1:                
                msg.motion = "Misc/Eye_Sad_03_02.keys"
            elif num == 2:
                msg.do_motion = True
                msg.motion = "Misc/Frustrated_01_04.keys"
                

        elif command == RobotBehaviors.REACT_PLAYER_CORRECT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False

            num = randint(0, 1)
            if num == 0:
                msg.motion = "Misc/Eye_to_Happy_01.keys"
            elif num == 1:
                msg.motion = "Misc/Eye_to_Happy_02.keys"
            

        elif command == RobotBehaviors.REACT_PLAYER_WRONG:
            msg.do_sound_playback = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_motion = True

            num = randint(0, 2)
            if num == 0:
                msg.motion = "Misc/Eye_Sad_03_01.keys"
            elif num == 1:                
                msg.motion = "Misc/Eye_Sad_03_02.keys"
            elif num == 2:
                msg.do_motion = True
                msg.motion = "Misc/Frustrated_01_04.keys"
                
            
        elif command == RobotBehaviors.REACT_TO_BEAT_CORRECT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = "Misc/Frustrated_01_04.keys"


        elif command == RobotBehaviors.REACT_TO_BEAT_WRONG:
            msg.do_motion = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = True
            msg.audio_filename = "SSA_laugh.m4a"

        elif command == RobotBehaviors.WIN_MOTION:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.motion = JiboAction.HAPPY_GO_LUCKY_DANCE

        elif command == RobotBehaviors.WIN_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_lookat = False
            msg.tts_text = "I win I win I win I win I win"

        elif command == RobotBehaviors.LOSE_MOTION:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.motion = "Misc/Sad_03.keys"

        elif command == RobotBehaviors.LOSE_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_lookat = False
            msg.tts_text = "You beat me. I'll try to do better next time"

        elif command == RobotBehaviors.EYE_FIDGET:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.motion = JiboAction.EYE_FIDGET

        elif command == RobotBehaviors.REACT_TO_BEAT:
            msg.do_motion = True
            msg.do_lookat = False
            msg.motion = "Misc/Frustrated_01_04.keys"

        elif command == RobotBehaviors.PLAYER_RING_PROMPT:
            msg.do_motion = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = True
            msg.audio_filename = "SSA_prompt.m4a"

        return msg

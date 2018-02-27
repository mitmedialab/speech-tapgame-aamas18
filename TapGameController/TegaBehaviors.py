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
    TegaAction = GlobalSettings.TegaAction


class TegaBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """
    @staticmethod
    def get_msg_from_behavior(command, *args): #pylint: disable=too-many-branches, too-many-statements

        msg = TegaAction()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        if command == RobotBehaviors.LOOK_AT_TABLET:
            lookat_pos = Vec3()
            lookat_pos.x = 0
            lookat_pos.y = 2
            lookat_pos.z = 20
            msg.do_look_at = True
            msg.look_at = lookat_pos

        elif command == RobotBehaviors.LOOK_CENTER:
            lookat_pos = Vec3()
            lookat_pos.x = 0
            lookat_pos.y = 10
            lookat_pos.z = 40
            msg.do_look_at = True
            msg.look_at = lookat_pos

        elif command == RobotBehaviors.SAY_HI:
            msg.wav_filename = "vocab_games/effects/say_hi.wav"

        ## Basic Ring Ins

        elif command == RobotBehaviors.RING_ANSWER_CORRECT:
            msg.motion = "PERKUP"
            
        elif command == RobotBehaviors.LATE_RING:
            msg.motion = "PERKUP"            

        elif command == RobotBehaviors.PRONOUNCE_CORRECT:
            msg.enqueue = True
            msg.wav_filename = "vocab_games/words/" + args[0][0].lower() + ".wav"

        elif command == RobotBehaviors.PRONOUNCE_WRONG_SOUND:
            msg.enqueue = True
            num = randint(0, 3)
            if num == 0:
                msg.wav_filename = "vocab_games/effects/thinking1.wav"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/puzzled1.wav"
            elif num == 2:
                msg.wav_filename = "vocab_games/effects/confused1.wav"                        
            elif num == 3:
                msg.motion = "THINKING"                        


        #### Reactions                 

        elif command == RobotBehaviors.REACT_TO_BEAT_CORRECT:
            num = randint(0, 2)
            if num == 0:
                msg.motion = "FRUSTRATED"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/angry2.wav"
            elif num == 2:
                msg.wav_filename = "vocab_games/effects/angry4.wav"


        elif command == RobotBehaviors.REACT_TO_BEAT_WRONG:
            num = randint(0, 1)
            if num == 0:
                msg.wav_filename = "vocab_games/effects/laugh1.wav"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/laugh2.wav"

        elif command == RobotBehaviors.REACT_ROBOT_CORRECT:
            num = randint(0, 1)
            if num == 0:
                msg.motion = "SMILE"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/woohoo1.wav"                
            

        elif command == RobotBehaviors.REACT_ROBOT_WRONG:
            num = randint(0, 1)
            if num == 0:
                msg.wav_filename = "vocab_games/effects/aww1.wav"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/sigh2.wav"

        elif command == RobotBehaviors.REACT_PLAYER_CORRECT:
            num = randint(0, 3)
            if num == 0:
                msg.motion = "SMILE"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/yes1.wav"
            elif num == 2:
                msg.wav_filename = "vocab_games/effects/good_job.wav"                    
            elif num == 3:
                msg.motion = "YES"                                    
            

        elif command == RobotBehaviors.REACT_PLAYER_WRONG:
            num = randint(0, 2)
            if num == 0:
                msg.wav_filename = "vocab_games/effects/aww1.wav"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/serene.wav"                
            elif num == 2:
                msg.motion = "NO"

        elif command == RobotBehaviors.WIN_MOTION:
            msg.motion = TegaAction.MOTION_EXCITED

        elif command == RobotBehaviors.WIN_SPEECH:
            pass
            # msg.wav_filename = "vocab_games/effects/woohoo1.wav"

        elif command == RobotBehaviors.LOSE_MOTION:
            msg.motion = TegaAction.MOTION_SAD

        elif command == RobotBehaviors.LOSE_SPEECH:
            pass
            # msg.wav_filename = "vocab_games/effects/sigh1.wav"
        elif command == RobotBehaviors.PLAYER_RING_PROMPT:
            num = randint(0, 1)
            if num == 0:
                msg.wav_filename = "vocab_games/effects/interested.wav"
            elif num == 1:
                msg.wav_filename = "vocab_games/effects/deep_think.wav"            

        return msg
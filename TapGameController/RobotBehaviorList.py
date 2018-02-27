"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name


class RobotBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """

    # Look Ats
    LOOK_AT_TABLET = 'LOOK_AT_TABLET'
    LOOK_CENTER= 'LOOK_CENTER'

    # Action Space Actions
    RING_ANSWER_CORRECT = 'RING_ANSWER_CORRECT'
    LATE_RING = 'LATE_RING'

    #Pronunciation Actions the robot can do after ringing in
    PRONOUNCE_CORRECT = 'PRONOUNCE_CORRECT'
    PRONOUNCE_WRONG_SPEECH = 'PRONOUNCE_WRONG_SPEECH'
    PRONOUNCE_WRONG_SOUND = 'PRONOUNCE_WRONG_SOUND'

    #Reaction Actions the robot can do after its results are revealed
    REACT_ROBOT_CORRECT = 'REACT_ANSWER_CORRECT'
    REACT_ROBOT_WRONG = 'REACT_ANSWER_WRONG'

    # Reactions to the robot getting beaten on the buzz
    REACT_TO_BEAT = 'REACT_TO_BEAT' # played when the robot buzzes in, but player already got there
    REACT_TO_BEAT_CORRECT = 'REACT_TO_BEAT_CORRECT' # played when players results are revealed
    REACT_TO_BEAT_WRONG = 'REACT_TO_BEAT_WRONG' # played when players results are revealed

    #Reaction Actions the robot can do after the player results are revealed
    REACT_PLAYER_CORRECT = 'REACT_PLAYER_CORRECT'
    REACT_PLAYER_WRONG = 'REACT_PLAYER_WRONG'

    # Other Misc. Cosmetic behaviors
    EYE_FIDGET = 'EYE_FIDGET'
    PLAYER_RING_PROMPT = 'PLAYER_RING_PROMPT'
    SAY_HI = 'SAY_HI'

    # After each game is finished, robot response to winning or losing
    WIN_MOTION = 'WIN_MOTION'
    WIN_SPEECH = 'WIN_SPEECH'

    LOSE_MOTION = 'LOSE_MOTION'
    LOSE_SPEECH = 'LOSE_SPEECH'

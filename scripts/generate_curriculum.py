##USAGE: should only be executed by Makefile, e.g. "make curriculum"

from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
from GameUtils.Curriculum import Curriculum
import numpy as np
import os

def build_curriculum():

    myUtils = PronunciationUtils()

    # fancy python one-liner to read all string attributes off of a class
    curriculum = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]

    np.save(os.getcwd() + myUtils.CURRICULUM_PATH, substitute_costs)
    np.savetxt(os.getcwd() + myUtils.CURRICULUM_PATH + '.txt', substitute_costs)

build_curriculum()
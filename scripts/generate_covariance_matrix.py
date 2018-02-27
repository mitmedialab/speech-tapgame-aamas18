##USAGE: should only be executed by Makefile, e.g. "make cov_matrix"

from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
from GameUtils.Curriculum import Curriculum
import os
import numpy as np

myUtils = PronunciationUtils()
myCurriculum = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]

loaded_weighted_lev_distances = np.load(os.getcwd() + myUtils.WEIGHTED_LEV_DISTANCE_PATH + '.npy')
loaded_gloVe_distances = np.load(os.getcwd() + myUtils.GLOVE_DISTANCE_PATH + '.npy')

### THESE MUST ADD UP TO 1!!! ###
PHONETIC_WEIGHT = .33
SEMANTIC_WEIGHT = .66

def create_cov_matrix():
    covariance_matrix = np.ones((len(myCurriculum), len(myCurriculum)),
                               dtype=np.float64)  # make a 2D array of 1's.

    for i in range(0, len(myCurriculum)):
        for j in range(0, len(myCurriculum)):

            #yeah I know we're calculating it twice - TODO: Optimize
            covariance_matrix[i][j] = (PHONETIC_WEIGHT * (1 - loaded_weighted_lev_distances[i][j])) + \
                                        (SEMANTIC_WEIGHT * loaded_gloVe_distances[i][j])

            print("cov between " + myCurriculum[i] + " and " + myCurriculum[j] + " was " + str(covariance_matrix[i][j]))


    np.save(os.getcwd() + PronunciationUtils.WORD_COVARIANCE_PATH, covariance_matrix)
    np.savetxt(os.getcwd() + PronunciationUtils.WORD_COVARIANCE_PATH + '.txt', covariance_matrix)

create_cov_matrix()
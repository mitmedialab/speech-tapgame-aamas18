##USAGE: should only be executed by Makefile, e.g. "make lev_matrix"

from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
import spacy
from GameUtils.Curriculum import Curriculum
import os
import numpy as np


#nlp = spacy.load('en')  # sets spacy up with the basic english language model (5000 words)
nlp = spacy.load('en_core_web_md') #this contains the FULL english word vector model (about 1mm words)
myUtils = PronunciationUtils()
myCurriculum = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]

def create_gloVe_distance_matrix():
    gloVe_distances = np.ones((len(myCurriculum), len(myCurriculum)),
                               dtype=np.float64)  # make a 2D array of 1's.

    for i in range(0, len(myCurriculum)):
        for j in range(0, len(myCurriculum)):

            #yeah I know we're calculating it twice - TODO: Optimize
            gloVe_distances[i][j] = measure_gloVe_cosine_distance(
                myCurriculum[i], myCurriculum[j])


    np.save(os.getcwd() + PronunciationUtils.GLOVE_DISTANCE_PATH, gloVe_distances)
    np.savetxt(os.getcwd() + PronunciationUtils.GLOVE_DISTANCE_PATH + '.txt', gloVe_distances)


def measure_gloVe_cosine_distance(word1, word2):

    if word1 == word2:
        normalized_score = 1.0
    else:
        word1Token = nlp(word1.lower())
        word2Token = nlp(word2.lower())

        normalized_score = min(word1Token.similarity(word2Token), 1.0)
    print("gloVe similarity (cosine distance) between " + word1 + " and " + word2 + " was " + str(normalized_score))
    return normalized_score

create_gloVe_distance_matrix()
##USAGE: should only be executed by Makefile, e.g. "make lev_matrix"

from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
from GameUtils.Curriculum import Curriculum

import pandas as pd
import os
import numpy as np
from weighted_levenshtein.clev import levenshtein as lev


WPSM_PATH = '/GameUtils/PronunciationUtils/data/wpsm.csv'
myUtils = PronunciationUtils()
myCurriculum = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]


def create_substitution_matrix():

    # read weighted phonemic similarity matrix,
    # downloaded from https://github.com/benhixon/benhixon.github.com/blob/master/wpsm.txt

    # the subsitution matrix 

    # because levenshtein is computed on a character-by-character basis, we use the nettalk phonemic
    # representation instead of Arpabet for this task

    # the

    # load the matrix csv file into a dataframe
    df = pd.read_csv(os.getcwd() + WPSM_PATH, sep=',', header=0, index_col=0)

    arpabet_phonemes = list(df.keys())
    #print(df['B'])

    substitute_costs = np.ones((128, 128), #128 is size of ASCII table we care about
                               dtype=np.float64)  # make a 2D array of 1's. ASCII table
    
    # update the original substitution matrix
    lowest = 100
    lowkey1 = None
    lowkey2 = None

    highest = -100
    lowkey1 = None
    lowkey2 = None

    # calculate min/max values of non-diagonal wpsm scores
    for key1 in arpabet_phonemes:
        for key2 in arpabet_phonemes:
            if not key1 == key2:
                if (-1 * df[key1][key2]) < lowest:
                    lowkey1 = key1
                    lowkey2 = key2
                    lowest = -1 * df[key1][key2]  # "P/B"

                if (-1 * df[key1][key2]) > highest:
                    highkey1 = key1
                    highkey2 = key2
                    highest = -1 * df[key1][key2] #TH DH

    print("lowest score was ")
    print(lowest)
    print(df[lowkey1][lowkey2])

    print("highest score was ")
    print(highest)
    print(df[highkey1][highkey2])

    range = highest - lowest
    print('Should be 1.0 followed by 0.0')
    print(normalize_wpsm_cost(highest, highest, lowest))
    print(normalize_wpsm_cost(lowest, highest, lowest))
    print('----------')

    for key1 in arpabet_phonemes:
        for key2 in arpabet_phonemes:
            nkey1 = myUtils.arpabet_map[key1]
            nkey2 = myUtils.arpabet_map[key2]

            if (nkey1 == nkey2):
                substitute_costs[ord(nkey1), ord(
                    nkey2)] = 0.0  # use zero substitution cost for diagonal entries

                #print("COST was 0 for")
                #print(nkey1)
                #print(nkey2)
            else:
                raw_cost = -1 * df[key1][key2] # negative scores in wpsm indicate less similarity, hence higher cost
                normalized_cost = normalize_wpsm_cost(raw_cost, highest, lowest)
                substitute_costs[ord(nkey1), ord(nkey2)] = normalized_cost

    np.save(os.getcwd() + PronunciationUtils.PHONEME_SUB_COST_PATH, substitute_costs)
    np.savetxt(os.getcwd() + PronunciationUtils.PHONEME_SUB_COST_PATH + '.txt', substitute_costs)

def create_arpabet_covariance_matrix():
    # update the original substitution matrix

    df = pd.read_csv(os.getcwd() + WPSM_PATH, sep=',', header=0, index_col=0)

    arpabet_phonemes = list(df.keys())
    arpabet_size = len(arpabet_phonemes) #SHOULD BE 39 x 39
    print(list(arpabet_phonemes))


    arpabet_covariance = np.ones((arpabet_size, arpabet_size), #128 is size of ASCII table we care about
                               dtype=np.float64)  # make a 2D array of 1's. ASCII table
    lowest = 100
    lowkey1 = None
    lowkey2 = None

    highest = -100
    lowkey1 = None
    lowkey2 = None

    # calculate min/max values of non-diagonal wpsm scores
    for key1 in arpabet_phonemes:
        for key2 in arpabet_phonemes:
            if not key1 == key2:
                if (df[key1][key2]) < lowest:
                    lowkey1 = key1
                    lowkey2 = key2
                    lowest = df[key1][key2]  # "P/B"

                if (df[key1][key2]) > highest:
                    highkey1 = key1
                    highkey2 = key2
                    highest = df[key1][key2] #TH DH

    # print("lowest score was ")
    # print(lowest)
    # print(df[lowkey1][lowkey2])

    # print("highest score was ")
    # print(highest)
    # print(df[highkey1][highkey2])

    range = highest - lowest
    print('Should be 0.8 followed by 0.01')
    print(normalize_wpsm_cost_for_covariance(highest, highest, lowest))
    print(normalize_wpsm_cost_for_covariance(lowest, highest, lowest))
    print('----------')

    for key1 in arpabet_phonemes:
        for key2 in arpabet_phonemes:
            nkey1 = myUtils.arpabet_map[key1]
            nkey2 = myUtils.arpabet_map[key2]

            if (nkey1 == nkey2):                

                # give covariance of 1 for identical keys
                arpabet_covariance[arpabet_phonemes.index(key1)][arpabet_phonemes.index(key2)] = 1.0
                #print("COV was 1 for")
                #print(nkey1)
                #print(nkey2)
            else:
                raw_cost = df[key1][key2]                                
                normalized_cov = normalize_wpsm_cost_for_covariance(raw_cost, highest, lowest)
                arpabet_covariance[arpabet_phonemes.index(key1)][arpabet_phonemes.index(key2)] = normalized_cov #covariance is 0-1

                #print(key1 + " AND " + key2 + " have covariance: " + str(normalized_cov))

    # a = substitute_costs[ord(nkey1)][:]
    # print(ord(nkey1))
    # print(a)
    # b = [x for x in a if not x == 1.0]
    # print(b)
    # print(len(b))
    # c = [x for x in a if x == 0.0]
    # print(c)
    # print(len(c))

    np.save(os.getcwd() + PronunciationUtils.ARPABET_COVARIANCE_PATH, arpabet_covariance)
    np.savetxt(os.getcwd() + PronunciationUtils.ARPABET_COVARIANCE_PATH + '.txt', arpabet_covariance)
def create_levenshtein_cost_matrix():
    weighted_levenshtein_distances = np.ones((len(myCurriculum), len(myCurriculum)),
                               dtype=np.float64)  # make a 2D array of 1's. ASCII table

    for i in range(0, len(myCurriculum)):
        for j in range(0, len(myCurriculum)):
            #yeah I know we're calculating it twice - TODO: Optimize
            weighted_levenshtein_distances[i][j] = measure_weighted_levenshtein_distance(
                myCurriculum[i], myCurriculum[j])


    np.save(os.getcwd() + PronunciationUtils.WEIGHTED_LEV_DISTANCE_PATH, weighted_levenshtein_distances)
    np.savetxt(os.getcwd() + PronunciationUtils.WEIGHTED_LEV_DISTANCE_PATH + '.txt', weighted_levenshtein_distances)


def measure_weighted_levenshtein_distance(word1, word2):
    # import weighted levenshtein library. if clev is missing, change the __init__.py in the weighted_levenshtein lib to add clev.so path to sys.
    # /anaconda3/lib/python3.6/site-packages/weighted_levenshtein
    # delete "from clev import *" in __init__.py

    # read the phoneme substitution costs from disk
    loaded_substitute_costs = np.load(os.getcwd() + myUtils.PHONEME_SUB_COST_PATH + '.npy')
    result = lev(myUtils.get_phonetic_similarity_rep(word1).encode(),
                 myUtils.get_phonetic_similarity_rep(word2).encode(),
                 substitute_costs=loaded_substitute_costs)

    # normalize the levenshtein score by taking max(str1,str2)
    denominator = max(len(word1), len(word2))
    normalized_score = (result / float(denominator))
    #print("weighted-lev distance between " + word1 + " and " + word2 + " was " + str(normalized_score))
    return normalized_score

def normalize_wpsm_cost(raw_cost, highest, lowest):
    """
    converts a raw_cost into the range (0,1), given the highest and lowest wpsm costs
    See: https://stackoverflow.com/questions/5294955/how-to-scale-down-a-range-of-numbers-with-a-known-min-and-max-value
    """
    # print(raw_cost)
    # print("TURNED INTO")
    scaled_cost = ((1 - 0) * (raw_cost - lowest) / (highest - lowest))
    # print(str(scaled_cost) + '\n')
    return scaled_cost

def normalize_wpsm_cost_for_covariance(raw_cost, highest, lowest):
    """
    converts a raw_cost into the range (0,.8), given the highest and lowest wpsm costs
    See: https://stackoverflow.com/questions/5294955/how-to-scale-down-a-range-of-numbers-with-a-known-min-and-max-value
    """
    # print(raw_cost)
    # print("TURNED INTO")
    scaled_cost = ((.8 - 0.01) * (raw_cost - lowest) / (highest - lowest))
    # print(str(scaled_cost) + '\n')
    return scaled_cost + .01    



create_substitution_matrix()
create_arpabet_covariance_matrix()
create_levenshtein_cost_matrix()
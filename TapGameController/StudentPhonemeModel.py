"""
This Module handles all aspects of student modeling and assessment
Some code remixed from
http://katbailey.github.io/post/gaussian-processes-for-dummies/
^ Great intro article for rolling your own GP
"""
# pylint: disable=import-error
import random

import math
import operator
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import pandas as pd

import pronouncing
import csv
import os


from GameUtils.Curriculum import Curriculum
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils

from .AgentModel import ActionSpace

WPSM_PATH = '/GameUtils/PronunciationUtils/data/wpsm.csv'



class StudentPhonemeModel(): # pylint: disable=invalid-name,consider-using-enumerate,too-many-instance-attributes

    """
    This class implements a Gaussian Process, intended to model student vocabulary knowledge
    It uses a kernel based on a phonetic distance heuristic.
    pylint's 'invalid-name' checker has been disabled to conform with the notation
    in Rasmussen and Williams
    """

    def __init__(self):

        self.DO_ACTIVE_LEARNING = True

        # load all phonemes into a list
        
        df = pd.read_csv(os.getcwd() + WPSM_PATH, sep=',', header=0, index_col=0)
        arpabet_phonemes = df.keys()
        self.curriculum = list(arpabet_phonemes) # for phoneme model, this is the list of arpabet phonemes
        self.word_list = [p for p in dir(Curriculum)
                           if isinstance(getattr(Curriculum, p), str)
                           and not p.startswith('__')] #this is the list of words, used for active learning selection

        self.pronunciation_utils = PronunciationUtils()

        self.loaded_covariance_matrix = np.load(
            os.getcwd() + self.pronunciation_utils.ARPABET_COVARIANCE_PATH + '.npy')

        # these parameters govern the assumed Gaussian noise added to the child's recorded
        # pronunciation assessment
        self.noise_mu = 0
        self.noise_sigma = .3

        self.X_train = [] # These are persistent lists of training data!
        self.Y_train = []
        self.words_so_far = [] # In order to do the active learning properly, we need to maintain a list of what words
                               # have been asked already

        self.means = [.5] * len(self.curriculum)  # These are the most recent posteriors
        self.variances = [.3] * len(self.curriculum) # Together they form the Student Model!
        #self.fig = None # Figure for drawing
        #self.plts = None #Plots

        self.n_rows = 5 # needs to be > 1        


    def init_model(self):
        """
        sets up the GP and kernel
        """


    def get_prior(self, n_samples):
        """
        Samples form a prior distribution over the words
        See Algorithm 2.1 in Rasmussen and Williams to follow along with implementation + notation
        """

        # Test data
        # we only care about evaluating the GP at this finite collection of points
        x_test = self.curriculum

        K_ss = self.phoneme_distance_kernel(x_test, x_test) #the Kernel Matrix for the words

        # Get cholesky decomposition (square root) of the
        # covariance matrix
        L = np.linalg.cholesky(K_ss + 1e-15 * np.eye(len(x_test)))

        # Sample 3 sets of standard normals for our test points,
        # multiply them by the square root of the covariance matrix
        f_prior = np.dot(L, np.random.normal(size=(len(x_test), n_samples)))
        return f_prior

    def train_and_compute_posterior(self, new_X_train, new_Y_train):
        """
        Takes in a list of labeled X and labeled Y data points and computes a new posterior
        E.g.
        new_X_train = ['K', 'AE', 'T']  # these numbers are the phoneme labels
        new_Y_train = [1, 1, 1] # these numbers correspond to phoneme scores

        See Algorithm 2.1 in Rasmussen and Williams to follow along with implementation + notation
        """

        Xtest = self.curriculum  # these numbers are just labels

        for i in range(len(new_X_train)):
            self.X_train.append(new_X_train[i])
            self.Y_train.append(new_Y_train[i])

        # compute cov of train set wrt itself + cov of all words wrt all words
        print("COMPUTING NEW GP POSTERIOR")
        print("XTRAIN, YTRAIN, MU, VAR")
        print(self.X_train)
        print(self.Y_train)
        print(self.means)
        print(self.variances)
        K = self.phoneme_distance_kernel(self.X_train, self.X_train)
        K_ss = self.phoneme_distance_kernel(Xtest, Xtest)

        cholesky_mtx = (K + 0.00005 * np.eye(len(self.X_train)) +
                               ((self.noise_sigma ** 2) * np.eye(len(self.X_train))))
        print(cholesky_mtx)
        # cholesky_mtx[cholesky_mtx > 1.0] = 1.0
        # print(cholesky_mtx)
        L = np.linalg.cholesky(cholesky_mtx)

        L_y = np.linalg.solve(L, (list(map(operator.sub, self.Y_train, self.get_mean(self.X_train))))) #pythonic way to subtract two lists #pylint: disable=line-too-long
        #L_y = np.linalg.solve(L, self.Y_train) #zero mean function version
        a = np.linalg.solve(L.T, L_y)

        # Compute the mean at our test points.
        K_s = self.phoneme_distance_kernel(self.X_train, Xtest)
        mu = self.get_mean(Xtest) + np.dot(K_s.T, a).reshape(len(self.curriculum), )
        #mu = np.dot(K_s.T, a).reshape(len(self.curriculum), ) #zero mean function version
        v = np.linalg.solve(L, K_s)

        # we only want the diagonal bc we want to know
        # variance of each variable independent of any others
        variance = np.diag(K_ss - np.dot(v.T, v))
        #stdv = np.sqrt(variance)

        self.means = mu
        self.variances = variance

        print("GP POSTERIOR IS NOW")
        print("XTRAIN, YTRAIN, MU, VAR")

        # dont let variances underflow below 0
        # self.variances.flags.writeable = True        
        # self.variances[self.variances < 0.00005] = 0.00005
        # self.means.flags.writeable = True        
        # self.means[self.variances < 0.00005] = 0.00005

        print(self.X_train)
        print(self.Y_train)
        print(self.means)
        print(self.variances)
        return self.means, self.variances


    def get_next_best_word(self, action):
        """
        gives an external caller the next best word to achieve some objective
        Active Learning paradigm is implemented here!
        """


        if self.DO_ACTIVE_LEARNING:

            print("WORDS SO FAR IS")
            print(self.words_so_far)

            if action == ActionSpace.RING_ANSWER_CORRECT:

                # choose lowest mean word of words that have not yet been asked about
                lowest_mean = 10.0 # any mean will be lower than initial value
                lowest_mean_index = 0


                for i in range(0, len(self.word_list)):
                    candidate_mean = self.get_phoneme_mean(self.word_list[i])
                    if candidate_mean < lowest_mean and self.word_list[i] not in self.words_so_far:
                        lowest_mean = candidate_mean
                        lowest_mean_index = i
                chosen_word = self.word_list[lowest_mean_index]
                print("CHOSEN WORD WAS")
                print(chosen_word)
                print("W MEAN OF " + str(lowest_mean))


            elif action == ActionSpace.DONT_RING:

                # choose highest var word
                highest_var = -10
                highest_var_index = 0

                for i in range(0, len(self.word_list)):
                    candidate_var = self.get_phoneme_var(self.word_list[i])
                    if candidate_var > highest_var and self.word_list[i] not in self.words_so_far:
                        highest_var = candidate_var
                        highest_var_index = i
                chosen_word = self.word_list[highest_var_index]
                print("CHOSEN WORD WAS")
                print(chosen_word)
                print("W VAR OF " + str(highest_var))

        else:
            # randint is inclusive
            chosen_word = self.curriculum[random.randint(0, len(self.curriculum) - 1)]


        return chosen_word

    def rbf_kernel(self, input_a, input_b, length_scale):
        """
        Implements to Radial Basis Function kernel
        """

        sqdist = np.sum(input_a ** 2, 1).reshape(-1, 1) + \
                 np.sum(input_b ** 2, 1) - 2 * np.dot(input_a, input_b.T)
        return np.exp(-.5 * (1 / length_scale) * sqdist)


    def phoneme_distance_kernel(self, phoneme_set_a, phoneme_set_b):
        """
        Calculates word matrix covariance via a phonetic-ONLY distance metric
        """
        #k = np.empty((len(word_set_a), len(word_set_b),))

        #first, validate that these are all words in our curriculum
        for elem in set().union(phoneme_set_a, phoneme_set_b):
            if not elem in self.curriculum:
                raise Exception(elem + ' is not a phoneme in our curriculum')

        k = np.ones((len(phoneme_set_a), len(phoneme_set_b)))
        for i in range(len(phoneme_set_a)):
            for j in range(len(phoneme_set_b)):
                k[i][j] = self.get_phoneme_cov(phoneme_set_a[i], phoneme_set_b[j])

        return k

    def get_phoneme_cov(self, phoneme1, phoneme2):
        """
        Returns specific values from the phoneme covariance matrix
        """

        index1 = self.curriculum.index(phoneme1)
        index2 = self.curriculum.index(phoneme2)

        #print("covariance between " + phoneme1 + " and " + phoneme2)
        #print(round(self.loaded_covariance_matrix[index1][index2], 3))
        return round(self.loaded_covariance_matrix[index1][index2], 3)

    def get_mean(self, X):
        """
        Implements a mean function for the Gaussian Process. Returns a list of means
        Given a list of words
        """

        #handles both vectors and scalars
        if len(X) > 0:
            return [0.5] * len(X)
        else:
            return 0.5

    def get_phoneme_mean(self, word):
        """
        gets the phoneme-based mean of a word, given the current model
        """
        phonemes_raw = pronouncing.phones_for_word(word.lower())[0].split(' ')
        phonemes = [''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
        
        phoneme_means = []
        for i in range(0, len(phonemes)):
            for j in range(0, len(self.curriculum)):
                if phonemes[i] == self.curriculum[j]:
                    phoneme_means.append(self.means[j])        
        return np.mean(phoneme_means)


    def get_phoneme_var(self, word):
        """
        gets the phoneme-based variance of a word, given the current model
        """
        phonemes_raw = pronouncing.phones_for_word(word.lower())[0].split(' ')
        phonemes = [''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
        
        phoneme_vars = []
        for i in range(0, len(phonemes)):
            for j in range(0, len(self.curriculum)):
                if phonemes[i] == self.curriculum[j]:
                    phoneme_vars.append(self.variances[j])        
        return np.mean(phoneme_vars)

    def init_plot(self):
        self.fig, self.plts = plt.subplots(self.n_rows, math.ceil(len(self.curriculum)/self.n_rows),
                                           figsize=(15, 10))

    def plot_curricular_distro(self):
        """
        Plots the most recent distribution
        """

        left = 0.125  # the left side of the subplots of the figure
        right = 0.9  # the right side of the subplots of the figure
        bottom = 0.4  # the bottom of the subplots of the figure
        top = 0.9  # the top of the subplots of the figure
        wspace = 1.0  # the amount of width reserved for blank space between subplots
        hspace = 0.2  # the amount of height reserved for white space between subplots

        for i in range(len(self.curriculum)):
            row_index = int(i / (len(self.curriculum) / self.n_rows))
            col_index = int(i % (len(self.curriculum) / self.n_rows))
            self.plts[row_index][col_index].set_xlim([-.5, 1.5])
            self.plts[row_index][col_index].set_ylim([0, 3])

            x = np.linspace(-3, 3, 50)
            self.clear_old_plot_lines()
            self.plts[row_index][col_index].plot(x, scipy.stats.norm.pdf(x, self.means[i],
                                                                         self.variances[i]),
                                                 color='b')
            self.plts[row_index][col_index].set_title(
                self.curriculum[i] + ":\n u= " + str(round(self.means[i], 2)) + ", var= " + str(
                    round(self.variances[i], 2)))

        plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
        self.fig.tight_layout()

        plt.show(block=False)
        self.fig.canvas.flush_events()
        plt.draw()

    def clear_old_plot_lines(self):
        """
        clears the previous lines that were drawn on the graph
        """
        for plotRow in self.plts:
            for plot in plotRow:
                if len(plot.lines) > 1:
                    del plot.lines[0]

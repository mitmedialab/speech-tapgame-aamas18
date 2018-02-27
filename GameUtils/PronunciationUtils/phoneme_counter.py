import pronouncing
import csv
import os
from GameUtils.Curriculum import Curriculum

class PhonemeCounter():
	def __init__(self):
		self.load_phoneme_dict()
		self.build_curriculum()
		self.count_all_phonemes()

	def build_curriculum(self):
	    # fancy python one-liner to read all string attributes off of a class
	    self.curriculum = [p for p in dir(Curriculum)
	                       if isinstance(getattr(Curriculum, p), str)
	                       and not p.startswith('__')]

	def load_phoneme_dict(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))

		# dictionary to store arpabet mapping
		self.phoneme_dict = {}
		with open(dir_path + '/data/arpabet_mapping.csv', 'r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				# row[1] is CMU arpabet, row[0] is modified version of arpabet for NETTalk database
				self.phoneme_dict.update({row[1]: 0})

	def get_and_update_phonemes(self, word):
		"""
		align a given word's graphemes with its phonemes using Nettalk
		"""
		print(word)
		# need to use lowercase form of word for pronouncing and nettalk dict lookup
		phonemes_raw = pronouncing.phones_for_word(word.lower())[0].split(' ')
		phonemes = [''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]

		print(phonemes)

		for p in phonemes:
			self.phoneme_dict[p] += 1

	def count_all_phonemes(self):
		for word in self.curriculum:
			self.get_and_update_phonemes(word)

		print(len(self.curriculum))
		print(self.phoneme_dict)

	# def count_phonemes(self)

if __name__ == '__main__':
	counter = PhonemeCounter()














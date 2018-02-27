import rosbag
import os
import sys
import glob
import subprocess
import yaml
import numpy as np
import csv
import math

AUDIO_DATA_FILE = os.getcwd() + '/rosbag/finished_audios_name_score.csv'

# MUST BE RUN IN PYTHON 2 ENVIRONMENT

# this command will republish compressed images from usb_cam/image_raw/compressed as decompressed images which can
# be viewed using image view (rosrun image_transport republish compressed in:=usb_cam/image_raw raw out:=cam_out/decompressed)

# rosrun image_transport republish compressed in:=usb_cam/image_raw raw out:=cam_out/decompressed

# rosrun bag_tools make_video.py /usb_cam/image_raw/compressed rosbag/p01_sam_experiment_2017-10-06-13-18-19.bag --fps 30


#TEST_BAG_PATH = os.getcwd() + '/rosbag/jayden_test_long.bag' 
#TEST_BAG_PATH = os.getcwd() + '/rosbag/p01_sam_experiment_2017-10-06-19-13-55.bag' 


test = '100  349k  100  5542  100  344k   1832   113k  0:00:03  0:00:03 --:--:--  114k{"status": "success", "text_score": {"text": "CAT", "word_intonation_list": [{"word": "CAT", "syllable_intonation_list": [[null, "RISE"]]}], "quality_score": 26.0, "word_score_list": [{"phone_score_list": [{"child_phones": [{"sound_most_like": "t", "quality_score": 3.7129629629629624, "extent": [58, 60]}, {"sound_most_like": "l", "quality_score": 1.9444444444444444, "extent": [60, 61]}, {"sound_most_like": "n", "quality_score": 1.9444444444444444, "extent": [61, 62]}, {"sound_most_like": "d", "quality_score": 1.9444444444444444, "extent": [62, 63]}, {"sound_most_like": "t", "quality_score": 2.3148148148148144, "extent": [63, 65]}, {"sound_most_like": "v", "quality_score": 1.9444444444444444, "extent": [65, 70]}, {"sound_most_like": "th", "quality_score": 1.9444444444444444, "extent": [70, 71]}, {"sound_most_like": "dh", "quality_score": 1.9444444444444444, "extent": [71, 74]}, {"sound_most_like": "b", "quality_score": 1.9444444444444444, "extent": [74, 75]}, {"sound_most_like": "dh", "quality_score": 1.9444444444444444, "extent": [75, 76]}, {"sound_most_like": "th", "quality_score": 1.9444444444444444, "extent": [76, 77]}, {"sound_most_like": "dh", "quality_score": 1.9444444444444444, "extent": [77, 79]}, {"sound_most_like": "th", "quality_score": 1.9444444444444444, "extent": [79, 80]}, {"sound_most_like": "d", "quality_score": 1.9444444444444444, "extent": [80, 82]}, {"sound_most_like": "dh", "quality_score": 1.9444444444444444, "extent": [82, 83]}, {"sound_most_like": "d", "quality_score": 1.9444444444444444, "extent": [83, 84]}, {"sound_most_like": "v", "quality_score": 1.9444444444444444, "extent": [84, 85]}, {"sound_most_like": "b", "quality_score": 1.9444444444444444, "extent": [85, 86]}, {"sound_most_like": "v", "quality_score": 4.777777777777778, "extent": [86, 87]}, {"sound_most_like": "jh", "quality_score": 5.037037037037037, "extent": [87, 90]}, {"sound_most_like": "k", "quality_score": 5.555555555555555, "extent": [90, 91]}, {"sound_most_like": "jh", "quality_score": 4.888888888888888, "extent": [91, 93]}, {"sound_most_like": "t", "quality_score": 4.555555555555555, "extent": [93, 94]}, {"sound_most_like": "w", "quality_score": 1.9444444444444444, "extent": [94, 95]}], "stress_level": null, "quality_score": 2.714714714714715, "phone": "k", "extent": [58, 95], "sound_most_like": "dh"}, {"child_phones": [{"sound_most_like": "w", "quality_score": 58.722222222222229, "extent": [95, 101]}, {"sound_most_like": "ay", "quality_score": 34.999999999999986, "extent": [101, 110]}, {"sound_most_like": "ow", "quality_score": 35.0, "extent": [110, 112]}, {"sound_most_like": "eh", "quality_score": 78.833333333333329, "extent": [112, 120]}, {"sound_most_like": "ey", "quality_score": 88.0, "extent": [120, 121]}, {"sound_most_like": "ow", "quality_score": 82.0, "extent": [121, 122]}, {"sound_most_like": "ah", "quality_score": 92.0, "extent": [122, 123]}, {"sound_most_like": "ay", "quality_score": 92.0, "extent": [123, 125]}, {"sound_most_like": "ah", "quality_score": 86.0, "extent": [125, 126]}, {"sound_most_like": "eh", "quality_score": 86.0, "extent": [126, 127]}, {"sound_most_like": "ay", "quality_score": 92.66666666666666, "extent": [127, 130]}], "stress_score": 100.0, "stress_level": 1, "quality_score": 64.685714285714283, "phone": "ae", "extent": [95, 130], "sound_most_like": "ay"}, {"child_phones": [{"sound_most_like": "eh", "quality_score": 15.0, "extent": [130, 131]}, {"sound_most_like": "m", "quality_score": 5.833333333333333, "extent": [131, 132]}, {"sound_most_like": "aa", "quality_score": 3.611111111111111, "extent": [132, 134]}, {"sound_most_like": "p", "quality_score": 14.5, "extent": [134, 136]}, {"sound_most_like": "f", "quality_score": 5.833333333333333, "extent": [136, 138]}, {"sound_most_like": "aa", "quality_score": 3.055555555555556, "extent": [138, 139]}, {"sound_most_like": "k", "quality_score": 14.666666666666666, "extent": [139, 140]}, {"sound_most_like": "aa", "quality_score": 1.3888888888888895, "extent": [140, 141]}, {"sound_most_like": "f", "quality_score": 15.666666666666666, "extent": [141, 142]'

class RosbagAnalyzer:

	def __init__(self, argv):

		## load the bag from the first arg		
		split_bag = argv[1].split('/')[-1].split('_')
		participant_id = split_bag[0]
		experimenter_id = split_bag[1]
		experiment_phase = split_bag[2]
		
		self.current_bag_path = os.getcwd() + '/' + argv[1]
		self.current_bag = rosbag.Bag(self.current_bag_path)

		## load the audio data from the second arg

		self.audio_data_files = []
		self.audio_data_scores = []
		with open(argv[2], 'rb') as csvfile:
			audio_data = csv.reader(csvfile)

			for row in audio_data:
				self.audio_data_files.append(row[0])
				self.audio_data_scores.append(float(row[1]))

		self.summarize_bag(self.current_bag)
		self.analyze_round_summaries(self.current_bag)

		# WAV_OUTPUT_FILENAME_PREFIX = '/GameUtils/PronunciationUtils/data/recordings/' +\
		# 	participant_id + '_' + experimenter_id + '_' + experiment_phase + '_*'

		# print('getting all matching audios from')
		# print(os.getcwd() + WAV_OUTPUT_FILENAME_PREFIX)
		# all_audios = glob.glob(os.getcwd() + WAV_OUTPUT_FILENAME_PREFIX)
		# print(all_audios)
		# print('-----------------------')

		# for audio in all_audios:
		# 	filename = audio.split('/')[-1]
		# 	round_word = filename.split('_')[3]
		# 	round_index = int(filename.split('_')[4][:-4]) # 4th index by split, ignore last 4 chars (.wav)
		# 	print("Round: " + str(round_index) + " word was " + round_word)



	# get some summary info about the bag

	def summarize_bag(self, bag):

		print("SUMMARIZING BAG")
		print(bag)


		bag_info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', self.current_bag_path],
		 stdout=subprocess.PIPE).communicate()[0])
		print("heres a handy info dict about the bag")
		print(bag_info_dict)

		topics = bag.get_type_and_topic_info()[1].keys()
		types = []

		for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
			types.append(bag.get_type_and_topic_info()[1].values()[i][0])

		print('------------')

		print("total message count")
		print(bag.get_message_count())
		print('------------')
		for i in range(len(topics)):
			print("topic \"" + str(topics[i]) + "\" message count")
			print(str(bag.get_message_count(topics[i])) + " messages")
			print("of type " + types[i])
			print('------------')
	
	
	def analyze_round_summaries(self, tap_game_bag):
		msgs = tap_game_bag.read_messages()
		round_summary_msgs = []
		audios = []

		total_ceiling_loss = 0
		total_floor_loss = 0
		total_rounded_loss = 0
		total_no_change_loss = 0

		for topic, msg, time in tap_game_bag.read_messages():
			#print topic
			if topic == '/tap_game_round_summary':
				round_summary_msgs.append(msg)
				audios.append(msg.audio_file)
				
		print(audios)
		for msg in round_summary_msgs:

			# calculate speechace loss for round
			ceiling, floor, rounded, no_change = self.calculate_speechace_loss(msg)
			total_ceiling_loss += ceiling
			total_floor_loss += floor
			total_rounded_loss += rounded
			total_no_change_loss += no_change

		print("TOTAL CEILING, FLOOR, ROUNDED, and NO_CHANGE LOSS")
		print(total_ceiling_loss)
		print(total_floor_loss)
		print(total_rounded_loss)
		print(total_no_change_loss)


		#Analyze final message for GP Means / Variances
		final_msg = round_summary_msgs[-1]
		print(final_msg)
		self.calculate_gp_loss(final_msg)

		

	def calculate_gp_loss(self, msg):
		"""
		computes several different forms of loss of speechace results compared to MTurk audio results
		"""
		

	def calculate_speechace_loss(self, msg):
		"""
		computes several different forms of loss of speechace results compared to MTurk audio results
		"""
		audio_data_candidate = msg.audio_file.split('/')[-1]

		ceiling_loss = 0
		floor_loss = 0
		rounded_loss = 0
		no_change_loss = 0

		if audio_data_candidate in self.audio_data_files:
			print(audio_data_candidate)
			audio_index = self.audio_data_files.index(audio_data_candidate)
			binned_avg_score = np.mean(msg.scores) / 20
			ground_truth_rating = self.audio_data_scores[audio_index]

			print(ground_truth_rating) 
			print(binned_avg_score) 

			# UNCOMMENT TO USE L1 LOSS
			# no_change_loss = abs(binned_avg_score - ground_truth_rating)
			# ceiling_loss = abs(math.ceil(binned_avg_score) - ground_truth_rating)
			# floor_loss = abs(math.floor(binned_avg_score) - ground_truth_rating)
			# rounded_loss = abs(round(binned_avg_score) - ground_truth_rating)

			# UNCOMMENT TO USE L2 LOSS
			no_change_loss = (binned_avg_score - ground_truth_rating)**2
			ceiling_loss = (math.ceil(binned_avg_score) - ground_truth_rating)**2
			floor_loss = (math.floor(binned_avg_score) - ground_truth_rating)**2
			rounded_loss = (round(binned_avg_score) - ground_truth_rating)**2

			print(ceiling_loss)
			print(floor_loss)
			print(rounded_loss)

		else:
			print("NO GROUND TRUTH RATING IN OUR DATASET")		

		return ceiling_loss, floor_loss, rounded_loss, no_change_loss

def speechace_send():
	pass

if __name__ == '__main__':
	# print(sys.argv)
	# if not len(sys.argv) == 3:
	# 	print("Usage: python2 -m scripts.rosbag_analysis [path-to-bag-file] [path-to-audio-results]") 
	# else:
	# 	myRBA = RosbagAnalyzer(sys.argv)		
	print(test)
	json_start_index = test.index('{')
	print(json_start_index)
	print(len(test))
	print(test[json_start_index:])
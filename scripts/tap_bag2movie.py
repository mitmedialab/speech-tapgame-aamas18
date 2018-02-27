#!/usr/bin/python

import subprocess
import glob

"""
This script is intended to collect all of the experiment + post-test bags with pids in PARTICIPANT_IDS and 
located in the ROSBAG_DATA_PATH, then turn them into mp4 movies
"""

PARTICIPANT_IDS = ['p16', 'p17', 'p19', 'p21', 'p22', 'p23', 'p24']
#PARTICIPANT_IDS = ['p03', 'p05', 'p06', 'p08', 'p09', 'p11', 'p12', 'p15', 'p16', 'p17', 'p19', 'p21', 'p22', 'p23', 'p24']

ROSBAG_DATA_PATH = "../rosbag/"
GLOBAL_BAG_DICT = {}

def main():
	# Get all bags
	for pid in PARTICIPANT_IDS:
	    practice_bags = glob.glob(ROSBAG_DATA_PATH + pid + '/*_practice_*.bag')
	    experiment_bags = glob.glob(ROSBAG_DATA_PATH + pid + '/*_experiment_*.bag')
	    posttest_bags = glob.glob(ROSBAG_DATA_PATH + pid + '/*_posttest_*.bag')
	    GLOBAL_BAG_DICT[pid + '_practice'] = practice_bags
	    GLOBAL_BAG_DICT[pid + '_experiment'] = experiment_bags
	    GLOBAL_BAG_DICT[pid + '_posttest'] = posttest_bags
	    print(pid)
	    print(len(experiment_bags))
	    print(len(posttest_bags))
	    print('-----')

	print(GLOBAL_BAG_DICT)


	for pid in PARTICIPANT_IDS:
		exp_bags = GLOBAL_BAG_DICT[pid + '_' + 'experiment']
		post_bags = GLOBAL_BAG_DICT[pid + '_' + 'posttest']

		print(exp_bags)
		print(post_bags)
		
		for i in range(0, len(exp_bags)):
			convert_bag_to_movie(exp_bags[i], pid, 'experiment', i)

		for i in range(0, len(post_bags)):
			convert_bag_to_movie(post_bags[i], pid, 'posttest', i)


def convert_bag_to_movie(bag_filename, pid, phase, vid_index):
	#pid = bag_filename.split('_')[0][:-3]
	#phase = bag_filename.split('_')[3]
	print("converting bag " + pid + phase)
	print(bag_filename)	
	CONVERSION_CMD = "rosrun bag_tools make_video.py /usb_cam/image_raw/compressed " + bag_filename + " --fps 30"
	pouts = subprocess.check_output(CONVERSION_CMD, shell=True)
	RENAME_CMD = "mv video.mp4 " + '../movies/' + pid + '_' + phase + '_' + str(vid_index) + '.mp4' 
	pouts = subprocess.check_output(RENAME_CMD, shell=True)

if __name__ == '__main__':
	main()
#!/usr/bin/env python

# This will execute the python script in the current environment
import yaml
import sys

import numpy as np

# Ensure that we have sourced our virtual environment
#import tensorflow as tf


class TransitionData:
	def __init__(self):
		self.position = np.array([0,0,0])
		self.quaternion_orientation = np.array([0,0,0,0])


def load_file(filepath):
	with open(filepath, 'r') as stream:
	    data_loaded = yaml.load(stream)
	    #print data_loaded	

	training_data = TransitionData()
	training_data.position[0] = data_loaded["swing_foot_starting_position"]["x"]
	training_data.position[1] = data_loaded["swing_foot_starting_position"]["y"]
	training_data.position[2] = data_loaded["swing_foot_starting_position"]["z"]

	print "training_data.position", training_data.position




if __name__ == "__main__":
	# print (sys.argv)
	for item in sys.argv:
		if '.yaml' in item:
			filepath = item
			print "loading yaml file:", item
			load_file(filepath)

			break
#!/usr/bin/env python

# This will execute the python script in the current environment
import yaml
import sys

import numpy as np

# Ensure that we have sourced our virtual environment
#import tensorflow as tf


def convert_quat_to_3vec(quat):
	# Normalize and extract quaternion
	x, y, z, w = quat/np.linalg.norm(quat)
	# Compute theta
	theta = np.arccos(w)

	if (theta <= 1e-6):
		return np.array([0.0, 0.0, 0.0])
	else:
		vec = np.array([x,y,z]) / np.sin(theta)
		vec = vec / np.linalg.norm(vec) * theta
		return vec 

class TransitionData:
	def __init__(self):
		self.path = ""

		self.stance_origin = ""
		self.manipulation_type = ""
		self.result = 0

		self.swing_foot_starting_position = np.array([0,0,0], dtype='f') #x, y, z
		self.swing_foot_starting_orientation_quat = np.array([0,0,0,0], dtype='f') #x,y,z,w

		self.pelvis_starting_position = np.array([0,0,0], dtype='f') #x, y, z
		self.pelvis_starting_orientation_quat = np.array([0, 0, 0, 0], dtype='f') #x,y,z,w


		self.left_hand_starting_position = np.array([0,0,0], dtype='f')
		self.left_hand_starting_orientation_quat = np.array([0,0,0,0], dtype='f')

		self.right_hand_starting_position = np.array([0,0,0], dtype='f')
		self.right_hand_starting_orientation_quat = np.array([0,0,0,0], dtype='f')

		self.landing_foot_position = np.array([0,0,0], dtype='f')
		self.landing_foot_orientation_quat = np.array([0,0,0,0], dtype='f')

	def loadPosition(self, data_loaded, key, pos):
		pos[0] = data_loaded[key]["x"]
		pos[1] = data_loaded[key]["y"]
		pos[2] = data_loaded[key]["z"]		

	def loadOrientation(self, data_loaded, key, ori):
		ori[0] = data_loaded[key]["x"]
		ori[1] = data_loaded[key]["y"]
		ori[2] = data_loaded[key]["z"]	
		ori[3] = data_loaded[key]["w"]			


	def loadData(self, data_loaded):
		self.path = filepath 

		self.stance_origin = data_loaded["stance_origin"] 
		self.manipulation_type = data_loaded["manipulation_type"]		
		self.result = 1 if data_loaded["result"] == "success" else 0

		self.loadPosition(data_loaded, "swing_foot_starting_position", self.swing_foot_starting_position)
		self.loadOrientation(data_loaded, "swing_foot_starting_orientation", self.swing_foot_starting_orientation_quat)

		self.loadPosition(data_loaded, "pelvis_starting_position", self.pelvis_starting_position)
		self.loadOrientation(data_loaded, "pelvis_starting_orientation", self.pelvis_starting_orientation_quat)

		self.loadPosition(data_loaded, "left_hand_starting_position", self.left_hand_starting_position)
		self.loadOrientation(data_loaded, "left_hand_starting_orientation", self.left_hand_starting_orientation_quat)

		self.loadPosition(data_loaded, "right_hand_starting_position", self.right_hand_starting_position)
		self.loadOrientation(data_loaded, "right_hand_starting_orientation", self.right_hand_starting_orientation_quat)

		self.loadPosition(data_loaded, "landing_foot_position", self.landing_foot_position)
		self.loadOrientation(data_loaded, "landing_foot_orientation", self.landing_foot_orientation_quat)


	def printData(self):
		print "yaml file:", self.path
		print " "
		print "  stance_origin", self.stance_origin
		print "  manipulation_type" , self.manipulation_type
		print "  result" , self.result
		print " "
		print "  swing_foot_starting_position" , self.swing_foot_starting_position
		print "  swing_foot_starting_orientation_quat" , self.swing_foot_starting_orientation_quat
		print " "
		print "  pelvis_starting_position" , self.pelvis_starting_position
		print "  pelvis_starting_orientation_quat" , self.pelvis_starting_orientation_quat
		print " "
		print "  left_hand_starting_position" , self.left_hand_starting_position
		print "  left_hand_starting_orientation_quat" , self.left_hand_starting_orientation_quat
		print " "
		print "  right_hand_starting_position" , self.right_hand_starting_position
		print "  right_hand_starting_orientation_quat" , self.right_hand_starting_orientation_quat
		print " "
		print "  landing_foot_position" , self.landing_foot_position
		print "  landing_foot_orientation_quat" , self.landing_foot_orientation_quat


def load_file(filepath):
	with open(filepath, 'r') as stream:
	    data_loaded = yaml.load(stream)
	    # print data_loaded	
	training_data = TransitionData()

	training_data.loadData(data_loaded)
	training_data.printData()




if __name__ == "__main__":
	# print (sys.argv)
	for item in sys.argv:
		if '.yaml' in item:
			filepath = item
			print "loading yaml file:", item
			load_file(filepath)

			break
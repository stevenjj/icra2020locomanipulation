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

class ContactTransitionData:
	def __init__(self, yaml_file_path=""):
		self.stance_origin_to_num = { "left_foot": 0, "right_foot":1 }
		self.manipulation_type_to_num = { "left_hand": 0, "right_hand":1, "both_hands":2 }
		
		self.path = ""

		self.stance_origin = ""
		self.stance_origin_num = 0

		self.manipulation_type = ""
		self.manipulation_type_num = 0
		self.result = 0

		self.swing_foot_starting_position = np.array([0,0,0], dtype='f') #x, y, z
		self.swing_foot_starting_orientation_quat = np.array([0,0,0,0], dtype='f') #x,y,z,w
		self.swing_foot_starting_orientation_vec = np.array([0,0,0], dtype='f') #rx,ry,rz

		self.pelvis_starting_position = np.array([0,0,0], dtype='f') #x, y, z
		self.pelvis_starting_orientation_quat = np.array([0, 0, 0, 0], dtype='f') #x,y,z,w
		self.pelvis_starting_orientation_vec = np.array([0,0,0], dtype='f') #rx,ry,rz

		self.left_hand_starting_position = np.array([0,0,0], dtype='f')
		self.left_hand_starting_orientation_quat = np.array([0,0,0,0], dtype='f')
		self.left_hand_starting_orientation_vec = np.array([0,0,0], dtype='f') #rx,ry,rz

		self.right_hand_starting_position = np.array([0,0,0], dtype='f')
		self.right_hand_starting_orientation_quat = np.array([0,0,0,0], dtype='f')
		self.right_hand_starting_orientation_vec = np.array([0,0,0], dtype='f') #rx,ry,rz

		self.landing_foot_position = np.array([0,0,0], dtype='f')
		self.landing_foot_orientation_quat = np.array([0,0,0,0], dtype='f')
		self.landing_foot_orientation_vec =  np.array([0,0,0], dtype='f') #rx,ry,rz

		self.set_x()
		self.set_y()

		if (yaml_file_path != ""):
			self.load_yaml_file(yaml_file_path)


	def set_x(self):
		# sets the x vector for the training data
		# stance_origin, manipulation_type
		self.x = np.concatenate( (self.swing_foot_starting_position, self.swing_foot_starting_orientation_vec,
								  self.pelvis_starting_position, self.pelvis_starting_orientation_vec,
								  self.left_hand_starting_position,self.left_hand_starting_orientation_vec,
								  self.right_hand_starting_position,self.right_hand_starting_orientation_vec,
								  self.landing_foot_position, self.landing_foot_orientation_vec) )
	def set_y(self):
		self.y = self.result

	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	def loadPosition(self, data_loaded, key, pos):
		pos[0] = data_loaded[key]["x"]
		pos[1] = data_loaded[key]["y"]
		pos[2] = data_loaded[key]["z"]		

	def loadOrientation(self, data_loaded, key, ori):
		ori[0] = data_loaded[key]["x"]
		ori[1] = data_loaded[key]["y"]
		ori[2] = data_loaded[key]["z"]	
		ori[3] = data_loaded[key]["w"]			

	def load_yaml_file(self, yaml_file_path):
		with open(filepath, 'r') as stream:
			data_loaded = yaml.load(stream)
			self.path = yaml_file_path		
			self.loadData(data_loaded)

	def loadData(self, data_loaded):
		self.stance_origin = data_loaded["stance_origin"] 
		self.manipulation_type = data_loaded["manipulation_type"]		
		self.result = 1 if data_loaded["result"] == "success" else 0

		self.stance_origin_num = self.stance_origin_to_num[self.stance_origin]
		self.manipulation_type_num = self.manipulation_type_to_num[self.manipulation_type]

		self.loadPosition(data_loaded, "swing_foot_starting_position", self.swing_foot_starting_position)
		self.loadOrientation(data_loaded, "swing_foot_starting_orientation", self.swing_foot_starting_orientation_quat)
		self.swing_foot_starting_orientation_vec = convert_quat_to_3vec(self.swing_foot_starting_orientation_quat)

		self.loadPosition(data_loaded, "pelvis_starting_position", self.pelvis_starting_position)
		self.loadOrientation(data_loaded, "pelvis_starting_orientation", self.pelvis_starting_orientation_quat)
		self.pelvis_starting_orientation_vec = convert_quat_to_3vec(self.pelvis_starting_orientation_quat)

		self.loadPosition(data_loaded, "left_hand_starting_position", self.left_hand_starting_position)
		self.loadOrientation(data_loaded, "left_hand_starting_orientation", self.left_hand_starting_orientation_quat)
		self.left_hand_starting_orientation_vec = convert_quat_to_3vec(self.left_hand_starting_orientation_quat)

		self.loadPosition(data_loaded, "right_hand_starting_position", self.right_hand_starting_position)
		self.loadOrientation(data_loaded, "right_hand_starting_orientation", self.right_hand_starting_orientation_quat)
		self.right_hand_starting_orientation_vec = convert_quat_to_3vec(self.right_hand_starting_orientation_quat)

		self.loadPosition(data_loaded, "landing_foot_position", self.landing_foot_position)
		self.loadOrientation(data_loaded, "landing_foot_orientation", self.landing_foot_orientation_quat)
		self.landing_foot_orientation_vec = convert_quat_to_3vec(self.landing_foot_orientation_quat)

		self.set_x()
		self.set_y()

	def printData(self):
		print "yaml file:", self.path
		print " "
		print "  stance_origin:", self.stance_origin, ", num representation = ", self.stance_origin_num
		print "  manipulation_type:" , self.manipulation_type, ", num representation = ",  self.manipulation_type_num
		print "  result:" , self.result
		print " "
		print "  swing_foot_starting_position:" , self.swing_foot_starting_position
		print "  swing_foot_starting_orientation_quat:" , self.swing_foot_starting_orientation_quat
		print "  swing_foot_starting_orientation_vec:" , self.swing_foot_starting_orientation_vec
		print " "
		print "  pelvis_starting_position:" , self.pelvis_starting_position
		print "  pelvis_starting_orientation_quat:" , self.pelvis_starting_orientation_quat
		print "  pelvis_starting_orientation_vec:" , self.pelvis_starting_orientation_vec
		print " "
		print "  left_hand_starting_position:" , self.left_hand_starting_position
		print "  left_hand_starting_orientation_quat:" , self.left_hand_starting_orientation_quat
		print "  left_hand_starting_orientation_vec:" , self.left_hand_starting_orientation_vec
		print " "
		print "  right_hand_starting_position:" , self.right_hand_starting_position
		print "  right_hand_starting_orientation_quat:" , self.right_hand_starting_orientation_quat
		print "  right_hand_starting_orientation_vec:" , self.right_hand_starting_orientation_vec
		print " "
		print "  landing_foot_position:" , self.landing_foot_position
		print "  landing_foot_orientation_quat:" , self.landing_foot_orientation_quat
		print "  landing_foot_orientation_vec:" , self.landing_foot_orientation_vec


def test_load_file(filepath):
	training_data = ContactTransitionData(filepath)
	training_data.printData()
	print "x = ", training_data.get_x()
	print "y = ", training_data.get_y()

	# training_data = ContactTransitionData()
	#training_data.load_yaml_file(filepath)


if __name__ == "__main__":
	# print (sys.argv)
	for item in sys.argv:
		if '.yaml' in item:
			filepath = item
			print "loading yaml file:", item
			test_load_file(filepath)

			break
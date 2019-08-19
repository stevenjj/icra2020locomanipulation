#!/usr/bin/env python
import os

import yaml
import sys

import load_training_datum as datum


global dataset_folder
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"


class ContactTransitionDataset:
	def __init__(self):
		self.num = 10000 # Total number of training data

		self.x = None
		self.y = None

		self.x_train_mean = None
		self.x_train_std = None

		self.x_train = None
		self.y_train = None

		self.x_val = None
		self.y_val = None

		self.x_test = None
		self.y_test = None

	def load_dataset(self, filepath):
		yaml_file_path = filepath + "/positive_examples"
		yaml_files = os.listdir(yaml_file_path)

		for file in yaml_files:
			# store data
			print file

		print len(yaml_files)

	def normalize_dataset(self):
		self.x


if __name__ == "__main__":
	print 'hello world'
	print os.listdir(dataset_folder)

	dataset = ContactTransitionDataset()
	dataset.load_dataset(dataset_folder)
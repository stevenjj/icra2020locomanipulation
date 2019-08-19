#!/usr/bin/env python
import os

import numpy as np

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

    def load_data_from_path(self, filepath, max_load=10):
        # get the yaml files from this directory
        yaml_files = os.listdir(filepath)

        count = 0
        obj = datum.ContactTransitionData()
        for file in yaml_files:
            if (count < max_load):
                print file 
                # Load the yaml file and add the training data
                obj.load_yaml_file(filepath + "/" + file)
                self.x.append(obj.get_x())
                self.y.append(obj.get_y())
                # increment counter
                count += 1

        print len(yaml_files)

    def load_dataset(self, filepath):
        print os.listdir(filepath)
        self.x = []
        self.y = []
        self.load_data_from_path(filepath + "/positive_examples", 10)
        self.load_data_from_path(filepath + "/negative_examples", 10)

        self.x = np.array(self.x)
        self.y = np.array(self.y)

        print self.x.shape
        print self.y.shape

    def normalize_dataset(self):
        self.x


if __name__ == "__main__":
    dataset = ContactTransitionDataset()
    dataset.load_dataset(dataset_folder)
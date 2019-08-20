#!/usr/bin/env python
import os

import numpy as np

import yaml
import sys

import load_training_datum as datum


global dataset_folder
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"


class ContactTransitionDataset:
    def __init__(self, num_in = 100):
        self.num = num_in # Total number of training data

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

    def load_data_from_path(self, filepath, max_load=100):
        # get the yaml files from this directory
        yaml_files = os.listdir(filepath)

        count = 0
        obj = datum.ContactTransitionData()
        for file in yaml_files:
            if (count < max_load):
                # Load the yaml file and add the training data
                if obj.load_yaml_file(filepath + "/" + file):
                    print "Loading", count+1 , "/", max_load,  filepath, "/", file 
                    # obj.printData()
                    self.x.append(obj.get_x())
                    self.y.append(obj.get_y())
                    # increment counter
                    count += 1
            else:
                break

        print len(yaml_files)

    def load_dataset(self, filepath):
        print os.listdir(filepath)

        self.x = []
        self.y = []
        self.load_data_from_path(filepath + "/positive_examples", self.num)
        self.load_data_from_path(filepath + "/negative_examples", self.num)

        # turn to numpy array
        self.x = np.array(self.x)
        self.y = np.array(self.y)

        self.normalize_dataset()


    def normalize_dataset(self):
        self.x_train_mean = np.mean(self.x, axis = 0)
        self.x_train_std = np.std(self.x, axis = 0)
        self.x_train = (self.x - self.x_train_mean) / self.x_train_std

        self.y_train = self.y

    def get_normalized_xy_train(self):
        return (self.x_train, self.y_train)


if __name__ == "__main__":
    dataset = ContactTransitionDataset()
    dataset.load_dataset(dataset_folder)

    print dataset.x_train.shape
    print "dataset dim = ", dataset.x_train.shape[1] 

    print "unnormalized data:"
    print dataset.x, dataset.y
    print "normalized data"
    print dataset.x_train, dataset.y
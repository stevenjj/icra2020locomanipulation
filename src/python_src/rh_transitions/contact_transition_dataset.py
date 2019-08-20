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

        self.x = np.array([])
        self.y = np.array([])

        self.x_train_mean = None
        self.x_train_std = None

        self.x_train = None
        self.y_train = None

        self.x_val = None
        self.y_val = None

        self.x_test = None
        self.y_test = None

    def load_data_from_path(self, filepath, data_x, data_y, max_load=100, stance_type=None, manipulation_type=None):
        # get the yaml files from this directory
        yaml_files = os.listdir(filepath)

        count = 0
        obj = datum.ContactTransitionData()
        for file in yaml_files:
            if (count < max_load):
                # Load the yaml file and add the training data
                if obj.load_yaml_file(filepath + "/" + file, stance_type, manipulation_type):
                    print "Loading", count+1 , "/", max_load,  filepath, "/", file 
                    # obj.printData()
                    data_x.append(obj.get_x())
                    data_y.append(obj.get_y())
                    # increment counter
                    count += 1
            else:
                break

    def load_dataset(self, filepath, stance_type=None, manipulation_type=None):
        print os.listdir(filepath)

        data_x = []
        data_y = []
        self.load_data_from_path(filepath + "/positive_examples", data_x, data_y, self.num, stance_type, manipulation_type)
        self.load_data_from_path(filepath + "/negative_examples", data_x, data_y, self.num, stance_type, manipulation_type)

        print "self.x.shape", self.x.shape

        # Create numpy array if the list was initially empty. Otherwise, append the dataset
        if (len(self.x) == 0 or len(self.y) == 0):          
            print "constructing numpy array"
            self.x = np.array(data_x)
            self.y = np.array(data_y)
        else:
            print "concatenating"
            self.x = np.concatenate( (self.x, np.array(data_x)), axis=0 )
            self.y = np.concatenate( (self.y, np.array(data_y)), axis=0 )

        print "self.x.shape", self.x.shape


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
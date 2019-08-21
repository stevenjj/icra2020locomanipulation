#!/usr/bin/env python
import os
import copy

import numpy as np

import yaml
import sys

import load_training_datum as datum


global dataset_folder
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"


class ContactTransitionDataset:
    def __init__(self, num_in = 100, trainset_percentage_in=0.8, valset_percentage_in=0.2):
        self.num = num_in # Total number of training data

        self.x = np.array([])
        self.y = np.array([])

        self.trainset_percentage = trainset_percentage_in
        self.valset_percentage = valset_percentage_in
        self.testset_percentage = 1.0 - trainset_percentage_in - valset_percentage_in

        self.x_train_mean = None
        self.x_train_std = None

        self.x_train = None
        self.y_train = None

        self.x_val = None
        self.y_val = None

        self.x_test = None
        self.y_test = None

        # Normalized datasets
        self.x_train_normalized = None
        self.x_val_normalized = None
        self.x_test_normalized = None

        # contact transition data
        self.obj = datum.ContactTransitionData()

    def load_data_from_path(self, filepath, data_x, data_y, max_load=100, stance_type=None, manipulation_type=None):
        # get the yaml files from this directory
        yaml_files = os.listdir(filepath)
        count = 0

        # print "Randomizing data load order..."
        # random_indices = np.random.choice(len(yaml_files), len(yaml_files), replace=False)
        # for i in range(0, len(random_indices)):
        for i in range(len(yaml_files)):
            if (count < max_load):
                # Load the yaml file and add the training data
                if self.obj.load_yaml_file(filepath + "/" + yaml_files[i], stance_type, manipulation_type):
                    print "Loading", count+1 , "/", max_load,  filepath, "/", yaml_files[i] 
                    # self.obj.printData()
                    data_x.append(self.obj.get_x())
                    data_y.append(self.obj.get_y())
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


    def prepare_and_normalize_dataset(self):
        self.prepare_dataset()
        self.normalize_dataset()

    def prepare_dataset(self):
        x_dataset_rand, y_dataset_rand = copy.deepcopy(self.get_xy())
        # Randomize the data
        random_indices = np.random.choice(x_dataset_rand.shape[0], x_dataset_rand.shape[0], replace=False)
        for i in range(0, len(random_indices)):
            x_dataset_rand[i] = self.x[random_indices[i]]
            y_dataset_rand[i] = self.y[random_indices[i]]

        self.x = x_dataset_rand
        self.y = y_dataset_rand

        n_train = int(self.trainset_percentage*len(self.x))
        n_val = int(self.valset_percentage*len(self.x))
        n_test = len(self.x) - n_train - n_val

        self.x_train = self.x[:n_train]
        self.y_train = self.y[:n_train]

        self.x_val = self.x[n_train : n_train + n_val]
        self.y_val = self.y[n_train : n_train + n_val]

        self.x_test = self.x[n_train + n_val :]
        self.y_test = self.y[n_train + n_val :]


    def normalize_dataset(self):
        # compute mean and standard deviation
        self.x_train_mean = np.mean(self.x_train, axis = 0)
        x_train_std = np.std(self.x_train, axis = 0)
        # set std to 1.0 if std is near 0:       
        eps = 1e-6
        self.x_train_std = np.array( [elem if abs(elem) >= eps else 1.0 for elem in x_train_std], dtype='f' )

        # Normalize the training data
        if (len(self.x_train) > 0):
            self.x_train_normalized = (self.x_train - self.x_train_mean) / self.x_train_std
        
        if (len(self.x_val) > 0):
            self.x_val_normalized = (self.x_val - self.x_train_mean) / self.x_train_std
        
        if (len(self.x_test) > 0):
            self.x_test_normalized = (self.x_test - self.x_train_mean) / self.x_train_std

    def get_normalized_xy_train(self):
        return (self.x_train_normalized, self.y_train)

    def get_normalized_xy_val(self):
        return (self.x_val_normalized, self.y_val)

    def get_normalized_xy_test(self):
        return (self.x_test_normalized, self.y_test)

    def get_xy(self):
        return (self.x, self.y)


    def enable_right_hand_data(self, bool_input):
        self.obj.enable_right_hand_data(bool_input)

    def enable_left_hand_data(self, bool_input):
        self.obj.enable_left_hand_data(bool_input)

    def enable_stance_origin_data(self, bool_input):
        self.obj.enable_stance_origin_data(bool_input)

if __name__ == "__main__":
    dataset = ContactTransitionDataset()
    dataset.load_dataset(dataset_folder)

    print dataset.x_train.shape
    print "dataset dim = ", dataset.x_train.shape[1] 

    print "unnormalized data:"
    print dataset.x, dataset.y
    print "normalized data"
    print dataset.x_train, dataset.y
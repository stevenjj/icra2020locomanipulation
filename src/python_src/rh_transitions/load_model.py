
#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

print(tf.version.VERSION)
print(tf.keras.__version__)

num_positive_data = 200
contact_transition_types = [ ("right_hand", "left_foot") ]
shorthand = {"right_hand" : "rh", "left_hand" : "lh", "both_hands" : "bh", "right_foot": "rf", "left_foot": "lf"}


def load_normalization_params(yaml_path):
    with open(yaml_path, 'r') as stream:
        data_loaded = yaml.load(stream)	
    return np.array(data_loaded['x_train_mean'], dtype='f'), np.array(data_loaded['x_train_std'], dtype='f')


def load_hyper_params(yaml_path):
    with open(yaml_path, 'r') as stream:
        data_loaded = yaml.load(stream)	
    return {'n_units_per_layer': data_loaded['n_units_per_layer'], 
    		'n_hidden_layers': data_loaded['n_hidden_layers'],
    		'l2_regularization': data_loaded['l2_regularization']}

def get_data(dataset, dataset_folder, contact_transition_types_input):

	for type in contact_transition_types_input:
		subfolder = "/transitions_data_with_task_space_info"
		dataset.load_dataset(dataset_folder + type[0] + subfolder, stance_type=type[1], manipulation_type=type[0], reverse_traversal=True)

	# Count total transition types
	num_transition_types = len(contact_transition_types_input)

	# Get the raw data
	(x_dataset, y_dataset) = dataset.get_xy()

	return x_dataset, y_dataset


# Create a simple dense network for binary classification
def create_model(dataset_dim, num_hidden_layers = 3, units_per_layer=64, l2_reg=0.01):
	# Construct Sequential Model
	model = tf.keras.Sequential()

	for i in range(num_hidden_layers):
		if (i == 0):
			model.add(layers.Dense(units_per_layer, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(l2_reg), input_shape=(dataset_dim,)))
		else:
			model.add(layers.Dense(units_per_layer, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(l2_reg)))			

	# Add the final layer
	model.add(layers.Dense(1, activation='sigmoid'))

	# Model
	model.compile(optimizer=tf.train.AdamOptimizer(0.001),
	              loss='binary_crossentropy',
	              metrics=['accuracy', 'binary_crossentropy', tf.keras.metrics.Precision(), tf.keras.metrics.Recall()])

	return model

# Get the data
dataset_folder = "/home/sjorgen1/Data/param_set_1/"
dataset = transition_dataset.ContactTransitionDataset(num_positive_data)

x_test, y_test = get_data(dataset, dataset_folder, contact_transition_types)
dataset_dim = x_test.shape[1]

# Identify the load path
load_folder = ""
for type in contact_transition_types:
	load_folder = load_folder + shorthand[type[0]] + "_" + shorthand[type[1]]
load_path = "./learned_model/" + load_folder + "/"

# Load the normalization parameters
x_train_mean, x_train_std = load_normalization_params(load_path+'normalization_params.yaml')
# Normalize the dataset
x_test = (x_test - x_train_mean) / x_train_std

# Load and create the model
hyper_params = load_hyper_params(load_path+'hyper_params.yaml')
n_hidden_layers = hyper_params['n_hidden_layers']
n_units_per_layer = hyper_params['n_units_per_layer']
l2_regularization = hyper_params['l2_regularization']

print "hyperparams = ", hyper_params

# best so far
model = create_model(dataset_dim, num_hidden_layers=n_hidden_layers, units_per_layer=n_units_per_layer, l2_reg=l2_regularization)
model.load_weights(load_path)
print model.summary()

# Evaluate
print "Evaluating test set:"
model.evaluate(x_test, y_test)





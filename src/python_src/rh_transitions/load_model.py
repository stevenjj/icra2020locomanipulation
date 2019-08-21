
#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
import numpy as np
import matplotlib.pyplot as plt

print(tf.version.VERSION)
print(tf.keras.__version__)

num_positive_data = 100
contact_transition_types = [ ("left_hand", "right_foot") ]
shorthand = {"right_hand" : "rh", "left_hand" : "lh", "both_hands" : "bh", "right_foot": "rf", "left_foot": "lf"}

def get_data(contact_transition_types_input):
	dataset_folder = "/home/sjorgen1/Data/param_set_1/"
	dataset = transition_dataset.ContactTransitionDataset(num_positive_data)
	dataset.enable_right_hand_data(False)
	dataset.enable_left_hand_data(True)
	dataset.enable_stance_origin_data(False)

	for type in contact_transition_types_input:
		subfolder = "/transitions_data_with_task_space_info"
		dataset.load_dataset(dataset_folder + type[0] + subfolder, stance_type=type[1], manipulation_type=type[0])

	# Count total transition types
	num_transition_types = len(contact_transition_types_input)

	# Get the normalize thefilepath + "/" + data
	(x_dataset, y_dataset) = dataset.get_normalized_xy_train()
	(x_dataset_rand, y_dataset_rand) = dataset.get_normalized_xy_train()

	# Randomize the data
	random_indices = np.random.choice(x_dataset.shape[0], x_dataset.shape[0], replace=False)
	for i in range(0, len(random_indices)):
		x_dataset_rand[i] = x_dataset[random_indices[i]]
		y_dataset_rand[i] = y_dataset[random_indices[i]]

	return x_dataset_rand, y_dataset_rand


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
x_test, y_test = get_data(contact_transition_types)
dataset_dim = x_test.shape[1]

# Load the model
save_folder = ""
for type in contact_transition_types:
	save_folder = save_folder + shorthand[type[0]] + "_" + shorthand[type[1]]

model = create_model(dataset_dim, num_hidden_layers=5, units_per_layer=256, l2_reg=0.001)
model.load_weights("./learned_model/" + save_folder + "/")
print model.summary()


# Evaluate
print "Evaluating test set:"
model.evaluate(x_test, y_test)





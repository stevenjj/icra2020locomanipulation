#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import numpy as np

print(tf.version.VERSION)
print(tf.keras.__version__)

# Load data
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"
total_data = 200
dataset = transition_dataset.ContactTransitionDataset(total_data/2)
dataset.load_dataset(dataset_folder)

# Get the normalize the data
(x_dataset, y_dataset) = dataset.get_normalized_xy_train()
(x_dataset_rand, y_dataset_rand) = dataset.get_normalized_xy_train()
dataset_dim = x_dataset.shape[1] 

# Randomize the data
random_indices = np.random.choice(x_dataset.shape[0], x_dataset.shape[0], replace=False)
for i in range(0, len(random_indices)):
	x_dataset_rand[i] = x_dataset[random_indices[i]]
	y_dataset_rand[i] = y_dataset[random_indices[i]]

trainset_size = int(total_data*0.8)


# Set the training set and test set
x_train = x_dataset_rand[:trainset_size]
y_train = y_dataset_rand[:trainset_size]

x_test = x_dataset_rand[trainset_size:]
y_test = y_dataset_rand[trainset_size:]


# Construct Sequential Model
model = tf.keras.Sequential()
model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01), input_shape=(dataset_dim,)))
model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
model.add(layers.Dense(1, activation='sigmoid'))
# Model
model.compile(optimizer=tf.train.AdamOptimizer(0.001),
              loss='binary_crossentropy',
              metrics=['accuracy'])


# model.add(layers.Dense(2, activation='softmax'))
# # Model
# model.compile(optimizer=tf.train.AdamOptimizer(0.001),
#               loss='sparse_categorical_crossentropy',
#               metrics=['accuracy'])

# Fit
model.fit(x_train, y_train, epochs=50, batch_size=32)

# Evaluate
print 
print "Evaluating test set:"
model.evaluate(x_test, y_test)


# print model.predict(x_train[:])
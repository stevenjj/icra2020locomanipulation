#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

print(tf.version.VERSION)
print(tf.keras.__version__)

# Load data
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"
dataset = transition_dataset.ContactTransitionDataset(20)
dataset.load_dataset(dataset_folder)

# Get the normalize the data
(x_train, y_train) = dataset.get_normalized_xy_train()

print x_train
print y_train

dataset_dim = x_train.shape[1] 

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
model.evaluate(x_train, y_train)

# model.add( layers.Dense(64, activation='relu', input_shape( dim ,)) )

print "prediction for ", x_train[:1]
print model.predict(x_train[:1])
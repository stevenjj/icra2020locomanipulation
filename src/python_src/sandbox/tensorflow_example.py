#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

print(tf.version.VERSION)
print(tf.keras.__version__)

# Load data
mnist = tf.keras.datasets.mnist
(x_train, y_train),(x_test, y_test) = mnist.load_data()
num_train_data = 10000
x_train, y_train = x_train[:num_train_data], y_train[:num_train_data] #only train on 10000 data points

# Normalize the data
x_train, x_test = x_train / 255.0, x_test / 255.0

# Construct Sequential Model
model = tf.keras.Sequential()
model.add(layers.Flatten(input_shape=(28, 28)))
model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
model.add(layers.Dense(10, activation='softmax'))

# Model
model.compile(optimizer=tf.train.AdamOptimizer(0.001),
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Fit
model.fit(x_train, y_train, epochs=5, batch_size=64)
# Evaluate
model.evaluate(x_test, y_test)

# model.add( layers.Dense(64, activation='relu', input_shape( dim ,)) )


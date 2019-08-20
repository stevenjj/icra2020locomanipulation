#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
import numpy as np
import matplotlib.pyplot as plt

print(tf.version.VERSION)
print(tf.keras.__version__)


def plot_history(histories, key='binary_crossentropy'):
  plt.figure(figsize=(16,10))

  for name, history in histories:
    val = plt.plot(history.epoch, history.history['val_'+key],
                   '--', label=name.title()+' Val')
    plt.plot(history.epoch, history.history[key], color=val[0].get_color(),
             label=name.title()+' Train')

  plt.xlabel('Epochs')
  plt.ylabel(key.replace('_',' ').title())
  plt.legend()

  plt.xlim([0,max(history.epoch)])


# Load data
dataset_folder = "/home/sjorgen1/Data/param_set_1/right_hand/transitions_data_with_task_space_info"
total_data = 100#2000
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


# Method 1 of saving the model: Use checkpoint callbacks
# Create checkpoint callback
checkpoint_path = "training_1/cp.ckpt"
checkpoint_dir = os.path.dirname(checkpoint_path)
cp_callback = tf.keras.callbacks.ModelCheckpoint(checkpoint_path,
                                                 save_weights_only=True,
                                                 verbose=1)

def create_model():
	# Construct Sequential Model
	model = tf.keras.Sequential()
	model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01), input_shape=(dataset_dim,)))
	model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
	model.add(layers.Dense(64, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(0.01)))
	model.add(layers.Dense(1, activation='sigmoid'))
	# Model
	model.compile(optimizer=tf.train.AdamOptimizer(0.001),
	              loss='binary_crossentropy',
	              metrics=['accuracy', 'binary_crossentropy'])

	# model.add(layers.Dense(2, activation='softmax'))
	# # Model
	# model.compile(optimizer=tf.train.AdamOptimizer(0.001),
	#               loss='sparse_categorical_crossentropy',
	#               metrics=['accuracy'])

	return model

# Create model and print summary
model = create_model()
print model.summary()

# Fit
model_history = model.fit(x_train, y_train, epochs=50, batch_size=32, validation_data=(x_test, y_test), verbose=2,
              callbacks = [cp_callback]) #pass callback to training)


# Evaluate
print "Evaluating test set:"
model.evaluate(x_test, y_test)


# Reload the trained model
# Method 1 of loading: Load a particular checkpoint
latest = tf.train.latest_checkpoint(checkpoint_dir)
print "Reloading the trained model using the latest checkpoint: ", latest
model2 = create_model()
model2.load_weights(latest)
loss, acc, bin_crossentroy = model2.evaluate(x_test, y_test)
print("Reloaded model, accuracy: {:5.2f}%".format(100*acc))


# Method 2 of saving and loading: Manually save the weights
# Saving and loading weights manually ------------------------------
# model.save_weights('./checkpoints/my_checkpoint')
# manually restore the weights
# model = create_model()
# model.load_weights('./checkpoints/my_checkpoint')


# Method 3 of saving and loading the entire model
# Save entire model to an HDF5 file and loading the entire model -------------------
# model.save('my_model.h5')
# # Load the entire model
# new_model = tf.keras.models.load_model('my_model.h5')
# new_model.compile(optimizer=tf.train.AdamOptimizer(0.001),
#               loss='binary_crossentropy',
#               metrics=['accuracy', 'binary_crossentropy'])
# new_model.summary()
# loss, acc, bin_crossentroy = new_model.evaluate(x_test, y_test)
# print("Restored model, accuracy: {:5.2f}%".format(100*acc))

# Plot model performance
plot_history([('baseline', model_history)])
plt.show()


# Do certain types of predictions
# print model.predict(x_train[:]) # model.predict(x_train[:(idx+1)])

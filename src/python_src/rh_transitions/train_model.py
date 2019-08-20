
#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
import numpy as np
import matplotlib.pyplot as plt

print(tf.version.VERSION)
print(tf.keras.__version__)


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
	              metrics=['accuracy', 'binary_crossentropy'])

	return model


# Plot Model Fit Histories
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
total_data = 20000#2000
dataset = transition_dataset.ContactTransitionDataset(total_data/2)
dataset.load_dataset(dataset_folder, stance_type="right_foot")

# Get the normalize thefilepath + "/" + data
(x_dataset, y_dataset) = dataset.get_normalized_xy_train()
(x_dataset_rand, y_dataset_rand) = dataset.get_normalized_xy_train()
dataset_dim = x_dataset.shape[1] 

# Randomize the data
random_indices = np.random.choice(x_dataset.shape[0], x_dataset.shape[0], replace=False)
for i in range(0, len(random_indices)):
	x_dataset_rand[i] = x_dataset[random_indices[i]]
	y_dataset_rand[i] = y_dataset[random_indices[i]]

# Set the training size
trainset_size = int(total_data*0.8)

# Set the training set and test set
x_train = x_dataset_rand[:trainset_size]
y_train = y_dataset_rand[:trainset_size]

x_test = x_dataset_rand[trainset_size:]
y_test = y_dataset_rand[trainset_size:]

# Create checkpoint callback
checkpoint_path = "training_1/baseline/cp.ckpt"
checkpoint_dir = os.path.dirname(checkpoint_path)
cp_callback = tf.keras.callbacks.ModelCheckpoint(checkpoint_path,
                                                 save_weights_only=True,
                                                 verbose=1)
# Create model and print summary
# best so far
model = create_model(dataset_dim, num_hidden_layers=5, units_per_layer=256, l2_reg=0.001)
model_history = model.fit(x_train, y_train, epochs=75, batch_size=32, validation_data=(x_test, y_test), verbose=2,
              			  callbacks = [cp_callback]) #pass callback to training)

# model = create_model(dataset_dim, num_hidden_layers=4)
# model_history = model.fit(x_train, y_train, epochs=50, batch_size=32, validation_data=(x_test, y_test), verbose=2,
#               			  callbacks = [cp_callback]) #pass callback to training)

# model.save_weights('./learned_model/rh_transitions')
# model.load_weights('./learned_model/rh_transitions')


# Evaluate
print "Evaluating test set:"
model.evaluate(x_test, y_test)


# Plot model performance
plot_history([('baseline', model_history)], key='acc')
plot_history([('baseline', model_history)])
plt.show()

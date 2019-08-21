
#!/usr/bin/env python

import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
import numpy as np
import random
import yaml
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
	              metrics=['accuracy', 'binary_crossentropy', tf.keras.metrics.Precision(), tf.keras.metrics.Recall()])

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

def save_model(save_directory, dataset, model, model_hyper_params):
	x_train_mean = [float(mean) for mean in dataset.x_train_mean] 
	x_train_std = [float(std) for std in dataset.x_train_std] 

	if not os.path.exists(save_directory):
		os.makedirs(save_directory)

	normalization_params = {}
	normalization_params['x_train_mean'] = x_train_mean
	normalization_params['x_train_std'] = x_train_std

	# Save the normalization parameters
	stream = file(save_directory + 'normalization_params.yaml', 'w') 			
	print yaml.dump(normalization_params)
	yaml.safe_dump(normalization_params, stream, allow_unicode=False)
	stream.close()

	# Save the model hyper parameters
	stream = file(save_directory + 'hyper_params.yaml', 'w') 			
	print yaml.dump(model_hyper_params)
	yaml.safe_dump(model_hyper_params, stream, default_flow_style=False)
	stream.close()

	# Save the model
	model.save_weights(save_directory)
	return

# Load data
dataset_folder = "/home/sjorgen1/Data/param_set_1/"

num_positive_data = 10 #10000 # per transition_type
dataset = transition_dataset.ContactTransitionDataset(num_positive_data)

contact_transition_types = [ ("right_hand", "right_foot") ]# [ ("right_hand", "right_foot"), ("right_hand", "left_foot") ]
shorthand = {"right_hand" : "rh", "left_hand" : "lh", "both_hands" : "bh", "right_foot": "rf", "left_foot": "lf"}

save_folder = ""
for type in contact_transition_types:
	subfolder = "/transitions_data_with_task_space_info"
	dataset.load_dataset(dataset_folder + type[0] + subfolder, stance_type=type[1], manipulation_type=type[0])
	save_folder = save_folder + shorthand[type[0]] + "_" + shorthand[type[1]]
# Prepare and normalize the dataset
dataset.prepare_and_normalize_dataset()

# Get train and val sets
x_train, y_train = dataset.get_normalized_xy_train()
x_test, y_test = dataset.get_normalized_xy_val()
dataset_dim = x_train.shape[1]


# Create checkpoint callback
checkpoint_path = "./training_1/" + save_folder + "/baseline/cp.ckpt"
checkpoint_dir = os.path.dirname(checkpoint_path)
cp_callback = tf.keras.callbacks.ModelCheckpoint(checkpoint_path,
                                                 save_weights_only=True,
                                                 verbose=1)
# Create model and print summary
n_hidden_layers = 5
n_units_per_layer = 512 #128
l2_regularization = 0.01

# best so far
model = create_model(dataset_dim, num_hidden_layers=n_hidden_layers, units_per_layer=n_units_per_layer, l2_reg=l2_regularization)
model_history = model.fit(x_train, y_train, epochs=20, batch_size=32, validation_data=(x_test, y_test), verbose=2,
              			  callbacks = [cp_callback]) #pass callback to training)




def create_model_hyper_params_map(n_hidden_layers, n_units_per_layer, l2_regularization):
	return {'n_hidden_layers': n_hidden_layers, 
		    'n_units_per_layer': n_units_per_layer,
		    'l2_regularization': l2_regularization}


# Save model
model_hyper_params = {}
model_hyper_params['baseline'] = create_model_hyper_params_map(n_hidden_layers, n_units_per_layer, l2_regularization)

save_directory = "./learned_model/" + save_folder + "/"
save_model(save_directory, dataset, model, model_hyper_params['baseline'])

# Reload the model
model = create_model(dataset_dim, num_hidden_layers=n_hidden_layers, units_per_layer=n_units_per_layer, l2_reg=l2_regularization)
model.load_weights(save_directory)

# Evaluate
print "Evaluating test set:"
baseline_loss, baseline_acc, baseline_b_ent, baseline_prec, baseline_recall = model.evaluate(x_test, y_test)
baseline_f1 = 2*(baseline_prec*baseline_recall)/(baseline_prec+baseline_recall)


# Plot model performance
models = {}
model_scores = {}
model_histories = []

models['baseline'] = model
model_histories.append(('baseline', model_history))
print "baseline f1 score = ", baseline_f1
best_model = 'baseline'

# plot_history(model_histories, key='acc')
# plot_history(model_histories)
# plt.show()


# Number of models to generate
num_models = 2

for i in range(num_models):
	n_hidden_layers = random.randint(2,10)
	n_units_per_layer = 100*random.randint(1, 10)
	l2_regularization = 10**(-1*random.randint(2,10))

	model_name = "nh_" + str(n_hidden_layers) + "upl_" + str(n_units_per_layer) + "l2r_" + str(l2_regularization)

	print "model #", i+1 , "out of", num_models
	print "attempting", model_name
	model = create_model(dataset_dim, num_hidden_layers=n_hidden_layers, units_per_layer=n_units_per_layer, l2_reg=l2_regularization)
	model_history = model.fit(x_train, y_train, epochs=20, batch_size=32, validation_data=(x_test, y_test), verbose=2) 
	model_histories.append((model_name, model_history))

	loss, acc, b_ent, prec, recall =  model.evaluate(x_test, y_test)
	f1_score = 2*(prec*recall)/(prec+recall)

	print "model:", model_name, "f1 score = ", f1_score

	# store model score
	model_scores[model_name] = f1_score
	# store model
	models[model_name] = model

	# store model hyperparams
	model_hyper_params[model_name] = create_model_hyper_params_map(n_hidden_layers, n_units_per_layer, l2_regularization)



print model_scores
best_model = max(model_scores, key=model_scores.get)
print "best performing model:", best_model , "with f1 score = ", model_scores[best_model]
print "baseline f1 score = ", baseline_f1

# save the model with the best score
save_model(save_directory, dataset, models[best_model], model_hyper_params[best_model])

plot_history(model_histories, key = 'acc')
plot_history(model_histories)
plt.show()



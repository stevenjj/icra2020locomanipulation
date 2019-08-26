#!/usr/bin/env python

from avatar_locomanipulation.srv import *
import rospy
import rospkg

import yaml
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers

class LocomanipulationFeasibilityClassifier:
	def __init__(self):
		self.dataset_dim = 10
		self.hyper_params = {}
		self.n_hidden_layers = 5
		self.n_units_per_layer = 512
		self.l2_regularization = 1e-2
		self.model = self.create_model(self.dataset_dim, num_hidden_layers=self.n_hidden_layers, units_per_layer=self.n_units_per_layer, l2_reg=self.l2_regularization)

	def load_model(self, stored_model_folder_path):
		print "Loading model from:", stored_model_folder_path
		self.hyper_params = self.load_hyper_params(stored_model_folder_path+'hyper_params.yaml')
		self.n_hidden_layers = self.hyper_params['n_hidden_layers']
		self.n_units_per_layer = self.hyper_params['n_units_per_layer']
		self.l2_regularization = self.hyper_params['l2_regularization']

		print "  Loaded hyperparams = ", self.hyper_params
		self.model = self.create_model(self.dataset_dim, num_hidden_layers=self.n_hidden_layers, units_per_layer=self.n_units_per_layer, l2_reg=self.l2_regularization)
		self.model.load_weights(stored_model_folder_path)

	def load_normalization_params(self, yaml_path):
	    with open(yaml_path, 'r') as stream:
	        data_loaded = yaml.load(stream)	
	    return np.array(data_loaded['x_train_mean'], dtype='f'), np.array(data_loaded['x_train_std'], dtype='f')


	def load_hyper_params(self, yaml_path):
	    with open(yaml_path, 'r') as stream:
	        data_loaded = yaml.load(stream)	
	    return {'n_units_per_layer': data_loaded['n_units_per_layer'], 
	    		'n_hidden_layers': data_loaded['n_hidden_layers'],
	    		'l2_regularization': data_loaded['l2_regularization']}

	# Create a simple dense network for binary classification
	def create_model(self, dataset_dim, num_hidden_layers = 3, units_per_layer=64, l2_reg=0.01):
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


	def perform_binary_classification(self, req):
		print "Input Vector:", np.array(req.x)
		res = BinaryClassifierQueryResponse()
		res.y = 0.75
		print "Returning:", res.y
		return res

	def start_classifier_server(self):
	    rospy.init_node('locomanipulation_feasibility_classifier_server')
	    s = rospy.Service('locomanipulation_feasibility_classifier', BinaryClassifierQuery, self.perform_binary_classification)
	    print "Locomanipulation Feasibility Classifier Server Started."
	    rospy.spin()

if __name__ == "__main__":
	classifier = LocomanipulationFeasibilityClassifier()

	# Load the stored model	
	rospack = rospkg.RosPack()
	model_name = 'lh_rflh_lfrh_lfrh_rfbh_rfbh_lf'
	stored_model_folder_path = rospack.get_path('avatar_locomanipulation') + '/src/python_src/rh_transitions/learned_model/' + model_name + '/'
	classifier.load_model(stored_model_folder_path)

	print stored_model_folder_path	

	classifier.start_classifier_server()
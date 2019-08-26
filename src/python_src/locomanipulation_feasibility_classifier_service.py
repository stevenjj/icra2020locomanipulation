#!/usr/bin/env python

from avatar_locomanipulation.srv import *

from std_msgs.msg import String

import rospy
import rospkg


import os
import yaml
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.backend import set_session


print(tf.version.VERSION)
print(tf.keras.__version__)

DEBUG = True
model_path_relative_to_package = '/nn_models/baseline_9000pts/'

class LocomanipulationFeasibilityClassifier:
    def __init__(self):
        self.input_dim = 32

        self.x_train_mean = None
        self.x_train_std = None

        self.model_path = ""

        self.hyper_params = {}
        self.n_hidden_layers = 5
        self.n_units_per_layer = 512
        self.l2_regularization = 1e-2
        self.model = None

        self.x_input = np.array([[0 for i in range(self.input_dim)]], dtype='f')

        self.session = tf.Session(graph=tf.Graph())

    def load_model(self, stored_model_folder_path):
        self.model_path = stored_model_folder_path
        print "Loading model from:", stored_model_folder_path
        self.x_train_mean, self.x_train_std = self.load_normalization_params(stored_model_folder_path+'normalization_params.yaml')

        self.hyper_params = self.load_hyper_params(stored_model_folder_path+'hyper_params.yaml')
        self.n_hidden_layers = self.hyper_params['n_hidden_layers']
        self.n_units_per_layer = self.hyper_params['n_units_per_layer']
        self.l2_regularization = self.hyper_params['l2_regularization']

        print "  Loaded hyperparams = ", self.hyper_params

        # Tensorflow is not multithread safe. So we have to save the sessions
        # See: https://stackoverflow.com/questions/52600230/reusing-tensorflow-session-in-multiple-threads-causes-crash
        #  and https://github.com/tensorflow/tensorflow/issues/28287#issuecomment-495168033
        with self.session.graph.as_default():
            set_session(self.session)
            self.model = self.create_model(self.input_dim, num_hidden_layers=self.n_hidden_layers, units_per_layer=self.n_units_per_layer, l2_reg=self.l2_regularization)
            self.model.load_weights(stored_model_folder_path)

            print self.model.summary()

            # The following lines may not be necessary
            self.model.predict(np.array([range(32)])) # Dummy prediction to finalize the graph
            self.graph = tf.compat.v1.get_default_graph() #tf.get_default_graph()
            self.graph.finalize() # finalize. Makes the graph read only


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
    def create_model(self, input_dim, num_hidden_layers = 3, units_per_layer=64, l2_reg=0.01):
        # Construct Sequential Model
        model = tf.keras.Sequential()

        for i in range(num_hidden_layers):
            if (i == 0):
                model.add(layers.Dense(units_per_layer, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(l2_reg), input_shape=(input_dim,)))
            else:
                model.add(layers.Dense(units_per_layer, activation='relu',  kernel_regularizer=tf.keras.regularizers.l2(l2_reg)))           

        # Add the final layer
        model.add(layers.Dense(1, activation='sigmoid'))

        # Model
        model.compile(optimizer=tf.train.AdamOptimizer(0.001),
                      loss='binary_crossentropy',
                      metrics=['accuracy', 'binary_crossentropy', tf.keras.metrics.Precision(), tf.keras.metrics.Recall()])
        return model

    def sample_classify(self):
        sample_x = np.array([[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]], dtype='f')
        print self.model.predict(sample_x)

    def perform_binary_classification(self, req):
        res = BinaryClassifierQueryResponse()
        if (len(req.x) != self.input_dim):
            rospy.logerr("Error. Input Dimension is %d but it must be %d", len(req.x), self.input_dim)
            res.y = -1.0
        else:
            # Apply Z-Normalization to the input 
            for i in range(len(req.x)):
                self.x_input[0][i] = (req.x[i] - self.x_train_mean[i]) / self.x_train_std[i]

            if DEBUG:
                print "Unnormalized Input Vector:", req.x
                print "Normalized Input Vector:", self.x_input
                print "  Input vector has shape:", self.x_input.shape
            with self.session.graph.as_default():
                set_session(self.session)
                res.y = self.model.predict(self.x_input)[0][0]
                if DEBUG:
                    print "prediction result = ", res.y
    
        if DEBUG:          
            print "Returning:", res.y
        return res

    def start_classifier_server(self):
        rospy.init_node('locomanipulation_feasibility_classifier_server')
        s = rospy.Service('locomanipulation_feasibility_classifier', BinaryClassifierQuery, self.perform_binary_classification)
        print "Locomanipulation Feasibility Classifier Server Started."
        rospy.spin()

if __name__ == "__main__":
    classifier = LocomanipulationFeasibilityClassifier()
 
    # # Load the stored model   
    rospack = rospkg.RosPack()
    model_name = 'lh_rflh_lfrh_lfrh_rfbh_rfbh_lf'
    stored_model_folder_path = rospack.get_path('avatar_locomanipulation') + model_path_relative_to_package + model_name + '/'
    print stored_model_folder_path  

    classifier.load_model(stored_model_folder_path)
    classifier.start_classifier_server()


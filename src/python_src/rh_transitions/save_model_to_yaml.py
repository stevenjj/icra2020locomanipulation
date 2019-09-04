import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
from ruamel.yaml import YAML
import numpy as np
import matplotlib.pyplot as plt

import load_model

from tensorflow.keras.backend import set_session


contact_transition_types = [ ("left_hand", "right_foot"), ("left_hand", "left_foot"), ("right_hand", "left_foot"), ("right_hand", "right_foot"), ("both_hands", "right_foot"), ("both_hands", "left_foot") ]# [ ("right_hand", "right_foot"), ("right_hand", "left_foot") ]
shorthand = {"right_hand" : "rh", "left_hand" : "lh", "both_hands" : "bh", "right_foot": "rf", "left_foot": "lf"}

# Identify the load path
load_folder = ""
for type in contact_transition_types:
	load_folder = load_folder + shorthand[type[0]] + "_" + shorthand[type[1]]
load_path = "./learned_model/" + load_folder + "/"

# Load and create the model
hyper_params = load_model.load_hyper_params(load_path+'hyper_params.yaml')
n_hidden_layers = hyper_params['n_hidden_layers']
n_units_per_layer = hyper_params['n_units_per_layer']
l2_regularization = hyper_params['l2_regularization']

#print ("hyperparams = ", hyper_params)

global model, graph

input_dim = 32
session = tf.Session(graph=tf.Graph())

with session.graph.as_default():
    set_session(session)
    model = load_model.create_model(input_dim, num_hidden_layers=n_hidden_layers, units_per_layer=n_units_per_layer, l2_reg=l2_regularization)
    model.load_weights(load_path)

    print model.summary()

    # The following lines may not be necessary
    model.predict(np.array([range(32)])) # Dummy prediction to finalize the graph
    graph = tf.compat.v1.get_default_graph() #tf.get_default_graph()
    graph.finalize() # finalize. Makes the graph read only

# init = tf.global_variables_initializer()
# sess = tf.Session()
# sess.run(init)

w=[]
b=[]
data = {}

save_path = "./learned_model/model.yaml"

with session.graph.as_default():
    set_session(session)
    for i in tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope = 'dense'):
        scope_name = i.name
        layer = scope_name.replace('dense_', '')
        layer = layer.replace('dense', '0')
        layer = layer.replace('/bias:0', '')

        layer = layer.replace('/kernel:0', '')
        int_layer = int(layer, 10)
        data_type = scope_name.replace('dense_'+layer+'/','')
        data_type = data_type.replace('dense/','')
        data_type = data_type.replace(':0', '')
        if data_type=='kernel':
            # w.append( session.run(tf.trainable_variables(scope_name)) )
            w.append(i.eval(session=session))
        elif data_type=='bias':
            # b.append( session.run(tf.trainable_variables(scope_name)) )
            # print(i.eval(session=sess))
            b.append(i.eval(session=session))



num_layers = int_layer+1
data['num_layer'] = num_layers

print('weight 5 = ')
print(w[5])
print('bias 5 = ')
print(b[5])

for layer_idx in range(num_layers):
    data['w'+str(layer_idx)] = w[layer_idx].tolist()
    data['b'+str(layer_idx)] = b[layer_idx].reshape(1, b[layer_idx].shape[0]).tolist()
    if (layer_idx == num_layers-1):
        data['act_fn'+str(layer_idx)] = 4
    else:
        data['act_fn'+str(layer_idx)] = 2

with open(save_path, 'w') as f:
   yaml = YAML()
   yaml.dump(data, f)

print('Saving Yaml to', save_path)

# print(tf.trainable_variables())
    #print(i.eval(session=sess))

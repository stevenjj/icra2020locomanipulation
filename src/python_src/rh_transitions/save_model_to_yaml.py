import tensorflow as tf
from tensorflow.keras import layers

import contact_transition_dataset as transition_dataset

import os
from ruamel.yaml import YAML
import numpy as np
import matplotlib.pyplot as plt

import load_model


init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)

w=[]
b=[]
data = {}

save_path = "./learned_model/model.yaml"

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
        w.append(i.eval(session=sess))
    elif data_type=='bias':
        b.append(i.eval(session=sess))

num_layers = int_layer+1
data['num_layer'] = int_layer

print(w[5].shape)

for layer_idx in range(num_layers):
    data['w'+str(layer_idx)] = w[layer_idx].tolist()
    data['b'+str(layer_idx)] = b[layer_idx].reshape(1, b[layer_idx].shape[0]).tolist()
    if (layer_idx == num_layers-1):
        data['act_fn'+str(layer_idx)] = 3
    else:
        data['act_fn'+str(layer_idx)] = 2

with open(save_path, 'w') as f:
    yaml = YAML()
    yaml.dump(data, f)

print('Saving Yaml to', save_path)
    #print(i.eval(session=sess))

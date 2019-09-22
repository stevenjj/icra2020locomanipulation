
#!/usr/bin/env python
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

yaml_path = "v2_diff_cart_traj.yaml"
with open(yaml_path, 'r') as stream:
    data_loaded = yaml.load(stream)	

time = np.array(data_loaded['t'])
z_l = np.array(data_loaded['left_foot_pos_z'])
z_r = np.array(data_loaded['right_foot_pos_z'])

t_max = max(time)
z_max = max(z_l)

fig, axs = plt.subplots(2)
axs[0].grid(False)
axs[0].plot(time, z_r)
axs[0].set(xlim=(0.0, t_max), ylim=(0.0, z_max))
axs[1].grid(False)
axs[1].plot(time, z_l)
axs[1].set(xlabel=r'$t$', xlim=(0.0, t_max), ylim=(0.0, z_max))

plt.show()
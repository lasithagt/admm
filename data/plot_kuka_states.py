

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import bagpy
from bagpy import bagreader
import pandas as pd
from scipy import signal

if __name__ == "__main__":

	# read the bag file
	b = bagreader('test_velocity.bag')

	print(b.topic_table)
	velocity = b.message_by_topic('/kuka/state/KUKAJointVelocity')
	data_velocity = pd.read_csv(velocity)

	# fig, ax = bagpy.create_fig(1)
	# ax[0].scatter(x='Time', y='velocity.quantity', data=vel)

	print(data_velocity['velocity.quantity'].values[0][0])

	# plot the computed velocity vs real from kuka






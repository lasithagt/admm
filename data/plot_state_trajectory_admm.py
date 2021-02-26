
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
	data_actual = np.array(np.load('./state_trajectory_admm_actual.npy'))
	data_actual = data_actual.squeeze()
	data_actual = data_actual.T


	data_desired = np.array(np.load('./state_trajectory_admm_desired.npy'))
	data_desired = data_desired.squeeze()
	data_desired = data_desired.T


	fig1 = plt.figure(1)
	ax = plt.axes(projection='3d')


	# Plot data for motion
	xline = data_actual[0,::]
	yline = data_actual[1,::]
	zline = data_actual[2,::]
	ax.plot3D(xline, yline, zline)

	ax.legend()
	# Plot data for motion
	xline = data_desired[0,::]
	yline = data_desired[1,::]
	zline = data_desired[2,::]
	ax.plot3D(xline, yline, zline)


	plt.legend()


	plt.show()
	# print(data.shape) 
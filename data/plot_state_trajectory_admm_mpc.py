

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
	# b = np.load('./state_trajectory_admm_mpc.npy')
	data_des = np.load('./state_trajectory_admm_mpc_desired.npy')
	data_mpc = np.load('./state_trajectory_admm_mpc.npy')

	
	print(data_mpc.shape)

	data_des = data_des.squeeze()
	data_des = data_des.T

	data_mpc = data_mpc.squeeze()
	data_mpc = data_mpc.T


	# line_c = ['b', 'g', 'r', 'c', 'k', 'm', 'y']
	# x_ = np.arange(0, b.shape[1], 1)

	# fig, axs = plt.subplots(2, 1)
	fig1 = plt.figure(1)
	ax = plt.axes(projection='3d')

	xline = data_des[0,::]
	yline = data_des[1,::]
	zline = data_des[2,::]

	ax.plot3D(xline, yline, zline, label='Desired')
	ax.scatter3D(xline[0], yline[0], zline[0], 'r+')

	print(data_mpc)
	# fig1 = plt.figure(2)
	# ax = plt.axes(projection='3d')

	xline = data_mpc[0,::]
	yline = data_mpc[1,::]
	zline = data_mpc[2,::]

	ax.plot3D(xline, yline, zline, label='Actual')
	ax.scatter3D(xline[0], yline[0], zline[0], 'b+')

	plt.legend()

	plt.show()


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
	# b = np.load('./state_trajectory_admm_mpc.npy')
	data_des = np.load('./state_trajectory_admm_mpc_desired.npy')
	data_mpc = np.load('./state_trajectory_admm_mpc.npy')
	data_mpc_state = np.load('./state_trajectory_admm_state_mpc.npy')

	data_cartesian_desired =  np.load('./state_trajectory_cartesian_desired.npy')
	data_actual_state =  np.load('./state_trajectory_actual_state.npy')

	
	# print(data_mpc.shape)

	data_des = data_des.squeeze()
	data_des = data_des.T

	data_mpc = data_mpc.squeeze()
	data_mpc = data_mpc.T

	data_mpc_state = data_mpc_state.squeeze()
	data_mpc_state = data_mpc_state.T

	data_cartesian_desired = data_cartesian_desired.squeeze()
	data_cartesian_desired = data_cartesian_desired.T

	data_actual_state = data_actual_state.squeeze()
	data_actual_state = data_actual_state.T

	print(data_actual_state[:,0:10])



	# line_c = ['b', 'g', 'r', 'c', 'k', 'm', 'y']
	# x_ = np.arange(0, b.shape[1], 1)

	# fig, axs = plt.subplots(2, 1)
	fig1 = plt.figure(1)
	ax = plt.axes(projection='3d')

	xline = data_des[0,::]
	yline = data_des[1,::]
	zline = data_des[2,::]

	ax.plot3D(xline, yline, zline, 'k-', label='Desired')
	ax.scatter3D(xline[0], yline[0], zline[0], 'o')

	# print(data_mpc)
	# fig1 = plt.figure(2)
	# ax = plt.axes(projection='3d')

	xline = data_mpc[0,::]
	yline = data_mpc[1,::]
	zline = data_mpc[2,::]

	# ax.plot3D(xline, yline, zline, label='Actual')

	# ax.scatter3D(xline, yline, zline, marker='o', c='g')


	xline = data_mpc_state[0,::]
	yline = data_mpc_state[1,::]
	zline = data_mpc_state[2,::]

	ax.plot3D(xline, yline, zline, c='b', label='commanded_state')
	ax.scatter3D(xline, yline, zline, c='b')

	xline = data_actual_state[0,::]
	yline = data_actual_state[1,::]
	zline = data_actual_state[2,::]

	ax.scatter3D(xline, yline, zline, marker='o', c='k', s=50, label='actual_position')


	xline = data_cartesian_desired[0,::]
	yline = data_cartesian_desired[1,::]
	zline = data_cartesian_desired[2,::]

	# ax.scatter3D(xline, yline, zline, c='r', marker='o', label='desired_position')
	# ax.plot3D(xline, yline, zline, c='r')

	plt.legend()
	ax.set_xlim([-0.1, 0.1])
	ax.set_ylim([-0.05, 0.05])

	plt.show()

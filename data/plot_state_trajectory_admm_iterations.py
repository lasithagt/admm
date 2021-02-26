
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
	data = np.array(np.load('./cartesian_trajectory_admm.npy'))

	data = data.squeeze()
	data = data.T


	fig1 = plt.figure(1)
	ax = plt.axes(projection='3d')
	# ax.set_aspect('equal')


	# Plot data for motion
	# for i in range(6):
	xline = data[0,::]
	yline = data[1,::]
	zline = data[2,::]
	# line_name = 'Iteration' + str(5)
	ax.plot3D(xline, yline, zline)

	ax.legend()

	fig2 = plt.figure(2)


	# Plot data for force
	x_time = np.arange(0, data.shape[2], 1)

	xline = data[4,3,::]
	yline = data[4,4,::]
	zline = data[4,5,::]
	plt.plot(x_time, xline, label='Force X')
	plt.plot(x_time, yline, label='Force Y')
	plt.plot(x_time, zline, label='Force Z')

	plt.legend()


	plt.show()
	# print(data.shape) 
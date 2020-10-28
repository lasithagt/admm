
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
	data = np.array(np.load('./cartesian_trajectory_admm.npy'))

	fig1 = plt.figure(1)
	ax = plt.axes(projection='3d')

	# Plot data for motion
	for i in range(6):
		xline = data[i,0,::]
		yline = data[i,1,::]
		zline = data[i,2,::]
		line_name = 'Iteration' + str(i)
		ax.plot3D(xline, yline, zline, label = line_name)

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
	print(data.shape) 
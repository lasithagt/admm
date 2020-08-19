
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
	b = np.load('../build/standalone⁩arr1.npy')
	b = b.squeeze()
	b = b.T

	# x_ = np.arange(0, b.shape[1], 1)

	# fig, axs = plt.subplots(2, 1)

	# for i in range(7):
	# 	axs[0].plt.plot(x_, b[i, ::])

# axs[0].plot(t, s1, t, s2)
# axs[0].set_xlim(0, 2)
# axs[0].set_xlabel('time')
# axs[0].set_ylabel('s1 and s2')
# axs[0].grid(True)

	# cxy, f = axs[1].cohere(s1, s2, 256, 1. / dt)
	# axs[1].set_ylabel('coherence')

	# fig.tight_layout()

	# plt.show()
	print(b) 
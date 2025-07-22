import numpy as np
import matplotlib.pyplot as plt

# Load data
A = np.loadtxt("COMpos.txt")

# Sizes
sA = A.shape

#plotting
plt.figure()

# X
plt.subplot(6,1,1)
plt.plot(A[:, 0], label="x position")
plt.legend()

# Y
plt.subplot(6,1,2)
plt.plot(A[:, 1], label="y position")
plt.legend()

# Z
plt.subplot(6,1,3)
plt.plot(A[:, 2], label="z position")
plt.legend()

# Row
plt.subplot(6,1,4)
plt.plot(A[:, 3], label="Row")
plt.legend()

# Pitch
plt.subplot(6,1,5)
plt.plot(A[:, 4], label="Pitch")
plt.legend()

# Yaw
plt.subplot(6,1,6)
plt.plot(A[:, 5], label="Yaw")
plt.legend()

# Show plot
plt.show()
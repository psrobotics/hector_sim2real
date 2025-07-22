import numpy as np
import matplotlib.pyplot as plt

# Load data
A = np.loadtxt("Temp.txt")

# Plotting
plt.figure()

# First subplot
plt.subplot(2, 1, 1)
plt.plot(A[:, 0:5])
plt.title("Left Leg")

# Second subplot
plt.subplot(2, 1, 2)
plt.plot(A[:, 5:10])
plt.title("Right Leg")

# Show plot
plt.show()

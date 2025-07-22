import numpy as np
import matplotlib.pyplot as plt

# Load data
A = np.loadtxt("Motor_Error.txt")

# Plotting
plt.figure()

# First subplot
plt.subplot(2, 1, 1)
plt.plot(A[:, 0:5])
plt.title("Columns 1 to 5")

# Second subplot
plt.subplot(2, 1, 2)
plt.plot(A[:, 5:10])
plt.title("Columns 6 to 10")

# Show plot
plt.show()

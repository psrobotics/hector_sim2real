import numpy as np
import matplotlib.pyplot as plt

# Constants
timestep = 30000

# Load data
A = np.loadtxt("raw_angle.txt")
B = np.loadtxt("Desired_Position.txt")
error = np.loadtxt("Motor_Error.txt")
torque_est = np.loadtxt("torque.txt")
torque_command = np.loadtxt("Tau_in_leg.txt")

# Sizes
sA = A.shape
sB = B.shape
serror = error.shape
storque = torque_est.shape
stau_command = torque_command.shape

# Plotting
plt.figure()

# First subplot
plt.subplot(2, 1, 1)
plt.plot(A[-timestep:, 3], label="q feedback")
plt.plot(B[-timestep:, 3], label="q command")
plt.plot(error[-timestep:, 3], label="error")
plt.plot(torque_est[-timestep:, 3], label="estimate torque")
plt.plot(torque_command[-timestep:, 3], label="command torque")
plt.legend()

# Second subplot
plt.subplot(2, 1, 2)
plt.plot(A[-timestep:, 8], label="q feedback")
plt.plot(B[-timestep:, 8], label="q command")
plt.plot(error[-timestep:, 8], label="error")
plt.plot(torque_est[-timestep:, 8], label="estimate torque")
plt.plot(torque_command[-timestep:, 8], label="command torque")
plt.legend()


# Show plot
plt.show()

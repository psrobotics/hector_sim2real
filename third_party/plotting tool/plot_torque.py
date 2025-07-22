import numpy as np
import matplotlib.pyplot as plt

# Load data
A = np.loadtxt("Feedback_torque.txt")
A[:, 3] = A[:, 3] / 1.545
A[:, 8] = A[:, 8] / 1.545
A = A[-4000:, :]

sA = A.shape
tA = np.arange(1, sA[0] + 1)

# Plotting
plt.figure()

# First subplot
plt.subplot(2, 2, 1)
plt.plot(A[:, 0:5])
plt.title("Feedback Torque (Columns 1 to 5)")

# Second subplot
plt.subplot(2, 2, 2)
plt.plot(A[:, 5:10])
plt.title("Feedback Torque (Columns 6 to 10)")

B = np.loadtxt("Tau_in_leg.txt")
B = B[-4000:, :]
sB = B.shape
tB = np.arange(1, sB[0] + 1)

# Third subplot
plt.subplot(2, 2, 3)
plt.plot(B[:, 0:5])
plt.title("Tau in Leg (Columns 1 to 5)")

# Fourth subplot
plt.subplot(2, 2, 4)
plt.plot(B[:, 5:10])
plt.title("Tau in Leg (Columns 6 to 10)")

plt.show()

# Plot Left Knee
plt.figure()
plt.plot(tA, A[:, 3], label="Feedback")
plt.plot(tB, B[:, 3], label="Command")
plt.title("Left Knee")
plt.legend()
plt.show()

# Plot Right Knee
plt.figure()
plt.plot(tA, A[:, 8], label="Feedback")
plt.plot(tB, B[:, 8], label="Command")
plt.title("Right Knee")
plt.legend()
plt.show()

# Plot Feedback Torques
plt.figure()
plt.plot(tA, A[:, 3], label="Left Knee")
plt.plot(tA, A[:, 8], label="Right Knee")
plt.title("Feedback Torques")
plt.legend()
plt.show()

# Plot Left Thigh
plt.figure()
plt.plot(tA, A[:, 2], label="Feedback")
plt.plot(tB, B[:, 2], label="Command")
plt.title("Left Thigh")
plt.legend()
plt.show()

# Plot Right Thigh
plt.figure()
plt.plot(tA, A[:, 7], label="Feedback")
plt.plot(tB, B[:, 7], label="Command")
plt.title("Right Thigh")
plt.legend()
plt.show()

# Plot Left Hip1
plt.figure()
plt.plot(tA, A[:, 0], label="Feedback")
plt.plot(tB, B[:, 0], label="Command")
plt.title("Left Hip1")
plt.legend()
plt.show()

# Plot Right Hip1
plt.figure()
plt.plot(tA, A[:, 5], label="Feedback")
plt.plot(tB, B[:, 5], label="Command")
plt.title("Right Hip1")
plt.legend()
plt.show()

# Plot Left Hip2
plt.figure()
plt.plot(tA, A[:, 1], label="Feedback")
plt.plot(tB, B[:, 1], label="Command")
plt.title("Left Hip2")
plt.legend()
plt.show()

# Plot Right Hip2
plt.figure()
plt.plot(tA, A[:, 6], label="Feedback")
plt.plot(tB, B[:, 6], label="Command")
plt.title("Right Hip2")
plt.legend()
plt.show()

# Plot Left Ankle
plt.figure()
plt.plot(tA, A[:, 4], label="Feedback")
plt.plot(tB, B[:, 4], label="Command")
plt.title("Left Ankle")
plt.legend()
plt.show()

# Plot Right Ankle
plt.figure()
plt.plot(tA, A[:, 9], label="Feedback")
plt.plot(tB, B[:, 9], label="Command")
plt.title("Right Ankle")
plt.legend()
plt.show()

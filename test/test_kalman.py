import unittest 
import numpy as np
import matplotlib.pyplot as plt

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)


# Define system matrices
A = np.array([[1, 1], [0, 1]])  # State transition matrix
H = np.array([[1, 0]])          # Observation matrix
Q = np.array([[0.1, 0], [0, 0.1]])  # Process noise covariance
R = np.array([[1]])             # Observation noise covariance
P = np.array([[1, 0], [0, 1]])  # Initial error covariance
x0 = np.array([0, 1])           # Initial state estimate

# Define the true initial state
true_x0 = np.array([0, 1])

# Create an instance of the Kalman filter
kf = Kalman(A, H, Q, R, P, x0)

# Simulate the system for 50 time steps
num_steps = 50
true_states = []
observations = []

# Initial true state
true_state = true_x0

for _ in range(num_steps):
    # True state update (without process noise for simplicity)
    true_state = np.dot(A, true_state)
    true_states.append(true_state)

    # Observation with noise
    observation = np.dot(H, true_state) + np.random.normal(0, 1, H.shape[0])
    observations.append(observation)

true_states = np.array(true_states)
observations = np.array(observations)

# Use the Kalman filter to estimate the states
estimated_states = kf.filter(observations)

# Evaluate the performance of the Kalman filter
performance = kf.evaluate(true_states, estimated_states)
print(f"Mean Squared Error: {performance['MSE']}")

# Plot the true states and the estimated states
plt.plot(true_states[:, 0], label='True Position')
plt.plot(estimated_states[:, 0], label='Estimated Position')
plt.plot(observations, label='Observations', linestyle='dotted')
plt.legend()
plt.xlabel('Time step')
plt.ylabel('Position')
plt.title('Kalman Filter State Estimation')
plt.show()

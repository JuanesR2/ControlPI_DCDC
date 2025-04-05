import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Define system matrices and parameters with float type
A = np.array([[0.0, 0.0],
              [0.0, -0.3]])

B = np.array([[0.0, -1.0],
              [1.0, 0.0]])

d = np.array([1.0, 0.0])

u_ref = 0.6

# Compute x_ref
x_ref = -np.linalg.solve(A + u_ref * B, d)

kp = 1
ki = 10

# Define y_d(x) function
def y_d(x):
    return np.dot((x - x_ref).T, (kp * A + ki * B) @ x_ref)

# Define u_control(x_e) function
def u_control(x_e):
    return -kp * y_d(x_e[:2]) - ki * x_e[2]

# Define derivative function for the differential equation
def convertidor(t, u):
    x_e = u
    du1_2 = A @ x_e[:2] + u_control(x_e) * B @ x_e[:2] + d
    du3 = y_d(x_e[:2])
    return np.concatenate([du1_2, [du3]])

# Initial conditions for differential equation
u0 = np.array([0.0, 0.0, 0.0])  # Initial state [x1(0); x2(0); x3(0)]

# Time span for simulation
tspan = (0.0, 20.0)

# Solve the ODE problem
sol = solve_ivp(convertidor, tspan, u0, dense_output=True)

# Extract the solution
tode = sol.t  # Time points
xode = sol.y.T  # State variables [x1, x2, x3]

# Plot the results using matplotlib
plt.plot(tode, xode)
plt.xlabel('Time')
plt.ylabel('State Variables')
plt.legend(['x1', 'x2', 'u_control'])
plt.show()

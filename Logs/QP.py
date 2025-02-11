import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# Define waypoints (simulating an RRT* rough path)
waypoints = np.array([[0, 0], [1, 2], [3, 4], [6, 5], [10, 5]])
n = len(waypoints) - 1  # Number of segments

# Time intervals between waypoints (assuming equal time steps)
T = np.linspace(0, 1, n + 1)  

# Define decision variables (position at each time step)
X = cp.Variable((n + 1, 2))  # 2D positions (x, y) at each waypoint

# Define quadratic cost function: Minimize snap (4th derivative)
cost = 0
for i in range(2, n - 1):  # Minimize snap (approximated using finite differences)
    snap_x = X[i-2, 0] - 4*X[i-1, 0] + 6*X[i, 0] - 4*X[i+1, 0] + X[i+2, 0]
    snap_y = X[i-2, 1] - 4*X[i-1, 1] + 6*X[i, 1] - 4*X[i+1, 1] + X[i+2, 1]
    cost += cp.square(snap_x) + cp.square(snap_y)

# Constraints
constraints = [X[0] == waypoints[0], X[-1] == waypoints[-1]]  # Start & end fixed
for i in range(1, n):  
    constraints.append(cp.norm(X[i] - waypoints[i], 2) <= 0.5)  # Stay near waypoints

# Solve QP problem
problem = cp.Problem(cp.Minimize(cost), constraints)
problem.solve()

# Extract optimized trajectory
optimized_traj = X.value

# Plot results
plt.figure(figsize=(8, 5))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro--', label="Original (RRT* output)")
plt.plot(optimized_traj[:, 0], optimized_traj[:, 1], 'bo-', label="Optimized (QP smooth)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Trajectory Optimization using QP (Min Snap)")
plt.grid()
plt.show()

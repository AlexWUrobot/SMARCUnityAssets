import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# Define waypoints (simulated RRT* rough path)
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
    constraints.append(cp.norm(X[i] - waypoints[i], 2) <= 1.0)  # Loosened from 0.5 to 1.0

# Define obstacle constraints
obstacles = [
    {"point": np.array([4, 3]), "normal": np.array([-1, 0])},  # Example obstacle (wall)
    {"point": np.array([7, 4]), "normal": np.array([-1, -1])}  # Another obstacle
]

critical_waypoints = [1, 3]  # Apply avoidance only at selected points

for i in critical_waypoints:
    for obs in obstacles:
        s_f = obs["point"]
        n_f = obs["normal"]
        constraints.append((X[i] - s_f) @ n_f >= -0.5)  # Loosen the constraint slightly

# Solve QP problem
problem = cp.Problem(cp.Minimize(cost), constraints)
problem.solve(solver=cp.SCS)

# Check if solution is found
if X.value is None:
    print("QP failed: No feasible solution found. Relax constraints or check setup.")
    X.value = waypoints  # Fall back to original waypoints

# Extract optimized trajectory
optimized_traj = X.value

# Plot results
plt.figure(figsize=(8, 5))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro--', label="Original (RRT* output)")
plt.plot(optimized_traj[:, 0], optimized_traj[:, 1], 'bo-', label="Optimized (QP smooth)")

# Plot obstacles
for obs in obstacles:
    plt.scatter(*obs["point"], marker='x', color='red', s=100, label="Obstacle")

plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Trajectory Optimization using QP with Obstacle Avoidance")
plt.grid()
plt.show()

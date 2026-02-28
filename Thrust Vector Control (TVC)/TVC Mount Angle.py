# This script determines the relationship between
# the thrust-vectoring motor angle (rocket mount angle)
# and the corresponding servo angle used by
# the rocket's control system.
import numpy as np # Library imports
from scipy.optimize import root
import matplotlib.pyplot as plt

# Mechanism and mount geometry
d = 0.0278 # Horizontal offset between rocket pivot and servo pivot
e = 0.0000 # vertical offset (approximately zero)
R2 = 0.0130 # servo arm to pivot
R1 = 0.0515 # rocket arm to pivot
L  = 0.0270 # rod length

# Geometrical constraint
def constraint(theta2, theta1):
    return (
        (d + R2*np.cos(theta2 - np.pi/2) - R1*np.cos(theta1))**2 +
        (e + R2*np.sin(theta2 - np.pi/2) - R1*np.sin(theta1))**2 -
        L**2
    )

# Solve for theta 2 (servo angle) given a theta 1 (motor angle)
def solve_theta(theta1, theta_guess=0):
    sol = root(lambda th: constraint(th, theta1),
           theta_guess,
           method='hybr',
           tol=1e-10)
    if not sol.success:
        print("Solver failed at theta1 =", theta1)
    return sol.x[0]

# Sweep mount angle
theta1 = np.linspace(-15, 15, 300) * np.pi / 180 # radians
theta2 = np.zeros_like(theta1)

theta_guess = 0 # Initial guess around neutral position

# Solve values for theta 2
for i in range(len(theta1)):
    theta2[i] = solve_theta(theta1[i], theta_guess)
    theta_guess = theta2[i]  # track branch

# R squared function
def r_squared(y, y_fit):
    ss_res = np.sum((y - y_fit)**2)
    ss_tot = np.sum((y - np.mean(y))**2)
    r = 1 - ss_res/ss_tot
    return r

# Curve fitting
x = np.array(theta1)
y = np.array(theta2)

# Line around origin where small angle approximations are valid
limit = 3 * np.pi / 180 # Limit in radians

# Create mask around -5 to 5 degrees
mask = np.abs(x) <= limit

# Apply mask
x_small = x[mask]
y_small = y[mask]

# Fit
m, C = np.polyfit(x_small, y_small, 1)

# Produce line
theta2_line = m*theta1 + C

# Quadratic fit
a, b, c = np.polyfit(x, y, 2)
quad = a*x**2 + b*x + c

# Calculate R squared for each fit
r_line = r_squared(y, theta2_line)
r_quad = r_squared(y, quad)

# Plot
plt.figure(figsize=(12,12))
plt.plot(theta1 * 180 / np.pi, theta2 * 180 / np.pi, color='red', label='Analytical Solution')
plt.plot(theta1 * 180 / np.pi, theta2_line * 180 / np.pi, '--', color = 'green', label=f'Line Fit, y = {m:.4f}x + {c:.4f}: R² = {r_line:.4f}')
plt.plot(theta1 * 180 / np.pi, quad * 180 / np.pi, '--', color = 'purple', label=f'Quadratic Fit, y = {a:.4f}x² + {b:.4f}x + {c:.4f}: R² = {r_quad:.4f}')
plt.xlabel(r'$\theta_1 \, \mathrm{(motor\ angle, deg)}$')
plt.ylabel(r'$\theta_2 \, \mathrm{(servo\ angle, deg)}$')
plt.grid()
plt.legend()
plt.show()

# Print statements
print("Linear fit:")
print(f"  y = {m:.4f}x + {c:.4f}")
print(f"  R² = {r_line:.4f}\n")
print("Quadratic fit:")
print(f"  y = {a:.4f}x² + {b:.4f}x + {c:.4f}")
print(f"  R² = {r_quad:.4f}")
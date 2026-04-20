import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-v0_8-whitegrid') 

# Simulation domain and system properties
start = 0
end = 3
# dt = 0.001 # Appriximately continuous time
dt = 0.071
I = 0.229
time_frame = np.arange(start, end + dt, dt)

# Define dictionaries containing ascent and descent case dynamics
ascent_case = {
    "thrust": 74, # Average motor thrust 
    "moment_arm": 0.363, # Moment arm of TVC mount, thrust action point to centre of mass distance
    "factor": 4, # Scale domain to test different ranges
    "burn_time": 1.2, # Motor burn time
    "name": "Ascent" # Name
}

descent_case = {
    "thrust": 20,
    "moment_arm": 0.28,
    "factor": 17,
    "burn_time": 2.5,
    "name": "Descent"
}

# PD controller
class PDController():
    def __init__(self, Kp, Kd): # Constructor initialises gains
        self.Kp = Kp
        self.Kd = Kd

    def compute(self, yaw, yaw_rate, yaw_ref=0):
        error = yaw_ref - yaw # Error
        return self.Kp * error - self.Kd * yaw_rate # Return PD output

# Simulate discrete time response to step input
def sim(controller, yaw, yaw_rate, time_frame, thrust, moment_arm):
    yaw_history = []
    tvc_history = []
    yaw = np.radians(yaw)
    yaw_rate = np.radians(yaw_rate)

    for _ in time_frame: # Walk forward through time steps and compute dynamics
        yaw_history.append(yaw)
        tvc = controller.compute(yaw, yaw_rate)
        tvc = np.clip(tvc, -15*np.pi/180, 15*np.pi/180) # Physical mount limit
        tvc_history.append(tvc)

        torque = thrust * moment_arm * np.sin(tvc)

        alpha = torque / I

        yaw_rate = yaw_rate + alpha * dt
        yaw = yaw + yaw_rate * dt 
    return yaw_history, tvc_history

# Continuous time sweep
# Kp_set = np.arange(0.1, 1.0, 0.1)
# Kp_set_fixed = 0.9
# Kd_set = np.arange(0.01, 0.1, 0.01)
# Kd_set_fixed = 0.09

def samples(factor):
    # Sweep through various combinations of gains
    Kp_set = np.arange(0.1, 0.25, 0.009375) * factor
    Kp_set_fixed = 0.25 * factor
    Kd_set = np.arange(0.01, 0.03, 0.0015) * factor
    Kd_set_fixed = 0.03 * factor

    values = []
    for Kd in Kd_set:
        values.append((Kp_set_fixed, Kd))
    for Kp in Kp_set:
        values.append((Kp, Kd_set_fixed))
    return values

# Plot for optimal values
fig, ax = plt.subplots(1, 2, figsize=(14, 8)) 
fig.suptitle("Close-Loop System Response to a 10° Correction for both Ascent and Descent Cases", fontsize=14, fontweight="bold")
cmap = plt.cm.turbo

cases = [ascent_case, descent_case]

for j, case in enumerate(cases):
    values = samples(case["factor"])
    iterations = len(values)
    norm = plt.Normalize(0, iterations - 1)

    for i, (Kp, Kd) in enumerate(values):
        controller = PDController(Kp=Kp, Kd=Kd)

        yaw_history, tvc_history = sim(controller,-10,0,time_frame,case["thrust"],case["moment_arm"]
        )

        color = cmap(norm(i))
        ax[j].plot(time_frame, np.degrees(yaw_history), color=color)

    # ax[j].set_title(f"{case["name"]} Case", fontsize=14)
    ax[j].set_xlabel("Time (s)", fontsize=12, fontweight="bold")
    ax[j].set_ylabel(f"{case["name"]} Rocket Angle (deg)", fontsize=12, fontweight="bold")
    ax[j].legend(["Optimal Gains", f"Kp={values[-1][0]:.2f}", f"Kd={values[-1][1]:.2f}"], fontsize=12, handlelength=0, loc="upper right")
    ax[j].grid(alpha=0.7)

# Colorbar (use last norm)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
cbar = fig.colorbar(sm, ax=ax[1])
cbar.set_label("Iteration Number", fontsize=12, fontweight="bold")

plt.tight_layout()
plt.show()

# controller = PDController(Kp=0.9, Kd=0.09) # Continuous time values

# System response to using the optimal values
fig, ax = plt.subplots(1, 2, figsize=(14, 8)) 
fig.suptitle("Close-Loop System Response to a 10° Correction for both Ascent and Descent Cases", fontsize=14, fontweight="bold")
cases = [ascent_case, descent_case]

for i, case in enumerate(cases):
    values = samples(case["factor"])
    opt_values = values[-1]
    controller = PDController(opt_values[0], opt_values[1])
    yaw_history, tvc_history = sim(controller,-10,0,time_frame,case["thrust"],case["moment_arm"]) # Step input of -10 with optimised values
    motor_y = np.linspace(min(yaw_history), max(tvc_history), 100) * 180 / np.pi
    ax[i].plot(time_frame, np.degrees(yaw_history), label=r'Rocket Angle (deg)')
    ax[i].plot(time_frame, np.degrees(tvc_history), label=r'TVC Mount Angle (deg)')
    ax[i].plot(np.zeros(100)+case["burn_time"], motor_y, color='red', linestyle='--', label=f'{case["name"]} Motor Burn Time (s)')
    ax[i].plot(time_frame, np.zeros(len(time_frame)), color='k', linestyle='--',  label=r'Target (0°)')

    # ax[i].set_title(f"{case["name"]} Case", fontsize=14)
    ax[i].set_xlabel(r"Time (s)", fontsize=12, fontweight="bold")
    ax[i].set_ylabel(f"{case["name"]} Angles (deg)", fontsize=12, fontweight="bold")
    ax[i].legend(loc="upper right")
    ax[i].grid(alpha=0.7)

plt.tight_layout()
plt.show()

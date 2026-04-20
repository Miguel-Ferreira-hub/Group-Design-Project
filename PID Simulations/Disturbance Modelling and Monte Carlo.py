import numpy as np
from numpy import random
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

plt.style.use('seaborn-v0_8-whitegrid')

np.random.seed(42) # Fix seed for reproducibility

# Simulation domain and time step
start = 0
end_ascent = 4.8
end_descent = 2.5
dt = 0.071
time_frame_ascent = np.arange(start, end_ascent, dt)
time_frame_descent = np.arange(start, end_descent, dt)
I = 0.229 # Mass moment of inertia

# Define dictionaries containing ascent and descent case dynamics
ascent_case = {
    "thrust": 74, # Average motor thrust 
    "moment_arm": 0.363, # Moment arm of TVC mount, thrust action point to centre of mass distance
    "factor": 4, # Scale domain to test different ranges
    "burn_time": 1.2, # Motor burn time
    "time_frame": time_frame_ascent, # Time frame of simulation
    "Kp": 0.96, # Optimised proportional gain
    "Kd": 0.12, # Optimised derivative gain
    "name": "Ascent" # Name
}

descent_case = { # Descent case dynamics
    "thrust": 20,
    "moment_arm": 0.28,
    "factor": 17,
    "burn_time": 2.5,
    "time_frame": time_frame_descent,
    "Kp": 4.09,
    "Kd": 0.51,
    "name": "Descent"
}

# Starting pitch and yaw characteristics for the rocket (neutral position)
default_yaw = 0
default_yaw_rate = 0
n_sims = 10000 # Number of Monte Carlo iterations

 # PD controller
class PDController():
    def __init__(self, Kp, Kd): # Constructor, initialise gain values
        self.Kp = Kp
        self.Kd = Kd

    def compute(self, yaw, yaw_rate, yaw_ref=0):
        error = yaw_ref - yaw # Compute error
        return self.Kp * error - self.Kd * yaw_rate # Return PD output

def gust(): # Generate random gust forces and torques: varying from 0 to 25 mph, returns signed values 
    low = 0 # Lowest gust speed in mph
    high = 10 # Highest gust speed in mph
    mph = np.arange(low,high) # Gust speed in mph
    velocity = mph/2.237 # Gust speed in m/s
    Cd = 1.5 # Assuming relatively high Cd
    D = 0.084 # Rocket tubing diameter
    L = 0.75 # Rocket body length
    A = D * L # Side area for cross flow
    rho = 1.225 # Density of air under standard conditions (1atm, 298K)
    force = 0.5 * rho * Cd * A * (random.choice(velocity))**2 # Drag force (suitable values generated)
    rocket_moment_arm = 0.12 # Distance between Cp and Cg
    disturbance_torque = force * rocket_moment_arm # Disturbance torque
    duration = np.random.uniform(0.200, 1) # Generate random gust duration
    sign = np.random.choice([-1,1]) # Generate random direction
    disturbance_torque *= sign # Apply direction to torque
    return disturbance_torque, duration

def sim(controller, angle, angle_rate, time_frame, thrust, moment_arm):
    angle_history = [] # Keeping track of history
    tvc_history = []
    gust_history = []
    angle = np.radians(angle) # Convert angles to radians
    angle_rate = np.radians(angle_rate)
    current_gust = 0 # Variable for the time step of the current gust
    gust_end_time = 0 # Variable for the end time of a gust
    gust_frequency = 1 # Expected frequency of 1 gust per second
    dt = 0.071 # Arduino sample time
    p = gust_frequency * dt # Probability of gust appearing based on Bernoulli trial time-step
    
    for t in time_frame: # This logic ensures gusts don't overlap
        if t >= gust_end_time: # Time must be greater than the current gust end time
            if np.random.rand() < p: # Chance of gust appearing if no gust is present
                current_gust, duration = gust() # Generate random gust
                gust_end_time = t + duration # Gust end time set as current time plus gust duration
            else:
                current_gust = 0 # No gust
        if t >= 1.2:
            thrust = 0 # No thrust after motor burn time
            tvc = 0 # TVC reset 
        else:
            tvc = controller.compute(angle, angle_rate) # Work out TVC angle
            tvc = np.clip(tvc, np.radians(-15), np.radians(15)) # TVC physical angle limit
        disturbance_torque = current_gust # Disturbance torque
        gust_history.append(disturbance_torque) # Keep track of history by adding step values
        angle_history.append(angle)
        tvc_history.append(tvc)

        torque = thrust * moment_arm * np.sin(tvc) + disturbance_torque # Restoring torque plus disturbance torque

        alpha = torque / I # Angular acceleration based on torque experienced

        angle_rate = angle_rate + alpha * dt # Angular rate
        angle = angle + angle_rate * dt # Angle position
    return angle_history, tvc_history, gust_history

# Monte Carlo simulation running various random gusts simulations
def monte_carlo(controller, n_sims, default_yaw, default_yaw_rate, time_frame, thrust, moment_arm, burn_time):
    final_angles = [] # Assign for history
    powered_angles = []
    mean =[]
    std = []
    for _ in range(0,n_sims):
        angle_history, _, _ = sim(controller, default_yaw, default_yaw_rate, time_frame, thrust, moment_arm)
        final_angles.append(angle_history[-1])
        mean.append(np.mean(final_angles))
        std.append(np.std(final_angles))
        mask = time_frame <= burn_time
        powered_angles.append(np.array(angle_history)[mask])
    metrics = { # Store metrics for yaw 
        "mean": np.degrees(np.mean(final_angles)),
        "std": np.degrees(np.std(final_angles)),
        "max": np.degrees(np.max(np.abs(final_angles))),
        "min": np.degrees(np.min(np.abs(final_angles)))
    }
    all_angles = np.degrees(final_angles) # Store final yaw angles from all iterations
    powered_angles = np.degrees(powered_angles)

    # Failure criteria
    threshold = 1 # Failure determined by deviation of this value from neutral position
    failure_rate = np.mean(powered_angles > threshold) # Fraction of all true values from boolean array where angle > threshold during powered flight

    # % that exceed n degrees by the end
    n1 = 15 # Cannot exceed 15 degrees
    threshold1 = np.mean(all_angles > n1) * 100

    n2 = 30 # Cannot exceed 15 degrees
    threshold2 = np.mean(all_angles > n2) * 100

    n3 = 45 # Cannot exceed 15 degrees
    threshold3 = np.mean(all_angles > n3) * 100

    # Print metrics
    print(f"-------------------{case["name"]}-------------------")
    print("Average Final Angle: ", metrics["mean"])
    print("Standard Deviation of Final Angle: ", metrics["std"])
    print("Maximum Final Angle (Absolute): ", metrics["max"])
    print("Minimum Final Angle (Absolute): ", np.float64(metrics["min"]))
    print("Failure Rate: ", np.float64(failure_rate))
    print(f"Percentage of runs that exceed {n1} degrees: {threshold1}")
    print(f"Percentage of runs that exceed {n2} degrees: {threshold2}")
    print(f"Percentage of runs that exceed {n3} degrees: {threshold3}")
    return metrics, all_angles, powered_angles, mean, std

# Plot distributions
fig, ax = plt.subplots(2, 2, figsize=(14, 8)) 
fig.suptitle("Monte Carlo Distribution", fontsize=14, fontweight="bold")
cases = [ascent_case, descent_case]

for i, case in enumerate(cases):
    controller = PDController(Kp=case["Kp"], Kd=case["Kd"])
    metrics,all_angles,powered_angles,mean,std = monte_carlo(controller, n_sims, default_yaw, default_yaw_rate, case["time_frame"], case["thrust"], case["moment_arm"], case["burn_time"])
    ax[0,i].hist(all_angles, bins=100)
    print(max(mean))
    ax[0,i].set_xlabel(f"{case["name"]} Final Angle (deg)", fontsize=12, fontweight="bold")
    ax[0,i].set_ylabel(f"Frequency", fontsize=12, fontweight="bold")
    ax[0,i].legend(loc="upper right")
    ax[0,i].grid(alpha=0.7)

    ax[1,i].plot(mean, label='Mean of Final Angle (deg)')
    ax[1,i].plot(std, label='Standard Deviation of Final Angle (deg)')
    ax[1,i].set_xlabel(f"Iteration Number", fontsize=12, fontweight="bold")
    ax[1,i].set_ylabel(f"{case["name"]} Statistical Properties", fontsize=12, fontweight="bold")
    ax[1,i].legend(loc="upper right")
    ax[1,i].grid(alpha=0.7)

ax[1,0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
ax[1,0].set_ylim(-0.2,0.7)
ax[1,1].set_ylim(-0.05,0.30)
plt.tight_layout()
plt.show()
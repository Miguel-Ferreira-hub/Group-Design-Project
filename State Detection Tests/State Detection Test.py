# This script aims to fabricate sensor data for a rocket in flight
# and read tested data to see where launch, burnout, apogee etc were detected
import matplotlib.pyplot as plt # Imports
import numpy as np
import pandas as pd
import os

# OS path and code decisions
directory = r'C:\Users\migue\Desktop\Part IV\GDP\Generated Data'
path = os.path.join(directory, 'DATA.csv')
plt.style.use('ggplot')
noise_enabled = True # When set to True, sensor readings are simulated with noise
path_state = os.path.join(directory, 'TEST3.csv')
states = pd.read_csv(path_state)
state_names = ['Launch Detected', 'Burnout Detected', 'Apogee Detected', 'Descent Detected', 'Landed']
time_of_event = {}
location = {}

# Locate detected state
for state in state_names:
    rows = states.loc[states['State Detection'] == state]

    if not rows.empty:
        time_of_event[state] = rows['Time (s)'].iloc[0]/1000
        location[state] = rows['Altitude (m)'].iloc[0]

# Initialising arrays for simulated data
time = np.linspace(0,20,1000)
altitude = np.zeros(len(time))
velocity = np.zeros(len(time))
acceleration = np.zeros(len(time))

# Generating noise
noise_alt = np.random.normal(0,0.27,1000)
noise_vel = np.random.normal(0,1.2,1000)
noise_accel = np.random.normal(0,0.62,1000)

# Generating curves
for i in range(len(time)):
    if time[i] < 5 or time[i] > 15:
        altitude[i] = 0
        velocity[i] = 0
        acceleration[i] = -9.81
    else:
        altitude[i] = -2*(time[i]-10)**2 + 50
        velocity[i] = -4*(time[i] - 10)

    
for i in range(len(time)):
    if time[i] > 5 and time[i] < 7.18:
        acceleration[i] = -25*(time[i]-6)**2 + 25
    elif time[i] > 7.18:
        acceleration[i] = -9.81

# Adding noise
if noise_enabled == True:
    altitude += noise_alt
    velocity += noise_vel
    acceleration += noise_accel

# Plotting curves and state detection
plt.figure(figsize=(12,12))
plt.plot(time, altitude, color='red', label='Altitude (m)')
plt.plot(time, velocity, color='yellow', label='Velocity (ms-1)')
plt.plot(time, acceleration, color='green', label='Acceleration (ms-2)')
for state in state_names: # Annotate on graph where state detection occurred
    plt.annotate(
        state, xy=(time_of_event[state],location[state]), xytext=(time_of_event[state],location[state]),
        fontsize=10,
        fontweight='bold'
    )
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m), Velocity (ms-1), Acceleration (ms-2)')
plt.legend()
plt.title('Simulated State Detection Test 3')
plt.show()

# Produce data files
data = pd.DataFrame({'Time (s)': time, 'Acceleration (ms-2)': acceleration, 'Velocity (ms-1)': velocity, 'Altitude (m)': altitude})
data.to_csv(path, index=False)
import numpy as np # Library imports
import matplotlib.pyplot as plt

neutral = 1480 # Neutral position on the servo in PWM (90 degrees)

angle_range = np.arange(-30,30) # Angle range moving either side

gradient = 10 # Approximate gradient between PWM signal and servo angle

pwm = gradient * angle_range + neutral # Relationship (linear)

# Plot relationship
plt.figure(figsize=(12,9))
plt.title('Relationship Between PWM Signal Time Period and Servo Angle Output')
plt.plot(angle_range+90, pwm, color='red')
plt.xlabel('Servo Angle (deg)')
plt.ylabel('PWM (μs)')
plt.grid()
plt.show()
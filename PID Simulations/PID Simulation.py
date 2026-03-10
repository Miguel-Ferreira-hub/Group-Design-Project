import numpy as np
import matplotlib.pyplot as plt

start = 0
end = 3
dt = 0.071
# dt = 0.001

thrust = 74
moment_arm = 0.28
I = 0.05

time_frame = np.arange(start, end + dt, dt)

class PDController():
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

    def compute(self, yaw, yaw_rate, yaw_ref=0):
        error = yaw_ref - yaw
        return self.Kp * error - self.Kd * yaw_rate

yaw = -10 * np.pi/180
yaw_rate = 0 * np.pi/180

#controller = PDController(Kp=0.9, Kd=0.09)

controller = PDController(Kp=0.25, Kd=0.030)

yaw_history = []
tvc_history = []

for _ in time_frame:
    yaw_history.append(yaw)
    tvc = controller.compute(yaw, yaw_rate)
    # TVC limit (26 degrees converted to radians)
    tvc = np.clip(tvc, -26*np.pi/180, 26*np.pi/180)
    tvc_history.append(tvc)

    torque = thrust * moment_arm * np.sin(tvc)

    alpha = torque / I

    yaw_rate = yaw_rate + alpha * dt
    yaw = yaw + yaw_rate * dt 
motor_y = np.linspace(min(yaw_history), max(tvc_history), 100) * 180 / np.pi
motor_t = np.zeros(100) + 1.2
motor_t2 = np.zeros(100) + 2.4
plt.figure(figsize=(10, 6))
plt.plot(time_frame, np.degrees(yaw_history), label='Rocket Angle (deg)')
plt.plot(time_frame, np.degrees(tvc_history), label='TVC Angle (deg)')
plt.plot(time_frame, np.zeros(len(time_frame)), color='gray', linestyle='--',  label='Target (0°)')
plt.plot(motor_t, motor_y, color='red', linestyle='--', label='Motor Burn time (s)')
plt.plot(motor_t2, motor_y, color='red', linestyle = '--', label='Second Motor Burn Time (s)')
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("PD Controller Response")
plt.legend(loc='upper right')
plt.grid()
plt.show()
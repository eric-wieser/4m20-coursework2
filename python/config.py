import numpy as np

N = 3

# the pulses corresponding to 0 and 90 degrees
servo_0_90 = np.array([
	[1205, 2031],
	[1396, 2274],
	[1157, 2022]
])
servo_per_radian = (servo_0_90[:,1] - servo_0_90[:,0]) / np.radians(90)
servo_0 = servo_0_90[:,0]

# the limits beyond which the servo can't tell the difference, and might cause
# damage
servo_limits = (550, 2300)

import numpy as np

N = 3

# the pulses corresponding to 0 and 90 degrees
servo_0_90 = np.array([
	[1170, 2040],
	[1410, 2274],
	[1217, 2055]
])
servo_per_radian = (servo_0_90[:,1] - servo_0_90[:,0]) / np.radians(90)
servo_0 = servo_0_90[:,0]

# the limits beyond which the servo can't tell the difference, and might cause
# damage
servo_limits = (550, 2300)

# the length of the links, in meters
lengths = np.array([
	0.125,
	0.148,
	0.149,
	0.139
])

# TODO: https://github.com/eric-wieser/4m20-coursework2/issues/12
com = lengths / 2

# TODO: https://github.com/eric-wieser/4m20-coursework2/issues/4
masses = np.ones(4)

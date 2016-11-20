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
radian_limit_from_zero = min( (servo_0_90[0][0] - servo_limits[0])/servo_per_radian[0] , abs(servo_0_90[0][1] - servo_limits[0])/servo_per_radian[1] )

# the length of the links, in meters
lengths = np.array([
	0.125,
	0.148,
	0.149,
	0.139
])

# These are the adc readings when a link is held horizontally, and the adjacent
# link allowed to fall under gravity. The heavier side of the joint is the on
# that should be held. By repeating the experiment with the setup upsidedown,
# we should get two torques that sum to zero, giving us the reading
# corresponding to zero torque
_adc_lims = np.array([
	[515, 525],
	[455, 600],
	[519, 544]
])
adc_0 = _adc_lims.mean(axis=1)

# TODO: https://github.com/eric-wieser/4m20-coursework2/issues/12
com = lengths / 2

# TODO: https://github.com/eric-wieser/4m20-coursework2/issues/4
masses = np.ones(4)


# Make everything read only!

for arr in [servo_0, servo_per_radian, adc_0, com, masses, lengths]:
	arr.setflags(write=False)

if __name__ == '__main__':
	print('servo_0', servo_0)
	print('servo_per_radian', servo_per_radian)
	print('radian_limit_from_zero', radian_limit_from_zero)
	print('adc_0', adc_0)
	print('lengths', lengths)
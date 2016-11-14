def pJ(qq):
	import numpy as np
	from numpy import sin, cos
	from numpy.linalg import pinv
	qq1 = qq[0]
	qq2 = qq[1]
	qq3 = qq[2]
	qq4 = qq[3]
	J1 = [-0.125*cos(qq1) - 0.148*cos(qq1 + qq2) - 0.149*cos(qq1 + qq2 + qq3) - 0.139*cos(qq1 + qq2 + qq3 + qq4), -0.148*cos(qq1 + qq2) - 0.149*cos(qq1 + qq2 + qq3) - 0.139*cos(qq1 + qq2 + qq3 + qq4), -0.149*cos(qq1 + qq2 + qq3) - 0.139*cos(qq1 + qq2 + qq3 + qq4), -0.139*cos(qq1 + qq2 + qq3 + qq4)]	
	J2 = [-0.125*sin(qq1) - 0.148*sin(qq1 + qq2) - 0.149*sin(qq1 + qq2 + qq3) - 0.139*sin(qq1 + qq2 + qq3 + qq4), -0.148*sin(qq1 + qq2) - 0.149*sin(qq1 + qq2 + qq3) - 0.139*sin(qq1 + qq2 + qq3 + qq4), -0.149*sin(qq1 + qq2 + qq3) - 0.139*sin(qq1 + qq2 + qq3 + qq4), -0.139*sin(qq1 + qq2 + qq3 + qq4)]
	J = np.array([J1, J2], dtype=np.float)
	return(pinv(J))

def f(qq):
	import numpy as np
	from numpy import sin, cos
	from numpy.linalg import pinv
	qq1 = qq[0]
	qq2 = qq[1]
	qq3 = qq[2]
	qq4 = qq[3]
	lengths = np.array([
		0.125,
		0.148,
		0.149,
		0.139
	])
	x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
	y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)
	return([[x],[y]])
import numpy as np
from numpy import sin, cos
from numpy.linalg import pinv

def pJ(qq):
	qq1 = qq[0][0]
	qq2 = qq[1][0]
	qq3 = qq[2][0]
	qq4 = qq[3][0]
	from config import lengths
	#J1 = [-lengths[0]*cos(qq1) - lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[3]*cos(qq1 + qq2 + qq3 + qq4)]
	#J2 = [-lengths[0]*sin(qq1) - lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[3]*sin(qq1 + qq2 + qq3 + qq4)]
	J = np.array([[-lengths[0]*cos(qq1) - lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[3]*cos(qq1 + qq2 + qq3 + qq4)], [-lengths[0]*sin(qq1) - lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[3]*sin(qq1 + qq2 + qq3 + qq4)]],dtype=np.float64)
	return(pinv(J))

def f(qq):
	qq1 = qq[0][0]
	qq2 = qq[1][0]
	qq3 = qq[2][0]
	qq4 = qq[3][0]
	from config import lengths
	#x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
	#y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)
	return(np.array([[-(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))],[lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)]], dtype=np.float64))
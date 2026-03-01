import math
from math import sin,cos
from numpy import arange

l2= 0.065       # link lengths in meters
l3= 0.094
l4= 0.101
l5= 0.180

def clamp(x,min_value,max_value):
    return max(min_value, min(x, max_value))

def arccos(x):
    # clamped, just to deal with small numerical errors
    x = clamp(x,-1,1)
    return math.acos(x)

def square_root(x):
    # clamped, to deal with small numerical errors
    if (x<0) and x>-(10**-14):
        x = 0
    return math.sqrt(x)

def forward_kinematics(alpha,beta):
    # Calculate arm height and extension from joint angles (diff from hardware coord system)
    z = (l5-l2)+l4*sin(alpha)-l3*sin(alpha-beta)
    rho = -l4*cos(alpha)+l3*cos(alpha-beta)
    return z,rho

def inverse_kinematics(z,rho):
    # Calculate joint angles (diff from hardware coord system) corresponding to an arm height and extension 
    abs_tolerance = 1e-9  # meters, for z and rho
    rel_tolerance = 1e-6

    inverse_beta = arccos((l3**2+l4**2-(z-l5+l2)**2-rho**2)/(2*l3*l4))
    E = (l3**2+l4**2-(z-l5+l2)**2-rho**2)/(2*l4)-l4     # aux variables
    F = (l3*square_root(1-((l3**2+l4**2-(z-l5+l2)**2-rho**2)/(2*l3*l4))**2))
    inverse_alpha1 = arccos((rho*E+F*square_root(E**2+F**2-rho**2))/(E**2+F**2))
    inverse_alpha2 = arccos((rho*E-F*square_root(E**2+F**2-rho**2))/(E**2+F**2))
    # Two possible solutions, one tends to be absolutely nonsense (not even a weird alternative configuration)
    # Run forward kinematics on our two solutions, keep the one that gives the original input back:
    z1,rho1 = forward_kinematics(inverse_alpha1,inverse_beta)
    z2,rho2 = forward_kinematics(inverse_alpha2,inverse_beta)
    if math.isclose(z1, z , rel_tol=rel_tolerance, abs_tol=abs_tolerance):
        if math.isclose(rho1, rho , rel_tol=rel_tolerance, abs_tol=abs_tolerance):
            return inverse_alpha1, inverse_beta
    if math.isclose(z2, z , rel_tol=rel_tolerance, abs_tol=abs_tolerance):
        if math.isclose(rho2, rho , rel_tol=rel_tolerance, abs_tol=abs_tolerance):
            return inverse_alpha2, inverse_beta
    # If neither of the solutions seems good:
    print("Inverse kinematics failed")

if __name__ == "__main__":
    alpha, beta = inverse_kinematics(z = 0.2, rho = 0.1)
    print(alpha*180/math.pi)
    print(beta*180/math.pi)
    # for alpha in arange(0,math.pi,math.pi/12):
    #     for beta in arange(0,math.pi,math.pi/12):
    #         z,rho = forward_kinematics(alpha,beta)
    #         inverse_alpha, inverse_beta = inverse_kinematics(z,rho)
    #         #print("For alpha={}, beta={}".format(alpha,beta))
    #         #print("Inverse: {}, {}".format(inverse_alpha, inverse_beta))
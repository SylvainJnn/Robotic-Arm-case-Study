import numpy as np

def compute_phi(x):
    if(x >0):
        return(0)
    return(-np.pi)

def compute_r3(r, l3, phi):#check
    return(r-l3*np.cos(phi))
    
def compute_z3(z, l3, phi):
    return(z-l3*np.sin(phi))

def compute_q2(r3, z3, l1, l2):
    print(r3, z3, l1, l2)
    numerator1 = r3**2 + z3**2
    numerator2 = l1**2 + l2**2
    denominator = 2 * l1* l2
    return(-np.arccos((numerator1-numerator2)/(denominator)))

def compute_beta(r3, z3):#recjeck
    numerator = z3
    denominator = np.sqrt(r3**2 + z3**2)
    return(np.arcsin(numerator/denominator))


def compute_gama_sin(q2, r, l2):    #sinus method
    return(np.arcsin(np.sin(q2)*l2/r))

def compute_gama_cos(r3, z3, l1, l2):#there is a sign error somewhere
    numerator = r3**2 + z3**2 + l1**2 - l2**2
    denominator = 2 * l1 * (r3**2 + z3**2)**(1/2)
    return(np.arccos(numerator/denominator))

def compute_q1(beta, gama):
    return(beta - gama)

def q1_2(r3,z3,l1,l2,q2):#compute first angle
    a = np.arctan2(z3,r3)
    b_numerator = l2*np.sin(q2)
    b_denominator = l1 + l2*np.cos(q2)
    b = np.arctan2(b_denominator,b_numerator)
    print(a,b)
    return(a + b)
    

def compute_q3(phi, q1, q2):
    return(phi - q1 - q2)

#=============
def q20(x,y,l1,l2):#compte second angle
    numerator1 = x**2 + y**2
    numerator2 = l1**2 + l2**2
    denominator = 2 * l1* l2
    return(-np.arccos((numerator1-numerator2)/(denominator)))

def q10(x,y,l1,l2,q2):#compute first angle
    a = np.arctan2(y,x)
    b_numerator = l2*np.sin(q2)
    b_denominator = l1 + l2*np.cos(q2)
    b = np.arctan2(b_numerator, b_denominator)
    print(a,b)
    return(a + b)


print("==========")
print("==========")
print("le MIEN")
l1 = 0.1
l2 = 0.1
l3 = 0.15
d = 0.03

coef =1

xx = 0.00
xy = (l1+l3+l2)

x = (xx**2 + xy**2)**0.5*coef
y = (d)*coef
phi = compute_phi(x)
#arm lengths

workspace_edge = ((x-1)**2 + y**2)**0.5#bad anmes
arm_edge = l1+l2
if(workspace_edge >arm_edge):
    print("not possible")

x3 = compute_r3(x, l3, phi)
y3 = compute_z3(y, l3, phi)
print("les 3: ")
print(x , y)
print(x3, y3)
print("====")
q2 = q20(x3,y3,l1,l2)
lim = False
if lim == True:
    if(q2 <-np.pi/2):
        q2 =-np.pi/2 
q1 = q10(x3,y3,l1,l2, q2)
q1 = q1 + np.pi/2 # dunno, but if we want the first one set as the unit cercle we have to do this
#q1 = -q1
if lim == True:
    if(q1 > np.pi/2):
        q1 = np.pi/2
q3 = phi - (q1+q2)

print("angles")
print(q1, q2, q3)
print(np.degrees(q1), np.degrees(q2),np.degrees(q3))


print(180 + np.degrees(q2))
print(180 + np.degrees(q3))


print("====")
print("====")
print("====")







#parameters
#x y position
x = x - l3
#arm lengths



q2 = q20(x,y,l1,l2)
q1 = q10(x,y,l1,l2, q2)
q1 = q1 + np.pi/2 # dunno, but if we want the first one set as the unit cercle we have to do this
print("yo")
print(q1, q2)
print(np.degrees(q1), np.degrees(q2))



print("==========")
print("==========")
print("==========")
print("==========")

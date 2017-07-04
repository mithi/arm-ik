
# coding: utf-8

# In[1]:

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix


# In[2]:

def pose(theta, alpha, a, d):
  # returns the pose T of one joint frame i with respect to the previous joint frame (i - 1)
  # given the parameters:
  # theta: theta[i]
  # alpha: alpha[i-1]
  # a: a[i-1]
  # d: d[i]

  r11, r12 = cos(theta), -sin(theta)
  r23, r33 = -sin(alpha), cos(alpha)
  r21 = sin(theta) * cos(alpha)
  r22 = cos(theta) * cos(alpha)
  r31 = sin(theta) * sin(alpha)
  r32 = cos(theta) * sin(alpha)
  y = -d * sin(alpha)
  z = d * cos(alpha)
    
  T = Matrix([
    [r11, r12, 0.0, a],
    [r21, r22, r23, y],
    [r31, r32, r33, z],
    [0.0, 0.0, 0.0, 1]
  ])
  
  T = simplify(T)

  return T


# In[3]:

# get the pose (homogenous transforms) of each joint wrt to previous joint

q1, q2, q3, q4, q5, q6= symbols('q1:7')

d90 = pi / 2 

T01 = pose(q1, 0, 0, 0.75)
T12 = pose(q2 - d90, -d90, 0.35, 0)
T23 = pose(q3, 0, 1.25, 0)
T34 = pose(q4, -d90, -0.054, 1.5)
T45 = pose(q5, d90, 0, 0)
T56 = pose(q6, -d90, 0, 0)
T6g = pose(0, 0, 0, 0.303)


# In[4]:

# From the poses, get the rotation of joint 3 wrt to the base frame and the transpose 
# We will need this later


T03 = simplify(T01 * T12 * T23)
R03 = T03[:3, :3]
R03T = R03.T

print("R03 = ")
print()
print(R03)

print("R03.T = ")
print()
print(R03T)


# In[5]:

# From the poses, get the rotation of joint 6 wrt to the joint 3
# We will need this later 

T36 = simplify(T34 * T45 * T56)
R36 = T36[:3, :3]

print("R36 = ")
print()
print(R36)


# In[6]:

# rotation matrices in x, y, z axes

def rotx(q):

  sq, cq = sin(q), cos(q)

  r = Matrix([
    [1., 0., 0.],
    [0., cq,-sq],
    [0., sq, cq]
  ])
    
  return r


def roty(q):

  sq, cq = sin(q), cos(q)

  r = Matrix([
    [ cq, 0., sq],
    [ 0., 1., 0.],
    [-sq, 0., cq]
  ])
    
  return r


def rotz(q):

  sq, cq = sin(q), cos(q)

  r = Matrix([
    [cq,-sq, 0.],
    [sq, cq, 0.],
    [0., 0., 1.]
  ])
    
  return r


# In[7]:

# the yaw, pitch roll is given wrt to the URDF frame 
# We must convert this to gripper frame by performing
# a rotation of 180 degrees ccw about the z axis and then 
# a rotation of 90 degrees cw about the new y axis

# This is the transpose of the rotation of the urdf frame wrt to gripper frame and its transpose
# ( which is strangely the same) which is important later

Rgu = (rotz(pi) * roty(-pi/2)).T
RguT = Rgu.T
print(RguT)
print(Rgu == RguT)


# In[8]:

#  euler_R is the composite rotation matrix of the following
# a rotation of alpha in the z axis
# a rotation of beta in the new y axis
# a rotation of gamma in the new x axis 
# this will be useful later 

alpha, beta, gamma = symbols('alpha beta gamma', real = True)
euler_R = simplify(rotz(alpha) * roty(beta) * rotx(gamma))
print(euler_R)


# In[9]:

def get_wrist_center(gripper_point, R0g, dg = 0.303):

  # get the coordinates of the wrist center wrt to the base frame (xw, yw, zw)
  # given the following info:
  # the coordinates of the gripper (end effector) (x, y, z)
  # the rotation of the gripper in gripper frame wrt to the base frame (R0u)
  # the distance between gripper and wrist center dg which is along common z axis
  xu, yu, zu = gripper_point 
    
  nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
  xw = xu - dg * nx
  yw = yu - dg * ny
  zw = zu - dg * nz 

  return xw, yw, zw


# In[10]:

# This is given position and orientation of the gripper wrt to URDFrame
px, py, pz = 0.49792, 1.3673, 2.4988
roll, pitch, yaw = 0.366, -0.078, 2.561


# In[11]:

gripper_point = px, py, pz

# This is the rotation of the gripper in URDF wrt to base frame 
R0u_eval = euler_R.evalf(subs = {alpha: yaw, beta: pitch, gamma: roll})

# R0g * Rgu = R0u 
R0g_eval = R0u_eval * RguT

# calculate wrist center
wrist_center = get_wrist_center(gripper_point, R0g_eval, dg = 0.303)
print("wrist_center", wrist_center)

# evaluated R0g
print("evaluated R0g:")
pprint(R0g_eval)


# In[12]:

def get_hypotenuse(a, b):
  # calculate the longest side given the two shorter sides of a right triangle using pythagorean theorem
  return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):
  # given all sides of a triangle a, b, c
  # calculate angle gamma between sides a and b  using cosine law
    
  cos_gamma = (a*a + b*b - c*c) / (2*a*b)
  sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
  gamma = atan2(sin_gamma, cos_gamma)

  return gamma

def get_first_three_angles(wrist_center):
  # given the wrist center which a tuple of 3 numbers x, y, z
  # (x, y, z) is the wrist center point wrt base frame
  # return the angles q1, q2, q3 for each respective joint
  # given geometry of the kuka kr210
    
  x, y, z  = wrist_center
    
  a1, a2, a3 = 0.35, 1.25, -0.054
  d1, d4 = 0.75, 1.5
  l = 1.50097168527591 #get_hypotenuse(d4, -a3)
  phi = 1.53481186671284 # atan2(d4, -a3)
  
  x_prime = get_hypotenuse(x, y)
  mx = x_prime -  a1
  my = z - d1 
  m = get_hypotenuse(mx, my)
  alpha = atan2(my, mx)
  
  gamma = get_cosine_law_angle(l, a2, m)
  beta = get_cosine_law_angle(m, a2, l)
  
  q1 = atan2(y, x)
  q2 = pi/2 - beta - alpha 
  q3 = -(gamma - phi)
    
  return q1, q2, q3 


# In[13]:

j1, j2, j3 = get_first_three_angles(wrist_center)

print("q1:", j1.evalf())
print("q2:", j2.evalf())
print("q3:", j3.evalf())


# In[14]:

'''
Recall that from our simplification earlier, R36 equals the following:

Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

From trigonometry we can get q4, q5, q6 if we know numerical values of all cells of matrix R36  
'''

def get_last_three_angles(R):
    
  sin_q4 = R[2, 2]
  cos_q4 =  -R[0, 2]
    
  sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2) 
  cos_q5 = R[1, 2]
    
  sin_q6 = -R[1, 1]
  cos_q6 = R[1, 0] 
  
  q4 = atan2(sin_q4, cos_q4)
  q5 = atan2(sin_q5, cos_q5)
  q6 = atan2(sin_q6, cos_q6)
    
  return q4, q5, q6


# ```
#  - R0g = R03 * R36 * R6g
#  - frame of joint 6 is the same orientation of gripper frame so  R6g = I
#  - R03.T * R0g = R03.T * R03 * R36 * I
#  ---> R36 = R03.T * R0g
#  
#  Recall we have this expression earlier for R03T:
#    Matrix([
#     [sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
#     [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
#     [            -sin(q1),              cos(q1),             0]])
# 
# 
#  Recall we also have evaluated R0g earlier.
#    Matrix([
#      [0.257143295038827, 0.488872082559650, -0.833595473062543],
#      [0.259329420712765, 0.796053601157403, 0.546851822377060], 
#      [0.930927267496960, -0.356795110642117, 0.0779209320563015]])
#      
# 
#   We also have solved for q1, q2, q3 earlier:
#     q1: 1.01249809363771
#     q2: -0.275800363737724
#     q3: -0.115686651053748
# 
#   So we can actually evaluate for R36 because we have numerical values for
#     R03.T and R0g
# ```

# In[15]:

R03T_eval = R03T.evalf(subs = {q1: j1.evalf(), q2: j2.evalf(), q3: j3.evalf()})
R36_eval = R03T_eval * R0g_eval

j4, j5, j6 = get_last_three_angles(R36_eval)

print("q1:", j1.evalf())
print("q2:", j2.evalf())
print("q3:", j3.evalf())
print("q4:", j4.evalf())
print("q5:", j5.evalf())
print("q6:", j6.evalf())


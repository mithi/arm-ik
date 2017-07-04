
# coding: utf-8

# In[1]:

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix


# In[2]:

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


# In[3]:

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


# In[4]:

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

T0g_a = simplify(T01 * T12 * T23 * T34 * T45 * T56 * T6g)


# In[5]:

# Total transform wrt gripper given
# yaw (alpha), pitch (beta), roll (beta)
# position px, py, pz

px, py, pz = symbols('px py pz', real = True)
alpha, beta, gamma = symbols('alpha beta gamma', real = True)

R = rotz(alpha) * roty(beta) * rotx(gamma) * (rotz(pi) * roty(-pi/2)).T

T0g_b = Matrix([
  [R[0, 0], R[0, 1], R[0, 2], px],
  [R[1, 0], R[1, 1], R[1, 2], py],
  [R[2, 0], R[2, 1], R[2, 2], pz],
  [0, 0, 0, 1]
])

T0g_b = simplify(trigsimp(T0g_b))
print(T0g_b)


# In[6]:

''' 
px, py, pz = 0.49792, 1.3673, 2.4988
roll, pitch, yaw = 0.366, -0.078, 2.561

q1: 1.01249809363771
q2: -0.275800363737724
q3: -0.115686651053751
q4: 1.63446527240323
q5: 1.52050002599430
q6: -0.815781306199679
'''

Tb = T0g_b.evalf(subs = {
  gamma: 0.366, #roll
  beta: -0.078, #pitch
  alpha: 2.561, #yaw
  px: 0.49792, 
  py: 1.3673,
  pz: 2.4988
})

print()
pprint(Tb)
print()
print(T0g_b)


# In[7]:

Ta = T0g_a.evalf(subs = {
  q1: 1.01249809363771,
  q2: -0.275800363737724,
  q3: -0.115686651053751,
  q4: 1.63446527240323,
  q5: 1.52050002599430,
  q6: -0.815781306199679    
})

print()
pprint(Ta)
print()
print(T0g_a)


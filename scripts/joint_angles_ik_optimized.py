
# coding: utf-8

# In[1]:

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix


# In[2]:

q1, q2, q3, q4, q5, q6= symbols('q1:7')
alpha, beta, gamma = symbols('alpha beta gamma', real = True)
px, py, pz = symbols('px py pz', real = True)


# In[3]:

R03 = Matrix([
[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
[sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
[        cos(q2 + q3),        -sin(q2 + q3),        0]])

R03T = Matrix([
[sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
[cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
[            -sin(q1),              cos(q1),             0]])


# In[4]:

R36 = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])


# In[5]:

R0u = Matrix([
[1.0*cos(alpha)*cos(beta), -1.0*sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), 1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
[1.0*sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma) + 1.0*cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha)],
[          -1.0*sin(beta),                                     1.0*sin(gamma)*cos(beta),                                    1.0*cos(beta)*cos(gamma)]])


# In[6]:

Rgu_eval = Matrix([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])
RguT_eval = Matrix([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])


# In[7]:

# Total transform wrt gripper given
# yaw (alpha), pitch (beta), roll (beta)
# position px, py, pz

T0g_b = Matrix([
[1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma),  1.0*sin(alpha)*cos(gamma) - 1.0*sin(beta)*sin(gamma)*cos(alpha), 1.0*cos(alpha)*cos(beta), px],
[sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha), -1.0*sin(alpha)*sin(beta)*sin(gamma) - 1.0*cos(alpha)*cos(gamma), 1.0*sin(alpha)*cos(beta), py],
[                                   1.0*cos(beta)*cos(gamma),                                        -1.0*sin(gamma)*cos(beta),           -1.0*sin(beta), pz],
[                                                          0,                                                                0,                        0,  1]])


# In[8]:

# Total transform wrt gripper given 
# angles q1, q2, q3, q4, q5, q6

T0g_a = Matrix([
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])


# In[9]:

def get_hypotenuse(a, b):
  return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):    
  cos_gamma = (a*a + b*b - c*c) / (2*a*b)
  sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
  gamma = atan2(sin_gamma, cos_gamma)

  return gamma

def get_wrist_center(gripper_point, R0g, dg = 0.303):

  xu, yu, zu = gripper_point 
    
  nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
  xw = xu - dg * nx
  yw = yu - dg * ny
  zw = zu - dg * nz 

  return xw, yw, zw
def get_first_three_angles(wrist_center):
    
  x, y, z  = wrist_center
    
  a1, a2, a3 = 0.35, 1.25, -0.054
  d1, d4 = 0.75, 1.5
  l = 1.50097168527591
  phi = 1.53481186671284
  
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


# In[10]:

def ik(x, y, z, roll, pitch, yaw, debug = False):
    
  # input: given position and orientation of the gripper wrt to URDFrame
  # output: angles q1, q2, q3, q4, q5, q6
    
  gripper_point = x, y, z

  R0u_eval = R0u.evalf(subs = {alpha: yaw, beta: pitch, gamma: roll})
  R0g_eval = R0u_eval * RguT_eval

  wrist_center = get_wrist_center(gripper_point, R0g_eval, dg = 0.303)

  j1, j2, j3 = get_first_three_angles(wrist_center)

  R03T_eval = R03T.evalf(subs = {q1: j1.evalf(), q2: j2.evalf(), q3: j3.evalf()})
  R36_eval = R03T_eval * R0g_eval

  j4, j5, j6 = get_last_three_angles(R36_eval)

  j1 = j1.evalf()
  j2 = j2.evalf()
  j3 = j3.evalf()
  j4 = j4.evalf()
  j5 = j5.evalf()
  j6 = j6.evalf()

  if debug:
  
    print()
    print("\n x:", x, "\n y:", y, "\n z:", z)
    print("\n roll:", roll, "\n pitch:", pitch, "\n yaw:", yaw)
        
    print()
    print(" j1:", j1, "\n j2:", j2, "\n j3:", j3)
    print(" j4:", j4, "\n j5:", j5, "\n j6:", j6)
    
    print()
    print("wrist_center", wrist_center)

    print()
    print("evaluated R0g:")
    pprint(R0g_eval)

    print()
    print("Total transform wrt gripper: given yaw (alpha), pitch (beta), roll (beta), px, py, pz")
    pprint(T0g_b.evalf(subs = {
      gamma: roll, beta: pitch, alpha: yaw, px: x, py: y, pz: z
    }))

    print()
    print("Total transform wrt gripper: given angles q1, q2, q3, q4, q5, q6")
    pprint(T0g_a.evalf(subs = {
      q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6    
    }))
    
  return j1, j2, j3, j4, j5, j6


# In[11]:

qs = ik(x = 0.49792, y = 1.3673, z = 2.4988,
        roll = 0.366, pitch = -0.078, yaw = 2.561, debug = True)


# In[18]:

qs = ik(x = 2.3537, y = -0.1255546, z = 2.841452,
        roll = 0.131008, pitch = -0.10541, yaw = 0.0491503, debug = True)

# -0.07, 0.41, -1.07, 0.32, 0.46, 0 


# In[19]:

qs = ik(x = 2.3537, y = -0.1255546, z = 2.841452,
        yaw = 0.131008, pitch = -0.10541, roll = 0.0491503, debug = True)

# -0.07, 0.41, -1.07, 0.32, 0.46, 0 


# In[ ]:




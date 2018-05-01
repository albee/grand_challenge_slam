import sys
!{sys.executable} -m pip install --user numpy scipy matplotlib nose
import numpy
import math
import matplotlib
import matplotlib.pyplot as plt
from numpy import pi
np = numpy
from utils import *
%matplotlib inline

# have a rotation matrix
def rot_mat(theta):
    return numpy.matrix([[numpy.cos(theta), -numpy.sin(theta)], [numpy.sin(theta), numpy.cos(theta)]])
  
  
# extensions of the above (and its inverse) that also returns jacobians (because jacobians are necessary to linearise)

def toFrame(F, p):
  """
  F: reference frame [f.x; f.y; f.theta] (e.g. robot pose)
  p: point in GLOBAL frame as COLUMN vector
  
  Returns:
  (pf, PF_f, PF_p): point in frame F, Jacobian wrt F, Jacobian wrt P
  
  """
  
  # to simplify the notation just a little:
  x = F[0,0]
  y = F[1,0]
  theta = F[2,0]
  px = p[0,0]
  py = p[1,0]
  
  R = rot_mat(theta)
  pf = R.T * (p - F[0:2,0]) # transposing a rotmat yields its inverse
  
  PF_f = numpy.matrix([
      [-numpy.cos(theta), -numpy.sin(theta), numpy.cos(theta) * (py - y) - numpy.sin(theta) * (px - x)],
      [numpy.sin(theta), -numpy.cos(theta), numpy.cos(theta) * (px - x) - numpy.sin(theta) * (py - y)]
  ])
  PF_p = R.T
  
  return (pf, PF_f, PF_p)

def fromFrame(F, pf):
  """
  F: reference frame [f.x; f.y; f.theta] (e.g. robot pose)
  pf: point in F frame as COLUMN vector
  
  Returns:
  (p, P_f, P_p): point in GLOBAL frame, Jacobian wrt F, Jacobian wrt P
  
  """
  
  # to simplify the notation just a little:
  x = F[0,0]
  y = F[1,0]
  theta = F[2,0]
  px = pf[0,0]
  py = pf[1,0]
  
  R = rot_mat(theta)
  p = R * pf + F[0:2,0]
  
  P_f = numpy.matrix([
      [1, 0, -py * numpy.cos(theta) - px * numpy.sin(theta)],
      [0, 1, px * numpy.cos(theta) - py * numpy.sin(theta)]
      
  ])
  P_p = R
  return (p, P_f, P_p)

  # Sensor model for a generic range/bearing sensor.

def scan(p):
  """
  Performs a range-and-bearing measurement of a 2D point in SENSOR frame.

  Return: (y, Y_p)
  y: the (range; bearing) measurement
  Y_p: jacobian wrt p

  """
  # to simplify notation...
  py = p[1,0]
  px = p[0,0]

  # write out the norm (for instructional purposes)
  d = numpy.sqrt(px ** 2 + py ** 2)

  y = numpy.matrix([[d], [numpy.arctan2(py, px)]])
  Y_p = numpy.matrix([
      [px / numpy.sqrt(px ** 2 + py ** 2),    py / numpy.sqrt(px ** 2 + py ** 2)],
      [-py / (px ** 2 * (py ** 2 / px ** 2 + 1)), 1 / (px * (py ** 2 / px ** 2) + 1)]
  ])

  return (y, Y_p)


def invScan(y):
  """
  Inverts a range-and-bearing measurement to a 2D point in SENSOR frame.

  Return: (p, P_y)
  """
  d = y[0,0]
  theta = y[1,0]

  p = numpy.matrix([[d * numpy.cos(theta)], [d * numpy.sin(theta)]])
  P_y = numpy.matrix([
      [numpy.cos(theta), -d * numpy.sin(theta)],
      [numpy.sin(theta), d * numpy.cos(theta)]
  ])

  return (p, P_y)


def observe(r, p):
  """
  Transforms a point p to robot frame r [x, y, theta] and takes a range-and-bearing
  measurement y.

  Return: (y, Y_r, Y_p)
  
  Note: this function is what some documents call h(p), with Jacobian H_r, H_p.
  """

  pr, PR_r, PR_p = toFrame(r, p)
  y, Y_pr = scan(pr)

  Y_r = Y_pr * PR_r
  Y_p = Y_pr * PR_p

  return (y, Y_r, Y_p)

def invObserve(r, y):
  """
  Transforms a range-and-bearing observation y [range, bearing] taken in a
  frame r [x, y, theta] into a world point p.
  
  Return: (p, P_r, P_y)
  
  Note: this function is what some documents call g(y), with Jacobian G_r, G_y.
  """

  pr, PR_y = invScan(y)
  p, P_r, P_pr = fromFrame(r, pr)
  P_y = P_pr * PR_y

  return (p, P_r, P_y)

  # robot dynamics

def move(F, u, n):
  """
  MOVE Robot motion, with separated control and perturbation inputs.
  
  In:
  F: robot pose F = [x; y; theta]
  u: control signal u = [dx; dy; dtheta]
  n: perturbation, additive to control signal n = [nx; ny; ntheta]
  Out:
  Fo: updated robot pose F = [x; y; theta]
  FO_F: Jacobian d(Fo) / d(F) 
  FO_n: Jacobian d(Fo) / d(n)
  """
  theta = F[2,0];
  
  dtheta = u[2,0] + n[2,0];
  
  thetao = theta + dtheta;
  if thetao > pi:
    thetao = thetao - 2*pi
  elif thetao < -pi:
    thetao = thetao + 2*pi

  # build position increment dp = [dx dy], from control signal dx
  dp = u[0:2,0]  
  p, P_f, P_p = fromFrame(F, dp)
  
  # Compute the necessary Jacobians.
  # Note that dtheta / dF depends only on the angle term
  # (the coefficient of theta in which is 1)
  # and that dtheta / dn depends only also on the angle term
  # (the coefficient of n_theta in which is dt, which for us is also 1).
  AO_a = 1
  AO_da = 1
  
  FO_F = numpy.concatenate([P_f, numpy.matrix([[0, 0, AO_a]])])
  FO_n = numpy.vstack([numpy.hstack([P_p, numpy.matrix('0; 0')]), numpy.matrix([[0, 0, AO_da]])])
  
  Fo = numpy.vstack([p, thetao])
  return [Fo, FO_F, FO_n]

  import numpy.random

########################################
# Demonstrations
########################################

# Dead Reckoning Demonstration
dt = 0.1

r = Robot()
r.request_control(1, 0, 1)

w = World()
w.create_random_P(100)
w.choose_random_L()
w.set_X(r.x)

trace = r.pos()
trace_w = w.X

for i in range(60):
    r.step_dead_reckon(w, dt)
    trace = numpy.concatenate([trace, r.pos()[:,0]], 1) # robot internal state
    trace_w = numpy.concatenate([trace_w, w.X[:,0]], 1) # robot ground truth state
    if i%10 == 0:
      w.plot_r(r)
    
plt.plot(trace[0].flat, trace[1].flat)
plt.plot(trace_w[0].flat, trace_w[1].flat)
plt.show()

# Map and Agent Demonstration
w = World()
r = Robot()
w.create_random_P(100)
w.choose_random_L()
w.tester()

# Robot Sensing Demonstration
w = World()
r = Robot()
w.create_random_P(100)
w.choose_random_L()
w.P = numpy.array([]).reshape(0, 2)

# Populate P with a bunch of sensor readings
for z in range(100):
  
  y_arr = np.matrix(r.request_landmarks(w))
  p_arr = np.zeros(y_arr.shape)
  
  for i in range(len(y_arr)):
    # in a non-test situation use a position estimator NOT the real position!!
    p, __1, __2 = invObserve(r.x, y_arr[i].T)
    p_arr[i] = p.T
  
  w.P = numpy.vstack((w.P, p_arr))
    
# and plot what things look like now
w.plot_w_r(r)

# Create Robot and World
w = World()
r = Robot()
w.create_random_P(100)
w.choose_random_L()
w.P = numpy.array([]).reshape(0, 2) #eliminate features landmarks were chosen from

# Create World Estimator
we = WorldEstimator(np.matrix(np.zeros(3)).T, np.matrix(np.zeros((3,3)))) 

# Display the GROUND_TRUTH landmarks and features, as well as the  state...
w.plot_w_r(r)

# Temporal Loop
for t in range(20):
  # Run an EKF iteration...
  we.doStep(r, w)

  # and update the world (robot=green; landmarks=red).
  r.step(w, 1)
  
  # display the ground truth states
  if t%2 == 0:
    w.plot_ground_truth()
  
  # display the estimated robot state at each step (robot=yellow)...
  if t%2 == 0:
    we.plot_r()
  
  # ...and display the estimated world (landmarks=blue) at each step.
    we.plot_l()

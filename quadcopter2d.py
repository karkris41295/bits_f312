from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np

##############################################################################
#INITIAL CONDITIONS: Where is the quadcopter at the beginning of the simulation?
##############################################################################
v0 = 0; u0 = 0; w0 = 0; x0 = 2.; y0 = 2.; theta0 = np.radians(0);

##############################################################################
#SET TARGET: Where do you want the quadcopter to be?
##############################################################################
target = [0, 5] # [x-coordinate, y-coordinate]

##############################################################################
#FUZZY CONTROLLER
##############################################################################
from skfuzzy import control as ctrl

#Antecedents
alt_error = ctrl.Antecedent(np.arange(-20, 20, .5), 'alt_error')
alt_delta = ctrl.Antecedent(np.arange(-5, 5, .125), 'alt_delta')

theta = ctrl.Antecedent(np.arange(-3.14, 3.14 ,.05), 'theta')
omega = ctrl.Antecedent(np.arange(-3.14/4, 3.14/4 ,.05/4), 'omega')

#Consequents
force = ctrl.Consequent(np.arange(5.8, 13.8, .2), 'force')
torque = ctrl.Consequent(np.arange(-3.14, 3.14, .05), 'torque')

# Auto-membership function population is possible with .automf(3, 5, or 7)
names = ['nb', 'ns', 'ze', 'ps', 'pb']

alt_error.automf(names=names)
alt_delta.automf(names=names)

#x_error.automf(names=names)
theta.automf(names=names)
omega.automf(names=names)

force.automf(names=names)
torque.automf(names=names)

# ALTITUDE CONTROL RULE BASE #################################################
rule0 = ctrl.Rule(antecedent=((alt_error['pb'] & alt_delta['nb']) |
                              (alt_error['ze'] & alt_delta['nb']) |
                              (alt_error['ps'] & alt_delta['nb'])),
                  consequent=force['pb'], label='rule pb')

rule1 = ctrl.Rule(antecedent=((alt_error['pb'] & alt_delta['ze']) |
                              (alt_error['pb'] & alt_delta['ns']) |
                              (alt_error['ps'] & alt_delta['ns']) |
                              (alt_error['ps'] & alt_delta['ze']) |
                              (alt_error['ns'] & alt_delta['nb']) |
                              (alt_error['ze'] & alt_delta['ns'])),
                  consequent=force['ps'], label='rule ps')

rule2 = ctrl.Rule(antecedent=((alt_error['pb'] & alt_delta['pb']) |
                              (alt_error['pb'] & alt_delta['ps']) |
                              (alt_error['ps'] & alt_delta['ps']) |
                              (alt_error['ze'] & alt_delta['ze']) |
                              (alt_error['ns'] & alt_delta['ns']) |
                              (alt_error['nb'] & alt_delta['nb']) |
                              (alt_error['nb'] & alt_delta['ns'])),
                  consequent=force['ze'], label='rule ze')

rule3 = ctrl.Rule(antecedent=((alt_error['ps'] & alt_delta['pb']) |
                              (alt_error['ze'] & alt_delta['ps']) |
                              (alt_error['ns'] & alt_delta['ps']) |
                              (alt_error['ns'] & alt_delta['ze']) |
                              (alt_error['nb'] & alt_delta['ze']) |
                              (alt_error['nb'] & alt_delta['ps'])),
                  consequent=force['ns'], label='rule ns')

rule4 = ctrl.Rule(antecedent=((alt_error['ze'] & alt_delta['pb']) |
                              (alt_error['ns'] & alt_delta['pb']) |
                              (alt_error['nb'] & alt_delta['pb'])),
                  consequent=force['nb'], label='rule nb')

# INCLINATION STABILIZATION RULE BASE ########################################

rule5 = ctrl.Rule(antecedent=((theta['pb'] & omega['nb']) |
                              (theta['ze'] & omega['nb']) |
                              (theta['ps'] & omega['nb'])),
                  consequent=torque['pb'], label='rule pb')

rule6 = ctrl.Rule(antecedent=((theta['pb'] & omega['ze']) |
                              (theta['pb'] & omega['ns']) |
                              (theta['ps'] & omega['ns']) |
                              (theta['ps'] & omega['ze']) |
                              (theta['ns'] & omega['nb']) |
                              (theta['ze'] & omega['ns'])),
                  consequent=torque['ps'], label='rule ps')

rule7 = ctrl.Rule(antecedent=((theta['pb'] & omega['pb']) |
                              (theta['pb'] & omega['ps']) |
                              (theta['ps'] & omega['ps']) |
                              (theta['ze'] & omega['ze']) |
                              (theta['ns'] & omega['ns']) |
                              (theta['nb'] & omega['nb']) |
                              (theta['nb'] & omega['ns'])),
                  consequent=torque['ze'], label='rule ze')

rule8 = ctrl.Rule(antecedent=((theta['ps'] & omega['pb']) |
                              (theta['ze'] & omega['ps']) |
                              (theta['ns'] & omega['ps']) |
                              (theta['ns'] & omega['ze']) |
                              (theta['nb'] & omega['ze']) |
                              (theta['nb'] & omega['ps'])),
                  consequent=torque['ns'], label='rule ns')

rule9 = ctrl.Rule(antecedent=((theta['ze'] & omega['pb']) |
                              (theta['ns'] & omega['pb']) |
                              (theta['nb'] & omega['pb'])),
                  consequent=torque['nb'], label='rule nb')

force_ctrl = ctrl.ControlSystem([rule0, rule1, rule2, rule3, rule4])
spin_ctrl = ctrl.ControlSystem([rule5, rule6, rule7, rule8, rule9])

spinning = ctrl.ControlSystemSimulation(spin_ctrl)
forcing = ctrl.ControlSystemSimulation(force_ctrl)
##############################################################################

##############################################################################
#QUADCOPTER SIMULATOR
##############################################################################
def Quadcopter2D(state,t):
  # unpack the state vector
  v = state[0]
  u = state[1]
  w = state[2]
  theta = state[5]

  # these are our constants
  m = 1.5 # Kilograms
  g = 9.8 # metres per second
  
  # error terms
  a_e = target[1] - state[3]
  forcing.input['alt_error'] = a_e * abs((20./(target[1]-y0))) #scaling gains
  forcing.input['alt_delta'] = v * abs((20./(target[1]-y0))) #scaling gains
  
  spinning.input['theta'] = -theta
  spinning.input['omega'] = w

  forcing.compute()
  f = forcing.output['force']
  spinning.compute()
  tau = spinning.output['torque'] 
  
  #tau = 0
  f1 = m * (f+tau)/2 # Force by right propeller in newtons
  f2 = m * (f-tau)/2 # Force by left propeller in newtons
  l = .7 # length of quadcopter in meters
  I = m*l**2/12. # moment of inertia about center of gravity
  
  v_dot = (-m*g + f1 * np.cos(theta) + f2*np.cos(theta))/m
  u_dot = -(f1*np.sin(theta) + f2 * np.sin(theta))/m
  w_dot = (f1*l/2 - f2*l/2)/I
  y_dot = v
  x_dot = u
  theta_dot = w

  # return the state derivatives
  return [v_dot, u_dot, w_dot, y_dot, x_dot, theta_dot]

##############################################################################

##############################################################################
#PLOTTING
##############################################################################

state0 = [v0, u0, w0, y0, x0, theta0]
t = np.arange(0.0, 80.0, 0.1) # Simulation of 10 seconds

state = odeint(Quadcopter2D, state0, t)

#Right propeller coordinates
rpy = state[:,3]+.7/2*np.sin(state[:,5])
rpx = state[:,4]+.7/2*np.cos(state[:,5])

#Left propeller coordinates
lpy = state[:,3]-.7/2*np.sin(state[:,5])
lpx = state[:,4]-.7/2*np.cos(state[:,5])

fig1 = plt.figure() 
plt.plot(rpx, rpy)
plt.plot(lpx, lpy)
plt.plot(state[:,4], state[:,3])
plt.scatter([state[0,4], lpx[0], rpx[0], lpx[-1],rpx[-1], state[-1,4]], [state[0,3], lpy[0], rpy[0], lpy[-1],rpy[-1], state[-1,3]])
plt.xlabel('x position')
plt.ylabel('y position')
plt.title('Universe (floor is at level 0)')
plt.legend(('right propeller', 'left propeller', 'center of gravity'))
plt.ylim(0,)

fig2 = plt.figure()

plt.subplot(311)
plt.plot(t, state[:,0])
plt.ylabel('Vertical Velocity')

plt.subplot(312)
plt.plot(t, state[:,1])
plt.ylabel('Horizontal Velocity')

plt.subplot(313)
plt.plot(t, state[:,2])
plt.ylabel('Angular Velocity')
plt.xlabel('TIME (sec)')

fig3 = plt.figure()

plt.subplot(311)
plt.plot(t, state[:,3])
plt.ylabel('Altitude')

plt.subplot(312)
plt.plot(t, state[:,4])
plt.ylabel('x-position')

plt.subplot(313)
plt.plot(t, state[:,5])
plt.xlabel('TIME (sec)')
plt.ylabel('Inclination (theta)')

plt.show()
##############################################################################
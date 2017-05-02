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
                              (alt_error['nb'] & alt_delta['ns'])),
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
                              (theta['nb'] & omega['ns'])),
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
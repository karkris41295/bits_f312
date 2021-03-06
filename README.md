# Quadcopter movement control using a set of fuzzy rules

Let's say we have a stick with two propellers on either end (Let's call this our quadcopter). Can we manipulate the firing sequence of the two propellers we have so that we can go to a specific setpoint on the 2D plane?

This project attempts to address this problem by using the **scikit-fuzzy** fuzzy logic toolbox to compute propeller thrust intensities.

There are two files in this repository:
1. **quadcopter2d.py** - The full code which consists of the fuzzy rule base, quadcopter simulation and plotting trajectories
2. **fuzzy_control.py** - Just the fuzzy rule base without the rest of the code

*Note: In its current form, the controller is only able to meet the set altitude target (y-axis) but wanders off in the horizontal direction (x-axis).*

This code was written as an assignment for a Fuzzy Logic & Neural Networks course.

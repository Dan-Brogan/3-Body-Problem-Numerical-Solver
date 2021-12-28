# 3-Body-Problem-Numerical-Solver
This repository contains the two MATLAB files necessary to simulate the 3 body problem.

INPUTS:
-> Initial positions        (Found in "main_3body_fun.m")
-> Initial velocities       (Found in "main_3body_fun.m")
-> Masses                   (Found in "ode_3body_fun.m")
-> Gravitational Constant   (Found in "ode_3body_fun.m")
-> Time span                (Found in "main_3body_fun.m")

OUTPUTS:
-> Graphical animation of body motion through time

This script calculates the acceleration of each body using the laws of gravitational perturbation and integrates twice to calculate the position of the bodies at every time step.

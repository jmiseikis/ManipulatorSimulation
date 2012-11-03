Robotic Manipulator Simulation
=====================

Simple robotic manipulator movement simulation in MATLAB, part of a weekly coursework.
The goal was to implement an inverse kinematics solution for a simplified 2-joint robotic 
manipulator operating in a two dimensional (x, y) workspace. The manipulator has to follow 
number of points located in a circular pattern.

This is MATLAB file, simply add it to your MATLAB path or current directory. Parameters like 
number of points and step size (beta) can be modified in the code. It will affect the accuracy 
and the speed of convergence.

The following graphs are generated which can be useful for model analysis (n = 10, beta = 0.01):

Position convergence

![Position convergence](https://raw.github.com/jmiseikis/ManipulatorSimulation/master/Images/Pos_b0.01_p10.jpg)

Joint Angles

![Angle convergence](https://raw.github.com/jmiseikis/ManipulatorSimulation/master/Images/Angles_b0.01_p10.jpg)

And the whole robot configuration (50 points)

![Robot configuration](https://raw.github.com/jmiseikis/ManipulatorSimulation/master/Images/Robot_p50.jpg)
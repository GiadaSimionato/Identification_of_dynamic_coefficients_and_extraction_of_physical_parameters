# Identification of dynamic coefficients and extraction of physical parameters.
## Estimation of internal wrenches using Newton-Euler algorithms: effect of the presence or not of physical constraints on parameters.

Project repository for the course of Robotics II, Sapienza University of Rome.

This repository contains the MATLAB code for the derivation of the dynamic models of a 3R spatial anthropomorphic and of the 7R KUKA LWR IV+ manipulators in order to identify their corresponding dynamic coefficients and to extract their sets of dynamic parameters. This work presents the result of the analysis of the internal wrenches according to feasible and unfeasible sets of parameters even in presence of friction phenomenon.

This work was completed on December 30th, 2019.

## Contents:

- [Structure](#structure)
- [Execution](#execution)
- [References](#references)

## Structure
- **Enviroment** folder contains the full code of the framework, divided into:
	- **3R** folder containing the code for the 3R spatial anthropomorphic manipulator without friction modelling;
	- **3R_friction** folder containing the code for the 3R spatial anthropomorphic manipulator with friction modelling;
	- **7R** folder containing the code for the 7R KUKA LWR IV+ manipulator without friction modelling;
	- **7R_friction** folder containing the code for the 7R KUKA LWR IV+ manipulator with friction modelling;
- **report.pdf** is the report of this project.
For more information about the contents of this repository please refer to Chapter 4 of the report.

## References
- C. Gaz, M. Cognetti, A. Oliva, P.R. Giordano, A. De Luca. *Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization.* IEEE Robotics and Automation Letters, Vol. 4, No. 4, 2019.
- C. Gaz, F. Flacco, A. De Luca. *Extracting Feasible Robot Parameters from Dynamic Coefficients using Nonlinear Optimization Methods.* IEEE International Conference on
Robotics and Automation (ICRA), 2016.
- C. D. Sousa, R. Cortesão. *Inertia Tensor Properties in Robot Dynamics Identification: A Linear Matrix Inequality Approach.* IEEE Transaction on Mechatronics, Vol. 24, No. 1, 2019.
- C. Gaz, A. De Luca. *Payload Estimation Based on Identified Coefficients of Robot Dynamics with an Application to Collision Detection.* IEEE/RSJ International Conference
on Intelligent Robots and Systems (IROS), 2017.
- C. Gaz, F. Flacco, A. De Luca. *Identifying the Dynamic Model Used by the KUKA LWR: A Reverse Engineering Approach.* IEEE International Conference on Robotics and
Automation (ICRA), 2014.

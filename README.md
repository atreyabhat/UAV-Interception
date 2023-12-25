# UAV Interception: Control & Trajectory Optimization

## Overview
This project explores Quadrotor Control methodologies focused on intercepting unauthorized UAVs using Linear Quadratic Regulation (LQR) for precise tracking and interception. Discussions encompass various aspects:
- **State Estimation**
- **System Dynamics**
- **Control Strategy and Controller Tuning**

The primary objective is to achieve efficient UAV interception by designing a system that minimizes state variables and control signals using matrices Q and R within the cost function.

## Objective
The primary goal of this project is to develop a control system for a quadrotor tasked with monitoring a restricted airspace. Upon the entry of an unauthorized Unmanned Aerial Vehicle (UAV) into the airspace, the quadrotor aims to execute the following maneuvers:
1. **Leave the Nest:** Activate when the UAV enters the airspace.
2. **Real-time Tracking:** Follow and capture the UAV.
3. **Return to the Nest:** Safely return with the captured UAV.

## Assumptions
- **Nest Location:** The nest is positioned at the center of the bottom surface, serving as the origin of the airspace.
- **Airspace Dimensions:** The airspace measures 10x10x10 meters.
- **UAV Entry Time:** The intruding UAV enters the airspace at time t0 = 0.
- **Disturbance Consideration:** The weight of the UAV is integrated into the disturbance force and moment vectors r and n.




# LQR-based UAV Interception

![Quadrotor Control Methodology](https://link-to-your-image-here/quadrotor_methodology.jpg)

## Introduction

This project focuses on Quadrotor Control methodologies for intercepting unauthorized UAVs, utilizing Linear Quadratic Regulation (LQR) for precision tracking and interception. The objective is to monitor a restricted airspace, with the quadrotor designed to fly, intercept, and return unauthorized UAVs. Assumptions include a central nest, a 10x10x10 meters airspace, and an intruding UAV entering at time $t_{0}=0$. The project aims to minimize errors between actual and desired positions, considering external disturbances.

## Methodology

### Controller Design

#### System Modeling

The quadrotor's dynamics are represented by state variables $x$, Euler angles $\alpha$, velocity $v$, and angular velocity $\omega$. The linearized dynamics are captured in Jacobian matrices A and B. The state-space representation and control law are defined as:
\[ \dot{x} = Ax + Bu \]
\[ K = R^{-1} (B^T S + N^T) \]

#### LQR Control Design

The cost function $J$ is minimized, incorporating state and control penalties. The controller design involves tuning Q and R matrices. The Kalman filter is employed for state estimation, enhancing pursuit efficiency.

### Simulation Results

#### Effects of different Q and R values

1. Increasing Q stabilizes quickly but requires high input force.
2. Decreasing Q stabilizes slowly with lower input forces.
3. Increasing R limits input force, leading to slower stabilization.
4. Decreasing R stabilizes quickly but requires higher input force.

![Tuned Q and R](https://github.com/atreyabhat/RBE502_UAV_Interceptor/assets/39030188/ced5d884-d56c-4de1-b3a8-cb124a55acba)

#### Integral Action LQR Controller

To nullify steady-state error, integral action is applied. This mitigates error and makes the LQR robust to disturbances.

![Steady State Error](https://github.com/atreyabhat/RBE502_UAV_Interceptor/assets/39030188/4c39762c-174f-4213-86e6-795e8c103b58)
![UAV and Quadrotor Trajectories](https://github.com/atreyabhat/RBE502_UAV_Interceptor/assets/39030188/572e053c-387f-4394-a0c4-c230254d1877)

## Conclusions

This project explored the control methodology for quadrotor interception, emphasizing LQR design. While effective, the controller exhibits limitations in scenarios involving abrupt changes and external disturbances. The addition of Proportional-Integral gain (LQR-PI) reduces steady-state error, enhancing robustness.


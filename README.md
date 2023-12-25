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


# Defense / Control Systems Portfolio  
**Abel Palmer**

This repository contains projects in control systems, guidance, tracking, and system identification, with a focus on autonomous drone tracking and interception. The work emphasizes learning dynamical models from data, designing controllers for those models, and validating performance in closed-loop simulation.

A central project is a **drone tracking and interception system** using a **gain-scheduled LQG controller augmented with derivative feedback (PD structure)**, implemented in both **Python and C++**, designed to track and neutralize a moving target under noise and model uncertainty.

---

## Topics Covered

### System Identification

Learning dynamical models from input–output data.

- Subspace identification (CVA / N4SID)
- Block Hankel matrix construction
- Singular value analysis for order selection
- Discrete-time state-space model estimation
- Validation on held-out datasets

Typical workflow:

1. Generate excitation signals (multisine / chirp)
2. Collect input–output data
3. Construct past–future Hankel matrices
4. Perform SVD and analyze singular values
5. Estimate models across candidate orders
6. Select order via validation error / information criteria

---

### Control Design

Controllers are designed using state-space and frequency-domain methods.

**State-space methods**
- LQR
- Kalman filtering
- LQG control
- Gain scheduling across operating conditions

**Classical control**
- PID / PD augmentation
- Loop shaping
- Sensitivity and complementary sensitivity analysis
- Nyquist and Bode methods

---

### Drone Tracking & Interception (Flagship Project)

Design and implementation of a closed-loop system for tracking and intercepting a moving target.

- Nonlinear quadrotor dynamics with linearization about operating points  
- Subspace identification of local linear models from simulated data  
- Gain-scheduled **LQG controller** for state estimation and control  
- **Derivative (PD) augmentation** for improved transient response  
- Tracking of a moving target with offset geometry (intercept behavior)  
- Implementation in both **Python (prototyping)** and **C++ (real-time capable structure)**  

Key features:

- Scheduling over velocity / direction operating points  
- Robustness to process and measurement noise  
- Closed-loop validation via simulation  
- Analysis of stability and bandwidth via frequency-domain tools  

---

### Robustness and Frequency-Domain Analysis

Analysis of feedback robustness and performance tradeoffs.

- Sensitivity and complementary sensitivity functions  
- Disturbance rejection characteristics  
- Bandwidth vs robustness tradeoffs  
- Nyquist stability margins  
- Loop shaping using Bode/Nyquist methods  
---

## Tools Used

- MATLAB
- Python
- NumPy / SciPy
- Matplotlib
- Jupyter notebooks
- C++ (Eigen, CMake)

MATLAB is used for control design and frequency-domain analysis.  
Python is used for system identification, simulation, and data processing.  
C++ is used for performance-oriented implementations of dynamics, estimation, and control.

---

## Motivation

Real systems are not known exactly; models must often be learned from data. This repository focuses on that workflow:

- excite a system  
- identify a model  
- design a controller  
- analyze robustness  

The emphasis is on how control methods behave when applied to learned models, and how to maintain stability and performance under modeling error and noise.

---

## Author

**Abel Palmer**  
Applied Mathematics – Control Systems

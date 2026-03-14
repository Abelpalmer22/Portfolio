# Quadrotor Project  
**Abel Palmer**

This project implements a complete system identification → control design → simulation pipeline for a quadrotor system. A nonlinear quadrotor model is constructed, linearized about a hover equilibrium, and then treated as an unknown system from which a model must be learned using input–output data. Controllers are then designed for both the learned model and the physics-derived linearization.

The goal is to examine how well standard control techniques perform when the plant model is obtained from data rather than from first principles.

---

## Model Construction and Linearization

A physically motivated **16-state nonlinear quadrotor model** is constructed and linearized about the hovering equilibrium.

The state vector includes

- position
- velocity
- attitude (Euler angles)
- angular velocity
- rotor speeds

Rotor speeds are included as states in order to model actuator dynamics. The commanded motor input \(u_i\) is assumed to track the actual rotor speed \(\Omega_i\) with first-order lag:

\[
\dot{\Omega}_i = \frac{1}{\tau_m}(u_i - \Omega_i)
\]

This captures the delay between commanded thrust and actual motor response.

The nonlinear model and hover linearization are implemented in

```
quadrotor_model.py
01_plant_definition_hover_linearization.ipynb
```

---

## System Identification from Data

**Notebook:** `02_plant_experiments.ipynb`

The quadrotor is treated as an unknown plant and identified from simulated input–output experiments.

### Excitation

Control inputs are generated using **multisine and chirp signals** spanning a range of frequencies in order to excite multiple dynamical modes of the system.

Gaussian noise is injected into the measured outputs to simulate sensor noise. Separate datasets are generated for

- training
- validation/testing

### Subspace Identification

The identification pipeline proceeds as follows:

1. Construct block Hankel matrices from the training dataset.
2. Compute the singular value decomposition (SVD) of the projected future outputs.
3. Plot singular values on a log scale to estimate plausible model orders.
4. Apply an **N4SID-style subspace identification algorithm** to extract discrete-time state-space realizations \(A,B,C,D\).
5. Simulate the identified models on the validation dataset and compute prediction error.
6. Plot eigenvalues of the identified state matrices to examine stability properties.
7. Evaluate candidate model orders using **AIC and BIC** as model-complexity checks.

---

## Control Design

**Notebook:** `03_lqg.ipynb`

Controllers are designed for both

- the **identified model**, and  
- the **physics-derived linearized model**.

### LQG Control

An \(H_2\) optimal controller (Linear Quadratic Gaussian control) is implemented.

The controller consists of

- an **LQR regulator**
- a **Kalman filter state estimator**

Design parameters include

- state penalty matrix \(Q\)
- control penalty matrix \(R\)
- process noise covariance \(W\)
- measurement noise covariance \(V\)

The noise model assumes Gaussian disturbances injected at each timestep. While this is a convenient modeling assumption, it tends to produce more persistent disturbance energy than would typically be observed in real hardware.

Example results are shown in

```
figures/plain_lqg_phys.png
```

### Observed Behavior

Simulations are performed using both the learned and physics-derived models.

Analysis includes

- eigenvalue plots of the closed-loop system
- PBH tests for controllability
- trajectory simulations under disturbance

Although the nominal regulator is stabilizing for the physics-derived model, the controller designed using the **identified realization** can be highly sensitive to estimation errors. In some simulations the learned model exhibits unstable closed-loop behavior, suggesting that the identified realization may be poorly conditioned.

### PID Augmentation

To explore robustness improvements, the controller is augmented with

- integral action
- derivative action

These modifications are evaluated by applying large perturbations to the system and examining the resulting transient response.

An alternative formulation is also explored in which the controller regulates **velocity** rather than absolute position in order to improve recovery from large disturbances.

---

## Current Status

This project is currently a **work in progress**.

Planned extensions include

- implementing **model predictive control (MPC)**
- comparing MPC performance with LQG/PID
- investigating conditioning issues in subspace identification
- studying how excitation design influences identification quality

In particular, improved excitation signals may be required to excite the full range of system modes in order to obtain well-conditioned state-space realizations.

---

## Files

```
quadrotor_model.py
01_plant_definition_hover_linearization.ipynb
02_plant_experiments.ipynb
03_lqg.ipynb
figures/
```
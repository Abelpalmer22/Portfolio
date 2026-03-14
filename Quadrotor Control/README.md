# Quadrotor Project  
**Abel Palmer**

This project implements a complete system identification → control design → simulation pipeline for a quadrotor system. A nonlinear quadrotor model is constructed, linearized about a hover equilibrium, and then treated as an unknown system from which a model must be learned using input–output data. Controllers are then designed for both the learned model and the physics-derived linearization.

The goal here is to implement standard industrial control techniques and investigate their performance when the plant model is learned from data instead of from first principles.

---

## Model Construction and Linearization

Firstly, a physically motivated **16-state nonlinear quadrotor model** is constructed and linearized around hover.

The state vector includes

- position
- velocity
- attitude (Euler angles)
- angular velocity
- rotor speeds

Rotor speeds are included as states in order to model actuator dynamics. The commanded motor input $$u_i$$ is assumed to track the actual rotor speed $$\Omega_i$$ with first-order lag:


$$ \dot{\Omega}_i = \frac{1}{\tau_m}(u_i - \Omega_i) $$

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
4. Apply an **N4SID-style subspace identification algorithm** to extract a DT state-space realization \(A,B,C,D\).
5. Simulate the identified models on the validation dataset and compute prediction error.
6. Plot eigenvalues of the identified state matrices to check stability.
7. Evaluate candidate model orders using **AIC and BIC** as complexity gut-checks.

---

## Control Design

**Notebook:** `03_lqg.ipynb`

Controllers and simulations are designed/run for both

- the **identified model (learned ABCD realization)**, and  
- the **physics-derived linearized model**.

### LQG Control

An $$H_2$$ optimal controller (Linear Quadratic Gaussian) with the classic LQR-Kalman filter loop is implemented:

Design parameters include

- state penalty matrix \(Q\)
- control penalty matrix \(R\)
- process noise covariance \(W\)
- measurement noise covariance \(V\)

The noise model assumes Gaussian disturbances injected at each timestep. It was found that this was a nice and mathematically convenient assumption but it tends to make the model shake a good bit more than would be expected or realistic in real hardware.

Example results are shown in

```
figures/plain_lqg_phys.png
```

### Observed Behavior

Simulations (trajectory plots) are performed using both the learned and physics-derived models.

Although the nominal regulator is stabilizing for the physics-derived model, the controller designed using the **identified realization** seems highly sensitive and poorly conditioned with reference to estimation errors. In most simulations the learned realization gives marginally stable if not unstable closed-loop behavior. This suggests the identified realization may be poorly conditioned (future work at bottom).

### PID Augmentation

The control scheme is then expanded to include an integral and a derivative (separately) component, in order to examine their stabilizing effects. It was noted during experimentation that given certain memory-length parameters on the integral, resonance was achieved which induced instability (a consequence of the smaller phase margin of an integrator).

These modifications are evaluated by applying large perturbations to the system and examining the resulting transient response.

The control scheme is then modified so that the drone "forgets" absolute position and simply focuses on minimizing translational velocity, which simulates more realistic drone dynamics.

---

## Current Status

This project is currently a **work in progress**.

I plan to extend this project by 

- implementing **model predictive control (MPC)** and comparing its performance with LQG/PID
- investigating conditioning issues in subspace identification
- studying how excitation design influences identification quality

As noted before, better excitation schema may be needed in order to produce informative enough data to yield a well-conditioned realization.

---

## Files

```
quadrotor_model.py
01_plant_definition_hover_linearization.ipynb
02_plant_experiments.ipynb
03_lqg.ipynb
figures/
```
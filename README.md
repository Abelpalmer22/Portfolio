# Defense/Control Systems Portfolio  
**Abel Palmer**

This repository contains a collection of control systems, guidance, tracking, navigation and system identification projects focused on building robust and tenacious tracking drone simulations, and learning (mostly linear, or linearized) dynamical models from data and designing controllers for those models. I employ typical control engineering workflows, like identifying models from input–output data, validation through simulation and designing controllers using classical control with modern robustness additions.

---

## Topics Covered

The projects in this repository involve techniques from system identification, classical control, and modern state-space control. Specific details can be found in the READMEs for each project.

### System Identification

Methods for learning dynamical models from experimental data.

- Subspace identification (CVA / N4SID style methods)
- Block Hankel matrix construction
- Singular value analysis for model order selection
- Identification of DT state-space models
- Validation of learned models on unseen datasets

Workflow used (subh as in the quadcopter project):

1. Generate excitation signals (multisine / chirp)
2. Collect IO data
3. Construct past–future Hankel matrices
4. Compute SVD and examine singular values
5. Estimate state-space models of different orders
6. Validate models through simulation error

---

### Control Design

Controllers are designed using both classical and modern techniques.

**State-space methods**

- LQR
- Kalman filtering
- LQG control
- State estimation for noisy systems

**Classical control**

- PID control
- Loop shaping
- Sensitivity and complementary sensitivity analysis
- Nyquist stability analysis
- Bode plots and frequency-domain robustness analysis

These controllers are applied to models learned through system identification.

---

### Robustness and Frequency-Domain Analysis

I examine robustness properties of feedback systems (e.g. in the shock absorber project):

Topics include

- sensitivity functions
- disturbance rejection
- bandwidth vs robustness tradeoffs
- Nyquist stability margins
- frequency-domain loop shaping
- interpretation of Bode and Nyquist plots

---

## Example Project Workflow

A typical project in this repository follows a structure like:

1. **Data generation or collection**

   A nonlinear or physical system is excited with carefully designed inputs.

2. **System identification**

   Subspace identification methods are used to estimate a linear state-space model.

3. **Model validation**

   The identified model is validated by comparing simulated outputs with observed data.

4. **Controller design**

   Controllers such as LQR, PID, or loop-shaped controllers are designed.

5. **Closed-loop simulation**

   The controlled system is simulated to evaluate

   - stability
   - disturbance rejection
   - transient response
   - robustness

---

## Repository Structure

```
projects/
    system_identification/
        subspace_id/
        hankel_svd_analysis/

    control_design/
        lqg_control/
        pid_control/
        loop_shaping/

    robust_analysis/
        nyquist_analysis/
        bode_analysis/

matlab/
    control_design.m
    frequency_analysis.m

python/
    subspace_id.ipynb
    simulation_tools.py

figures/
    bode_plots/
    nyquist_plots/
    singular_values/
```

---

## Tools Used

- MATLAB
- Python
- NumPy / SciPy
- Matplotlib
- Jupyter notebooks

MATLAB is used primarily for control design and frequency-domain analysis, while Python is used for system identification, simulation, and data processing.

---

## Motivation

Real engineering systems are rarely known exactly. Instead, engineers often learn models from measured data and design controllers based on those models.

This repository explores that workflow:

- excite a system  
- identify a model  
- design a controller  
- analyze robustness  

The emphasis is on understanding how control techniques behave when models are learned from data rather than derived from first principles.

---

## Author

**Abel Palmer**  
Applied mathematics and control systems

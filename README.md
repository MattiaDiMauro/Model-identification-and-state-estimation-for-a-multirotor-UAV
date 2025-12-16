# Model-identification-and-state-estimation-for-a-multirotor-UAV

**Institution:** Politecnico di Milano  
**Course:** Estimation and learning  
**Professor:** Marco Lovera  
**Academic Year:** 2025/2026  
---

## Overview

MATLAB project on data-driven system identification and state estimation for the ANT-X quadrotor UAV using a 2DoF experimental test-bed. Includes closed-loop longitudinal dynamics identification from flight data and the design of a discrete-time Kalman-based pitch attitude estimator using IMU measurements.
---

## Main Topics

### System Identification
- Experimental flight test design and data acquisition
- Excitation input design (sine sweep and PRBS signals)
- Black-box state-space model identification
- Grey-box second-order state-space model estimation
- Model validation with independent datasets
- Fitting metrics computation (FIT%, VAF, RMSE)
- Uncertainty quantification and analysis
- Model order selection and comparison

### State Estimation
- Discrete-time Kalman Filter implementation from scratch
- Single-axis Multiplicative Extended Kalman Filter (MEKF) for attitude estimation
- Predictor-corrector formulation
- Gyroscope integration for pitch rate
- Accelerometer-based gravity vector measurement model
- Process and measurement noise covariance tuning
- Validation against experimental ground-truth data
- Performance evaluation in small-angle and large-angle maneuvers
- Residual analysis and filter consistency checks

---

## Requirements

- MATLAB 
- Control System Toolbox
- System Identification Toolbox
- Signal Processing Toolbox (optional, for advanced filtering)

---

## References

- Course notes
- Ljung L., *System Identification: Theory for the User*, Prentice Hall
- Crassidis J.L., Junkins J.L., *Optimal Estimation of Dynamic Systems*, CRC Press
- Markley F.L., Crassidis J.L., *Fundamentals of Spacecraft Attitude Determination and Control*, Springer

---

## License

Educational purposes – Politecnico di Milano (2024/2025)


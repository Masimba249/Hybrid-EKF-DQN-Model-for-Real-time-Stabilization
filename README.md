# Hybrid EKF-DQN Model for Real-time Stabilization and Drift Reduction of a Quadcopter UAV

<grok-card data-id="6043b2" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="b83d78" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="0c07c6" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


**A hybrid reinforcement learning approach integrating Extended Kalman Filter (EKF) for robust state estimation and Deep Q-Network (DQN) for adaptive decision-making, achieving superior quadcopter stabilization and drift reduction in adverse environments (e.g., wind, GPS-denied scenarios).**

[![GitHub stars](https://img.shields.io/github/stars/yourusername/ekf-dqn-quadcopter?style=social)](https://github.com/yourusername/ekf-dqn-quadcopter/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/yourusername/ekf-dqn-quadcopter?style=social)](https://github.com/yourusername/ekf-dqn-quadcopter/network/members)
[![GitHub issues](https://img.shields.io/github/issues/yourusername/ekf-dqn-quadcopter)](https://github.com/yourusername/ekf-dqn-quadcopter/issues)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Authors

- **Kuanysh Yessenzhanov** — School of Information Technology and Engineering, Kazakh-British Technical University, Almaty, Kazakhstan
- **Collins Masimba** — School of Information Technology and Engineering, Kazakh-British Technical University, Almaty, Kazakhstan (cmasimba@kbtu.kz)
- **Ilyas Muhammad** — School of Information Technology and Engineering, Kazakh-British Technical University, Almaty, Kazakhstan (m.ilyas@kbtu.kz)
- **Muhammad Hassan Tanveer** (Corresponding Author) — Dept. of Robotics and Mechatronics Engineering, Kennesaw State University, Marietta, GA 30060, USA
- **Razvan Cristian Voicu** — Dept. of Robotics and Mechatronics Engineering, Kennesaw State University, Marietta, GA 30060, USA

## Abstract (from Conference Paper)

Intelligent air combat has been one of the most trending topics of discussion across different disciplines of research. This research work presents a real-time stabilization and drift analysis of UAVs using a Multi-layered EKF-DQN Model. Several neural networks have been proposed to address the challenges of drone stabilization and drift in unmanned aerial vehicles.

Drone autonomy is characterized by the use of operations that require little or no human interaction. A lot of research has shown the implementation of traditional algorithms to determine the drone’s stability. Nevertheless, stability and drift present numerous difficulties, especially when utilizing traditional algorithms and testing them under hostile environmental settings.

The implementation of artificial intelligence plays a crucial role in predicting obstructions. To address this issue, we compare conventional AI algorithms against our reinforcement learning approach (DQN) to resolve drift analysis and stability of UAVs. Our model extracts 3D features from point clouds to provide precise stability even under adverse environmental conditions.

**Index Terms** — UAV, Quadcopter, Extended Kalman Filter (EKF), Deep Q-Network (DQN), Reinforcement Learning, Drift Reduction, Stabilization

<grok-card data-id="ff4876" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*Representative block diagram of a quadcopter control system (illustrating hybrid architecture concepts)*

## Problem Statement

Unmanned Aerial Vehicles (UAVs) frequently experience drift and instability during dynamic operations, especially in GPS-denied or uncertain environments. Traditional control methods (e.g., PID, EKF alone) fail to adequately compensate for real-time drift and complex nonlinear dynamics. EKF provides robust state estimation but lacks learning capacity. DQN offers adaptive decision-making but struggles with convergence in continuous state spaces.

This work proposes a **hybrid EKF-DQN model** that combines EKF's robustness with DQN's adaptability for effective real-time UAV stability and drift correction.

<grok-card data-id="606a72" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="801b3b" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*Extended Kalman Filter (EKF) diagrams for state estimation in drones*

<grok-card data-id="18aaf8" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="411a1c" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*Deep Q-Network (DQN) architecture diagrams*

## Methodology

- **EKF Component**: Handles nonlinear state estimation (position, velocity, attitude) using sensor fusion (IMU, GPS, monocular camera).
- **DQN Component**: Learns optimal control policies through reinforcement learning in continuous state spaces via trial-and-error (reward/punishment paradigm).
- **Hybrid Integration**: EKF provides accurate state inputs to DQN; DQN outputs adaptive control actions.
- **Training**: Experience replay, target network, ε-greedy exploration.
- **Implementation**: MATLAB with Reinforcement Learning Toolbox.

## Simulation Results

Simulations demonstrate superior performance in ideal, moderate (20 m/s), and strong (50 m/s) wind conditions.

<grok-card data-id="b68b79" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="3a3004" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="60777b" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*MATLAB/Simulink quadcopter trajectory tracking and simulation examples*

<grok-card data-id="40b5c6" data-type="image_card"  data-arg-size="LARGE" ></grok-card>



<grok-card data-id="bb8f5e" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*DQN reward convergence curves during training*

<grok-card data-id="23d1ca" data-type="image_card"  data-arg-size="LARGE" ></grok-card>


*Step response characteristics (rise time, overshoot, settling time)*

### Comparative Performance vs. PID

| Metric                | Hybrid EKF-DQN | Traditional PID |
|-----------------------|----------------|-----------------|
| Rise Time             | ~2 s           | ~5 s            |
| Overshoot             | ~50%           | ~40%            |
| Settling Time         | ~10 s          | ~7 s            |
| Steady-State Error    | ~5%            | ~4%             |
| RMSE (Position)       | 1.2 m          | 1.5 m           |
| Wind Disturbance Rejection | Excellent     | Moderate        |

The hybrid model excels in disturbance rejection and adaptability.

## Repository Structure

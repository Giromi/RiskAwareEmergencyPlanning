# 🚑 Risk-Aware Emergency Vehicle Path Planning using LLM, RRT*, and MPC

> An AI-assisted autonomous emergency response system that minimizes traffic casualties by integrating **Large Language Models (LLMs), RRT*, Spline2D, and Model Predictive Control (MPC)**.

---

## 📖 Overview

This project presents an AI-assisted emergency vehicle planning framework for unavoidable traffic collision scenarios.

Unlike conventional autonomous driving systems that focus solely on obstacle avoidance, the proposed framework aims to **minimize human casualties** by combining semantic reasoning from a Large Language Model (LLM) with classical motion planning and optimal vehicle control.

The system first analyzes a traffic scene, predicts the safest collision point through LLM reasoning, generates an optimal path using RRT*, smooths the trajectory with Spline2D, and finally tracks the path using Model Predictive Control (MPC). The complete framework was validated through Webots simulations in urban traffic environments.

---

## 🎯 Motivation

Autonomous vehicles inevitably encounter situations where collisions cannot be completely avoided.

Instead of simply avoiding obstacles, autonomous systems should determine the collision strategy that minimizes overall human casualties.

This project investigates how Large Language Models can assist emergency decision-making while maintaining dynamically feasible trajectories for autonomous vehicles.

---

## 🏗 System Architecture

```mermaid
flowchart TD

A[🚨 Emergency Detected]
B[🗺 Environment Information]
C[📊 Grid Map Generation]
D[🧠 LLM Decision Module]
E[🎯 Collision Point Selection]
F[🌳 RRT* Path Planning]
G[📈 Spline2D Path Smoothing]
H[🚗 MPC Controller]
I[🤖 Webots Vehicle]
J[📊 Simulation Evaluation]

A --> B
B --> C
C --> D
D --> E
E --> F
F --> G
G --> H
H --> I
I --> J
```

---

# 🧠 LLM-Based Collision Reasoning

The proposed framework employs **Few-Shot Prompting** to improve collision reasoning capability.

Instead of relying solely on Zero-Shot prompting, representative traffic accident examples are provided to the LLM, allowing it to infer safer collision points under emergency situations.

### Features

- Grid-map representation
- Few-Shot Prompt Engineering
- Collision point prediction
- Casualty-aware reasoning
- Zero-Shot vs Few-Shot comparison

---
```mermaid
flowchart TD

A[Webots Simulation]
B[Environment Information]
C[LLM-Based Collision Reasoning]
D[Candidate Collision Goals]
E[RRT* Path Planning]
F[Spline2D Path Smoothing]
G[MPC Trajectory Tracking]
H[Reached Candidate Goal?]
I[Final Destination Reached?]
J[Vehicle Speed ≤ 10 km/h?]
K[Mission Complete]

A --> B
B --> C
C --> D
D --> E
E --> F
F --> G
G --> H

H -- No --> E
H -- Yes --> I

I -- No --> E
I -- Yes --> J

J -- No --> G
J -- Yes --> K
```
# 🌳 Path Planning

After selecting the collision point, the planner generates a collision-aware trajectory using **RRT***.

The generated waypoints are then refined through **Spline2D interpolation**, producing a smooth and dynamically feasible trajectory.

### Features

- Collision-aware waypoint generation
- RRT* global planning
- Spline2D trajectory smoothing
- Curvature optimization

---

# 🚗 Model Predictive Control (MPC)

The generated trajectory is tracked using **Model Predictive Control (MPC)**.

The controller optimizes steering commands while satisfying vehicle dynamics and steering constraints.

### Optimization Objectives

- Position tracking error
- Steering effort
- Steering rate
- Vehicle dynamics constraints
- Velocity constraints

---

# 🖥 Simulation Environment

The proposed framework was validated using **Webots**.

Simulation environment includes:

- Urban road scenarios
- Dynamic traffic environments
- Emergency vehicle navigation
- Collision-aware trajectory execution

---

# 🧪 Experimental Scenario

Experiments were conducted on multiple urban traffic scenarios, including the **Seoul City Hall intersection**.

The proposed framework generated significantly safer collision strategies compared with conventional planning methods.

---

# 📊 Results

| Scenario | Actual Casualties | Proposed Framework |
|-----------|------------------:|-------------------:|
| Seoul City Hall Intersection | 16 | 3 |

### Performance

- ✅ LLM-based collision reasoning
- ✅ Few-Shot prompting
- ✅ Collision-aware planning
- ✅ RRT* path planning
- ✅ Spline2D smoothing
- ✅ MPC trajectory tracking
- ✅ Webots validation

---

# 🛠 Technologies

| Category | Technologies |
|-----------|--------------|
| AI | Large Language Models (LLM), Few-Shot Prompting |
| Planning | RRT*, Spline2D |
| Control | Model Predictive Control (MPC) |
| Simulation | Webots |
| Programming | Python |
| Libraries | NumPy, Matplotlib |

---

# 📂 Repository Structure

```text
EmergencyPlanning/
│
├── llm/
│   ├── prompts/
│   ├── reasoning/
│   └── collision_prediction/
│
├── planner/
│   ├── rrt_star/
│   ├── spline2d/
│   └── waypoint_generation/
│
├── controller/
│   ├── mpc/
│   └── vehicle_model/
│
├── simulation/
│   ├── webots/
│   ├── scenarios/
│   └── maps/
│
├── experiments/
│
├── results/
│
└── docs/
```

---

# 👨‍💻 My Contributions

- Designed the overall AI-assisted emergency planning framework.
- Developed the LLM-based collision reasoning pipeline using Few-Shot prompting.
- Implemented grid-map generation and prompt construction.
- Integrated RRT* for collision-aware path planning.
- Applied Spline2D interpolation for trajectory smoothing.
- Designed and tuned the MPC controller for vehicle trajectory tracking.
- Built the complete simulation pipeline in Webots.
- Evaluated the framework through multiple urban crash scenarios.

---

# 🚀 Future Work

- Vision-Language Model (VLM) integration
- Camera and LiDAR perception
- Dynamic obstacle prediction
- ROS2 deployment
- CARLA implementation
- Multi-agent emergency planning
- Real vehicle validation

---

# 📚 Keywords

`Autonomous Driving` `Emergency Planning` `Large Language Models` `Few-Shot Learning` `Motion Planning` `RRT*` `Spline2D` `Model Predictive Control` `Webots` `Robotics`

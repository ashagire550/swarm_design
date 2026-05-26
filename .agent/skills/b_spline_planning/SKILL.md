---
name: "B-Spline Kinodynamic Planning"
description: "Implementation guidelines for generating, evaluating, and manipulating uniform B-splines for quadrotor trajectory generation."
---

# B-Spline Kinodynamic Planning

When designing the Local Planner Agent for the quadrotor swarm, use uniform B-splines to guarantee continuous, smooth motion.

## Core Concepts

1. **Definition**: A trajectory $\Phi(t)$ of degree $k$ is parameterized by a set of control points $Q_i$. For this system, always use a **degree $k=3$ (cubic)** to ensure jerk is continuous.
2. **Local Support**: Modifying a single control point $Q_i$ only affects the trajectory in the time interval influenced by exactly $k+1$ basis functions. This enables highly efficient and localized replanning without recalculating the entire global path.
3. **Convex Hull Property**: The B-spline curve is strictly contained within the convex hull of its control points. If the control polygons are collision-free, the quadrotor trajectory is guaranteed to be collision-free.

## Implementation Guidelines

### 1. Control Point Initialization
- Before optimization, initialize control points by sampling evenly spaced points along an A* or straight-line path.
- Distance between control points should be dictated by a constant time interval $\Delta t$ and expected average velocity.

### 2. Derivative Computation (Kinematics)
Do not numerically differentiate the curve. Utilize the derivative property of B-splines, which states that the derivative of a B-spline of degree $k$ is another B-spline of degree $k-1$.
- **Velocity** control points: $V_i = \frac{Q_{i+1} - Q_i}{\Delta t}$
- **Acceleration** control points: $A_i = \frac{V_{i+1} - V_i}{\Delta t} = \frac{Q_{i+2} - 2Q_{i+1} + Q_i}{\Delta t^2}$
- **Jerk** control points: $J_i = \frac{A_{i+1} - A_i}{\Delta t} = \frac{Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i}{\Delta t^3}$

### 3. Dynamic Feasibility Check
Constrain maximum velocity and acceleration by checking if any control point derivation $|V_i| > v_{max}$ or $|A_i| > a_{max}$. Because of the convex hull property, if the derivative control points respect the dynamic limits, the entire continuous trajectory strictly satisfies the limits.

### 4. Time Regularization
If dynamic limits are violated during optimization, apply a uniform time scaling factor instead of completely rejecting the trajectory. 
$r_{max} = \max(\sqrt[1]{v_{max\_found} / v_{limit}}, \sqrt[2]{a_{max\_found} / a_{limit}}, \sqrt[3]{j_{max\_found} / j_{limit}})$
Scale the intervals by $\Delta t_{new} = \Delta t \cdot r_{max}$ and recompute the final B-spline.

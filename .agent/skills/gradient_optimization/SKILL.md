---
name: "Gradient-Based Optimization for Planners"
description: "Implementation rules for creating analytical gradients and running L-BFGS or similar optimization solvers on trajectory control points."
---

# Gradient-Based Optimization

The core of the EGO-planner architecture replaces heavy sampling with gradient-based unconstrained optimization.

## Objective Function

The state vector $x$ consists of the modifiable B-spline control points. The optimization attempts to minimize the total scalar cost $J$:

`J(Q) = λ_s * J_s + λ_c * J_c + λ_d * J_d`

Where:
*   $J_s$: Smoothness Cost
*   $J_c$: Collision Cost
*   $J_d$: Dynamic Feasibility Cost

## Calculating Gradients Analytically

**CRITICAL REQUIREMENT:** Never use numerical gradients (finite differences) as they ruin the <1ms latency target. The gradient $\nabla J(Q)$ must be defined analytically.

### 1. Smoothness Cost $J_s$
Minimizes jerk or acceleration squared to ensure smooth quadrotor flight.
*   **Cost**: $J_s = \sum_{i} ||J_i||^2$ (Sum of squared jerk control points).
*   **Gradient**: Differentiate the cost w.r.t each constituent control point. Since $J_i$ linearly depends on $\{Q_i, Q_{i+1}, Q_{i+2}, Q_{i+3}\}$, the gradient $\frac{\partial J_s}{\partial Q_k}$ propagates across adjacent control points via a simple sliding window matrix operation.

### 2. Dynamic Feasibility Cost $J_d$
Soft penalizes velocity and acceleration control points that exceed the vehicle limits ($v_{max}$, $a_{max}$).
*   **Cost**: Use a one-sided quadratic penalty. 
    `if ||V_i|| > v_max: J_v += (||V_i||² - v_max²)²`
    Else, zero penalty. Apply the same for acceleration.
*   **Gradient**: Analytically differentiate the penalty term with respect to $Q$.

## Solving the Problem

### Execution Framework
1. **Initial Guess**: The A* path provides an excellent convex basin of attraction.
2. **Solver**: Utilize `L-BFGS` (e.g., via NLopt or Ceres Solver). It achieves superlinear convergence and uses memory efficiently.
3. **Execution Time Safety**: Strictly limit the optimizer to a maximum time allowance (e.g., 20ms). If convergence isn't reached, abort and rely on an intermediate viable trajectory.
4. **Trigger Conditions**: Only run the optimizer if an obstacle intersection is detected (event-triggered) or a new goal is designated. Do not optimize continuously in free space.

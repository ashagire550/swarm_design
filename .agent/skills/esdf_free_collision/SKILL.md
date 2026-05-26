---
name: "ESDF-Free Collision Avoidance"
description: "Implementation guidelines for discarding voxel-grids and using gradient-based anchor and vector strategies for collision avoidance."
---

# ESDF-Free Collision Avoidance

To break free from grid mapping bottlenecks, implement an event-triggered, vector-based collision avoidance mechanism.

## Core Concept
1. Generate an initial path from a lightweight graph search strategy (like A* on a sparse pointcloud).
2. Represent the drone's path as B-spline control points $Q$.
3. Check interactions between the pointcloud (or depth obstacles) and $Q$.
4. If an obstacle boundary overlaps $Q_i$, generate an obstacle-avoiding anchor ($p$) and normal direction ($v$).

## Generation of {p, v} Pairs

If a line segment intersecting control points hits an obstacle:
*   **Anchor Point (p):** The closest valid free-space surface point on the geometric boundary.
*   **Normal Vector (v):** A normalized unit vector pointing from the obstacle surface outward into free space.

*Implementation Note:* For a given control point inside an obstacle, search radially outward along the initial path's normal vectors until clear space is found. This surface limit dictates $p$, and the search direction is $v$.

## Translating into Cost & Gradient

### The Collision Penalty $J_c$
Since we want the control point $Q_i$ to lie outside the obstacle boundary (in the direction of $v$), we define the penalty distance $d_i$:
$d_i = (Q_i - p)^T \cdot v$

If $d_i < 0$ (the point is "behind" the free-space plane, i.e., inside the obstacle):
$J_c += \sum (d_i)^3$ (Use cubic or quadratic penalization for scale).

If $d_i \ge 0$, penalty = 0.

### The Gradient $\nabla J_c$
Since $\frac{\partial d_i}{\partial Q_i} = v$:
$\nabla J_c = 3 \cdot d_i^2 \cdot v$ (for cubic cost).

This directly yields the force vector pushing the control point out of the obstacle.

## Advantages over ESDF
*   You do not need to process global map regions where the drone is not flying.
*   You do not need to evaluate raycasting and signed distance across millions of points.
*   Only the control points currently intersecting the obstacles trigger the $O(1)$ penalty calculation.

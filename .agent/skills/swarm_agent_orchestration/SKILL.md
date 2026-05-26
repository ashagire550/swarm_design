---
name: "Swarm AI Orchestration & Network"
description: "Guidelines and architecture for multi-agent negotiation, collision avoidance, and network sharing."
---

# Swarm AI Orchestration & Network

The swarm intelligence operates in a decentralized manner. Centralizing computation ruins scalability. Instead, rely on local, ego-centric planners collaborating via shared state.

## Shared State Architecture
Each drone periodically broadcasts an uncompressed minimal packet via a Low-Latency network (UDP or custom MAVLink). The packet must contain:
1.  **Drone ID**
2.  **Timestamp ($t_0$)**
3.  **Current Odom** (Pos, Vel, Accel)
4.  **Predicted Trajectory** (A lightweight list of control points or coefficients of its current B-Spline plan).

## Inter-Drone Collision Avoidance

To prevent a drone turning into another drone's path, implement predictive obstacle avoidance.
1. When Drone A receives Drone B's control points, Drone A functionally evaluates Drone B's position at corresponding future timestamps $t$.
2. Drone A treats Drone B as an expanding spherical dynamic obstacle.
3. If Drone A's future trajectory path intersects Drone B's predicted path concurrently, generate a $\{p, v\}$ vector set pushing Drone A's trajectory away from B.

## Deadlock & Priority Negotiation

**Avoid Symmetric Avoidance:**
If two drones fly straight at one another, their identical avoidance algorithms may attempt identically symmetrical dodges (both dodge left, still crashing).
*   **Resolution:** Implement an ID-based or altitude-based yielding hierarchy. For example, drone with a lower ID has "right of way" and ignores Drone B. Drone B assumes Drone A's trajectory is immutable and executes the total displacement maneuver.

**Dead-Reckoning During Comm Limits:**
If comms drop out, drones lock the last received B-spline of their peers. Because B-splines represent continuous intent (not just current velocity), a drone can reliably predict its neighbor's path for several seconds forward without new telemetry.

## Swarm Agent Goals
The higher-level logic (Task / Swarm Agent) injects "Goal" waypoints into the Local Planner to influence behavior without modifying the collision loop:
*   **Flocking / Aggregation:** Swarm Agent calculates the center-of-mass of peers and sets intermediary goal points to pull drones together.
*   **Separation:** Inject static repulsive gradient centers to maintain geometric formations (e.g., V-shapes, line-abreast).

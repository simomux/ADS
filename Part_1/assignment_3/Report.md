# Particle Filter Localization — Report

This report analyzes a Particle Filter implementation for localizing a forklift in a known map using 9 LiDAR reflectors. The filter runs on a recorded ROS bag and estimates the vehicle's 2D pose (x, y, θ).

Each scenario reports:

- Configuration (N particles, initialization mode, noise parameters)
- Estimated trajectory vs. Hipert reference (`pf_slam.txt`)

---

## Scenario 1: Guided Initialization, N = 200

### S1 - Configuration

| Parameter | Value |
| --- | --- |
| Initialization | GPS prior at (2, 1, -1) |
| N particles | 200 |
| σ_init (x, y, θ) | 0.5 m, 0.5 m, 0.1 rad |
| σ_pos (x, y, θ) | 0.05 m, 0.05 m, 0.05 rad |
| σ_landmark (x, y) | 0.4 m, 0.4 m |
| Data association | Nearest Neighbor |
| Resampling | Resampling wheel |

![Scenario1](./results/Scenario1/Scenario1.1.png)

![Scenario1](./results/Scenario1/Scenario1.1_overlap.png)

### S1 - Trajectory

The filter successfully tracks the forklift throughout the entire simulation (blue), closely matching the reference solution (green). A small cluster of outlier points appears near the center of the map, corresponding to frames where the best particle momentarily jumped to a nearby landmark due to ambiguous observations.

The estimated trajectory closely follows the reference, with an overall RMSE of 0.491 m and a mean error of 0.182 m. The RMSE is dominated by a brief spike during the first ~50 frames (initialization phase); once converged, the per-frame error consistently stays below 0.3 m.

Tests have been made also with 1000 particles in the same conditions, but no considerable change was noticed (RMSE 0.492 m, mean 0.161 m - virtually identical). Images and data are still available in the results dir as `Scenario1.2`.

---

## Scenario 2 - Random Initialization

### S2 - Configuration

All sub-scenarios share the same parameters except N:

| Parameter | Value |
| --- | --- |
| Initialization | Randomly uniform over map bounds |
| σ_init (x, y, θ) | 0.5 m, 0.5 m, 0.1 rad |
| σ_pos (x, y, θ) | 0.05 m, 0.05 m, 0.05 rad |
| σ_landmark (x, y) | 0.4 m, 0.4 m |
| Data association | Nearest Neighbor |
| Resampling | Resampling wheel |

### S2.1 - N = 200 (baseline)

![Scenario 2.1](./results/Scenario2/Scenario2.1.png)

![Scenario 2.1 overlap](./results/Scenario2/Scenario2.1_overlap.png)

The filter fails to converge with 200 particles. With ~500 m² of map area and a 3D pose space (x, y, θ), the effective particle density is ~0.03 particles/unit — making it very unlikely that any particle starts near the true pose (2, 1, −1). RMSE: 6.849 m, mean: 6.632 m.

### S2.2 - N = 1000 (best result)

![Scenario 2.2](./results/Scenario2/Scenario2.2.png)

![Scenario 2.2 overlap](./results/Scenario2/Scenario2.2_overlap.png)

Increasing to 1000 particles produces a clear improvement. The filter converges to the correct region after ~400 frames and then closely tracks the reference trajectory, with per-frame error dropping to well below 0.5 m for the majority of the run. Overall RMSE: 2.011 m, mean: 0.880 m - both dominated by the long convergence phase at the start.

### S2 - Trajectory analysis

Further experiments with N = 2500 (S2.3) and N = 5000 (S2.4) were conducted but both diverged, with RMSE of 11.3 m and 18.6 m respectively. Plots are available in `results/Scenario2/` for reference.

The failure at higher N is not purely algorithmic: each particle adds computation time, and since `delta_t` in the prediction step is measured as real wall-clock time, excessive processing latency causes the bicycle model to predict large position jumps, triggering filter divergence. This creates a hard computational budget constraint for this implementation.

The non-monotone behaviour (N=1000 converges, N=2500 does not) also reflects the stochastic nature of random initialization - whether the filter converges depends on whether at least one particle happens to start near the true pose, which is not guaranteed for any fixed N.

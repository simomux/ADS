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

Increasing to 1000 particles produces a clear improvement. The filter converges to the correct region after ~250 frames and then closely tracks the reference trajectory, with per-frame error dropping to well below 0.5 m for the majority of the run. Overall RMSE: 2.011 m, mean: 0.880 m - both dominated by the long convergence phase at the start.

### S2 - Trajectory analysis

Further experiments with N = 2500 (S2.3) and N = 5000 (S2.4) were conducted but both diverged, with RMSE of 11.3 m and 18.6 m respectively. Plots are available in `results/Scenario2/` for reference.

The failure at higher N is not purely algorithmic: each particle adds computation time, and since `delta_t` in the prediction step is measured as real wall-clock time, excessive processing latency causes the bicycle model to predict large position jumps, triggering filter divergence. This creates a hard computational budget constraint for this implementation.

The non-monotone behaviour (N=1000 converges, N=2500 does not) also reflects the stochastic nature of random initialization - whether the filter converges depends on whether at least one particle happens to start near the true pose, which is not guaranteed for any fixed N.

---

## Scenario 3 - Mahalanobis Data Association, Guided Init, N = 200

### S3 - Configuration

| Parameter | Value |
| --- | --- |
| Initialization | GPS prior at (2, 1, -1) |
| N particles | 200 |
| σ_init (x, y, θ) | 0.5 m, 0.5 m, 0.1 rad |
| σ_pos (x, y, θ) | 0.05 m, 0.05 m, 0.05 rad |
| σ_landmark (x, y) | 0.4 m, 0.4 m |
| Data association | Mahalanobis + chi-squared gating (99%, warm-up 100 frames) |
| Resampling | Resampling wheel |

The data association is replaced with Mahalanobis distance with chi-squared gating at the 99% confidence level (threshold 9.210).

An observation is associated to the nearest landmark only if its Mahalanobis distance falls within the gate; otherwise it is rejected and the particle receives a boundary-level weight penalty (`exp(-9.21/2) / (2π σ²)`).

A warm-up phase of 100 frames uses plain Nearest Neighbor to allow the filter to converge before gating is enforced. Number of frames was determined just from trial and error.

### S3.1 - Without warm-up (failed attempt)

![Scenario 3.1](./results/Scenario3/Scenario3.1.png)

![Scenario 3.1 overlap](./results/Scenario3/Scenario3.1_overlap.png)

Without the warm-up phase, the gate immediately rejects most observations during the first frames, with particles falling outside the ~1.2 m gate radius. All weights collapse and the filter diverges (RMSE: 8.563 m).

### S3.2 - With warm-up (best result)

![Scenario 3.2](./results/Scenario3/Scenario3.2.png)

![Scenario 3.2 overlap](./results/Scenario3/Scenario3.2_overlap.png)

With the 100-frame warm-up the filter converges normally and then Mahalanobis gating takes over.

Results slighly improve over the NN baseline: RMSE 0.482 m (vs 0.491 m), mean error 0.163 m (vs 0.182 m). The improvement is most visible in the right-hand curve of the trajectory (y ≈ 10–12 m in the overlap plot).

---

## Scenario 4 - Mahalanobis Data Association, Random Init, N = 1000

Same configuration as S2.2 (random init, N=1000) but with Mahalanobis data association and a 250-frame warm-up, matching the convergence time observed in S2.2.

Two warm-up durations were tested to find when Mahalanobis should activate relative to the convergence point (~250 frames).

### S4.1 - warm-up = 400 frames (Mahalanobis never activates)

![Scenario 4.1](./results/Scenario4/Scenario4.1.png)

![Scenario 4.1 overlap](./results/Scenario4/Scenario4.1_overlap.png)

RMSE 2.036 m, mean 0.888 m, being marginally worse than plain NN (S2.2). Since the warm-up exceeds the convergence time, Mahalanobis never activates; this run is effectively identical to S2.2 and confirms the baseline.

### S4.2 - warm-up = 250 frames (Mahalanobis activates post-convergence)

![Scenario 4.2](./results/Scenario4/Scenario4.2.png)

![Scenario 4.2 overlap](./results/Scenario4/Scenario4.2_overlap.png)

RMSE 2.004 m, mean 0.858 m. This is the best result for random initialization. By aligning the warm-up cutoff with the actual convergence point, Mahalanobis takes over exactly when the particles are already near the true pose and gating can be effective.

The improvement over NN is consistent with Scenario 3: once converged, stricter data association very subtly improves tracking quality.

The main takeaway is that Mahalanobis gating requires the filter to be already converged to be beneficial.

---

## Scenario 5 - Systematic Resampling, Guided Init, N = 200

Systematic resampling replaces the resampling wheel. Instead of N independent random draws, a single value `u ~ Uniform(0, 1/N)` is drawn and N equally-spaced points `u, u+1/N, ..., u+(N-1)/N` are placed on the normalized cumulative weight distribution. This guarantees that a particle with weight `w_i` is selected at least `floor(N·w_i)` times, reducing resampling variance from O(1/N) to O(1/N²).

### S5.1 - NN + Systematic resampling

| Parameter | Value |
| --- | --- |
| Initialization | GPS prior at (2, 1, -1) |
| N particles | 200 |
| Data association | Nearest Neighbor |
| Resampling | Systematic |

![Scenario 5.1](./results/Scenario5/Scenario5.1.png)

![Scenario 5.1 overlap](./results/Scenario5/Scenario5.1_overlap.png)

RMSE 0.493 m, mean 0.182 m, basically identical to the wheel baseline (S1.1: 0.491 m, 0.182 m). With N=200 and guided initialization, the resampling wheel already has low variance and systematic resampling offers no measurable improvement on a single run.

### S5.2 - Mahalanobis + Systematic resampling (better one)

| Parameter | Value |
| --- | --- |
| Initialization | GPS prior at (2, 1, -1) |
| N particles | 200 |
| Data association | Mahalanobis + chi-squared gating (99%, warm-up 100 frames) |
| Resampling | Systematic |

![Scenario 5.2](./results/Scenario5/Scenario5.2.png)

![Scenario 5.2 overlap](./results/Scenario5/Scenario5.2_overlap.png)

RMSE 0.478 m, mean 0.165 m. This is the best result across all guided-init scenarios. Combining both improvements yields a small additive gain over Mahalanobis alone.

| Scenario | Data association | Resampling | RMSE | Mean |
| --- | --- | --- | --- | --- |
| S1.1 | Nearest Neighbor | Wheel | 0.491 m | 0.182 m |
| S3.2 | Mahalanobis | Wheel | 0.482 m | 0.163 m |
| S5.1 | Nearest Neighbor | Systematic | 0.493 m | 0.182 m |
| S5.2 | Mahalanobis | Systematic | **0.478 m** | **0.165 m** |

The dominant contribution is Mahalanobis data association. Systematic resampling adds marginal improvement on top.

However, comparing the overlap plots visually, S3.2 (Mahalanobis + wheel) produces a smoother, cleaner trajectory than S5.2, despite its slightly worse RMSE.

The cause of this jitter is structural: after convergence, all particles are tightly clustered near the true pose with similar weights. Systematic resampling's near-deterministic selection rotates through these near-equal-weight particles in a quasi round-robin fashion, causing the best particle to switch frequently → frame-to-frame jitter. The resampling wheel's stochastic draws instead tend to keep the same high-weight particle for longer stretches, producing a smoother trajectory.

The paradox is that S5.2 still achieves better RMSE: even though it is jitterier on average, the wheel occasionally draws a genuinely low-weight particle as best particle, causing large isolated error spikes that inflate the squared error term in RMSE disproportionately. Systematic eliminates those spikes entirely.

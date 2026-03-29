# Assignment 2 - Part 3

AY: 2025-2026

## Exercise 1 - Longitudinal Control

Plot the requested outputs for both velocity conditions and indicate what
are the settling time and eventually the overshoot, as well as other things
you could consider useful.

![Trajectory](./img/Es1/Speed1/Trajectory.png)

As you can see from the graph above in Ex.1 the vehicle is only capable of going straight since lateral control is disabled. The trajectory computed by the vehicle is of course the same for both testcases at 15 and 25m/s.

The PID gains were set for both test cases as follows after trial and error:

```python
kp=1, ki=0.25, kd=0.01
```

### Speed = 15m/s

![Longitudinal Velocity](./img/Es1/Speed1/LongitudinalVelocity.png)

At 15m/s you can notice how in the first 50 timesteps (2.5s) the vehicle reaches 14m/s and takes the next 150 (7.5s) timesteps to reach target speed slowly and controlled without overshooting. After 200 timesteps (10s) it settles at target speed and maintains it continuously.

![Velocity Error](./img/Es1/Speed1/VelocityError.png)

You can see also how the initial velocity error starts at 5m/s until it converges to 0.

![Longitudinal Accelleration](./img/Es1/Speed1/LongitudinalAccelleration.png)

You can notice how the initial acceleration is constant at 2m/s^2 for the first ~20 timesteps and then decreases until reaching the minimum required to counter drag and rolling resistances (~0.23m/s^2).

### Speed = 25m/s

![Longitudinal Velocity](./img/Es1/Speed2/LongitudinalVelocity.png)

At 25m/s you can notice that target speed is reached after ~300 timesteps (15s) having a rapid increase for the first ~150 timestep (7,5s), then slower approaching the desired speed for the remaining ~150 timesteps (7.5s).

![Velocity Error](./img/Es1/Speed2/VelocityError.png)

Velocity error starts at 15m/s and converges to 0 slightly after target speed is reached.

![Longitudinal Accelleration](./img/Es1/Speed2/LongitudinalAcceleration.png)

Acceleration is constant at 2m/s^2 for the first 100 timesteps (5s), and then slowly converging to the minimum required to maintain target speed after ~400 timesteps (20s).

## Exercise 2 - Low-speed Lateral Control

For Exercise 2, the same PID gains were kept for longitudinal control.

### Pure Pursuit

The Pure Pursuit algorithm steers the vehicle toward a look-ahead point on the path at a distance `Lf` ahead of the rear axle. The steering angle is computed as:

```python
delta = atan(2 * L * sin(alpha) / Lf)
```

where `alpha` is the heading error to the look-ahead point and `L` is the wheelbase. The look-ahead distance is speed-dependent:

```python
Lf = k_pp * vx + look_ahead = 0.3 * vx + 1.0
```

The parameter `k_pp` was increased from `0.001` to `0.3` after observing that the original value caused highly aggressive steering corrections every timestep, heavily disturbing the longitudinal velocity through lateral-longitudinal coupling in the nonlinear model. The increased look-ahead smooths the steering response by targeting a point further ahead, reducing unnecessary corrections on straight sections.

#### Speed = 10m/s

![Trajectory](./img/Es2/Speed1/PurePursuit/Trajectory.png)

The vehicle follows the oval path accurately across the full lap, with the simulated trajectory overlapping the reference spline.

![Longitudinal Velocity](./img/Es2/Speed1/PurePursuit/LongitudinalVelocity.png)

The longitudinal velocity settles quickly at 10 m/s. Small periodic dips correspond to the two curves of the oval where lateral forces cause minor perturbations, which the PID corrects rapidly.

![Velocity Error](./img/Es2/Speed1/PurePursuit/VelocityError.png)

The velocity error peaks at 0.12 m/s (1.2% of target speed) at the start and stays well below the 5% threshold (0.5 m/s) throughout the lap.

![Lateral Error](./img/Es2/Speed1/PurePursuit/LateralError.png)

The lateral error remains negligible, peaking at ~0.05m in the corners. Well within the 1m requirement.

#### Speed = 20m/s

![Trajectory](./img/Es2/Speed2/PurePursuit/Trajectory.png)

Path tracking remains accurate at 20 m/s, with the trajectory closely following the reference spline.

![Velocity Error](./img/Es2/Speed2/PurePursuit/VelocityError.png)

At higher speed the velocity perturbations at the curves are more pronounced, with a peak error of ~0.21 m/s (1.05% of target speed). This is still well within the 5% requirement (1.0 m/s).

![Lateral Error](./img/Es2/Speed2/PurePursuit/LateralError.png)

Lateral error increases compared to 10 m/s due to the higher speed reducing the effectiveness of the look-ahead correction, peaking at ~0.20m. Both requirements are satisfied in both speed conditions.

### Stanley

The Stanley controller computes the steering angle using both the heading error and the cross-track error measured at the front axle:

```python
delta = heading_error + atan(k * cross_track_error / vx)
```

The gain `k` scales the cross-track correction and was tuned to `k_stanley = 3.0`. Lower values (e.g. 0.5–1.5) were tested but resulted in lateral errors up to 0.6m at 20 m/s, while `k=3.0` achieved better tracking. Note that because the correction term is divided by `vx`, a higher `k` is needed at higher speeds to maintain the same effective gain.

#### Speed = 10m/s

![Trajectory](./img/Es2/Speed1/Stanley/Trajectory.png)

The vehicle accurately follows the oval path across the full lap.

![Longitudinal Velocity](./img/Es2/Speed1/Stanley/LongitudinalVelocity.png)

Longitudinal velocity behaviour is very similar to Pure Pursuit at 10 m/s, settling quickly with small perturbations at the curves.

![Velocity Error](./img/Es2/Speed1/Stanley/VelocityError.png)

Velocity error peaks at ~0.12 m/s (1.2% of target speed), identical to Pure Pursuit, and converges to near zero on the straights.

![Lateral Error](./img/Es2/Speed1/Stanley/LateralError.png)

Lateral error stays within ±0.08m throughout the lap — slightly larger than Pure Pursuit (±0.05m) but still negligible.

#### Speed = 20m/s

![Trajectory](./img/Es2/Speed2/Stanley/Trajectory.png)

Path tracking remains accurate at 20 m/s.

![Longitudinal Velocity](./img/Es2/Speed2/Stanley/LongitudinalVelocity.png)

The velocity profile is comparable to Pure Pursuit at 20 m/s, with the same characteristic spike (~20.12 m/s) at the tighter curve.

![Velocity Error](./img/Es2/Speed2/Stanley/VelocityError.png)

Velocity error peaks at ~0.20 m/s (1.0% of target speed), within the 5% requirement.

![Lateral Error](./img/Es2/Speed2/Stanley/LateralError.png)

Lateral error peaks at ~0.45m — larger than Pure Pursuit (0.20m) at the same speed. This is expected: Stanley's cross-track correction is attenuated at high speed by the `1/vx` term, halving the cross-correction term when speed doubles, making it less reactive than at low speed. Despite this, the 1m requirement is still met.

## Exercise 3: High-speed Lateral Control

For Exercise 3, high-speed lateral control is implemented using **Model Predictive Control (MPC)** via the CasADi solver, combined with the same PID longitudinal controller from Exercise 1.

### Weight Tuning

Tuning the MPC weights required significant experimentation, particularly at 25 m/s where the kinematic model approximation becomes less accurate due to significant tire slip:

- **Control weight too low** (1000–15000): allowed too-aggressive steering corrections, causing instability or immediate lateral error violations in the curves.
- **Control weight too high + low heading weight** (100000, heading=10): IPOPT could not find a feasible solution at curve entry. The solver was over-constrained on steering while under-incentivized to align with the path heading.
- **Final configuration** (x=150, y=150, heading=20, control=60000): balances path tracking and smooth control. The higher heading weight ensures the vehicle pre-aligns with the curve direction, reducing the entry spike in lateral error.

### Speed = 23 m/s

![Trajectory](./img/Es3/mpc/Speed1/Trajectory.png)

The vehicle tracks the oval path accurately for a full lap at 23 m/s, with the simulated trajectory closely overlapping the reference spline.

![Longitudinal Velocity](./img/Es3/mpc/Speed1/LongitudinalVelocity.png)

Longitudinal velocity stays close to 23 m/s throughout, with small perturbations at the curves due to lateral-longitudinal coupling in the nonlinear model.

![Velocity Error](./img/Es3/mpc/Speed1/VelocityError.png)

Peak velocity error is ~0.35 m/s (1.5% of target speed), well within the 5% requirement (1.15 m/s).

![Lateral Error](./img/Es3/mpc/Speed1/LateralError.png)

Lateral error peaks at ~0.60m in the curves, comfortably within the 1m requirement.

![Lateral Velocity](./img/Es3/mpc/Speed1/LateralVelocity.png)

Lateral velocity reaches up to ~1.5 m/s in the curves, consistent with the vehicle dynamics at this speed.

### Speed = 25 m/s

At 25 m/s the vehicle operates near the limits of the kinematic bicycle model used internally by the MPC. The mismatch between the kinematic prediction and the actual nonlinear dynamics causes oscillatory steering corrections in the curves, which is the dominant behaviour visible in the next plots.

![Trajectory](./img/Es3/mpc/Speed2/Trajectory.png)

The vehicle completes the full lap at 25 m/s with the trajectory closely following the reference spline.

![Longitudinal Velocity](./img/Es3/mpc/Speed2/LongitudinalVelocity.png)

Longitudinal velocity oscillates between ~24.4 and ~25.4 m/s. The oscillations are larger than at 23 m/s due to stronger lateral forces in the curves coupling back into the longitudinal dynamics.

![Velocity Error](./img/Es3/mpc/Speed2/VelocityError.png)

Peak velocity error is ~0.63 m/s (2.5% of target speed), within the 5% requirement (1.25 m/s).

![Lateral Error](./img/Es3/mpc/Speed2/LateralError.png)

Lateral error peaks at ~0.93m at the first curve entry and ~0.95m in the second curve. The oscillatory pattern visible throughout the curves is characteristic of the MPC compensating for the growing mismatch between the kinematic internal model and the actual nonlinear dynamics at high speed.

![Lateral Velocity](./img/Es3/mpc/Speed2/LateralVelocity.png)

Lateral velocity reaches up to ~2.5 m/s in the curves, significantly higher than at 23 m/s, indicating substantial tire slip consistent with operation near the nonlinear tire force saturation region.

![Front Slip Angle](./img/Es3/mpc/Speed2/FrontSlipAngle.png)

Front slip angle peaks at ~0.12 rad and shows clear oscillations in both curves, reflecting the MPC repeatedly correcting the heading mismatch.

Both speed conditions satisfy the requirements without relaxing the constraints. 

The oscillatory behaviour at 25 m/s is a structural limitation of the mismatch between the **kinematic** internal model used by the MPC and the **nonlinear dynamic** model used in simulation. The MPC internal model assumes zero lateral velocity and derives yaw rate purely from geometry, with no independent tire slip dynamics. At high speed, significant tire slip develops and the real vehicle exhibits non-negligible lateral velocity and yaw rate which the MPC cannot predict, causing the oscillatory corrections visible in the curves.

Implementing even a simple dynamic model and substituing it to the kinetic one would probably achieve less jerky behaviour through cornerns and higher speeds, since over 25m/s the kinematic model cannot maintain the 1m lateral error constraint.

### Bonus: Pure Pursuit at High Speed

Pure Pursuit was also tested at 23 m/s and 25 m/s as a comparison to MPC. Being a purely geometric controller with no internal vehicle model, it is inherently free from model mismatch. It computes the steering angle solely from the geometry between the vehicle and the look-ahead point, regardless of speed.

#### PP at 23 m/s

![Trajectory](./img/Es3/pp/Speed1/Trajectory.png)

The vehicle tracks the oval path accurately for a full lap.

![Velocity Error](./img/Es3/pp/Speed1/VelocityError.png)

Peak velocity error is ~0.32 m/s (1.4% of target speed).

![Lateral Error](./img/Es3/pp/Speed1/LateralError.png)

Lateral error peaks at ~0.42m, acting significantly better than MPC (0.60m) at the same speed, with no oscillatory corrections.

#### PP at 25 m/s

![Trajectory](./img/Es3/pp/Speed2/Trajectory.png)

The vehicle completes the full lap at 25 m/s, tracking the reference spline closely.

![Velocity Error](./img/Es3/pp/Speed2/VelocityError.png)

Peak velocity error is ~0.60 m/s (2.4% of target speed).

![Lateral Error](./img/Es3/pp/Speed2/LateralError.png)

Lateral error peaks at ~0.65m, being well below the 1m requirement and considerably better than MPC (0.95m) at the same speed. The tracking is smooth with no oscillatory behaviour.

Pure Pursuit outperforms MPC at both speeds, achieving ~30% lower lateral error at 23 m/s and ~32% lower at 25 m/s, thanks to having no internal model assumption. From 27 m/s on, this tuned version of PP becomes inefficient, failing to maintain the 1m lateral error constraint as soon as it enters the initial hairpin.

### Bonus: Stanley at High Speed

Stanley was also tested at 23 m/s and 25 m/s. Recall that Stanley computes the steering angle as:

```python
delta = heading_error + atan(k * cross_track_error / vx)
```

The cross-track correction term is divided by `vx`, making it progressively less effective as speed increases. This is the same limitation observed in Exercise 2 at 20 m/s, now amplified at 23–25 m/s.

#### Stanley at 23 m/s

![Trajectory](./img/Es3/stanley/Speed1/Trajectory.png)

The vehicle completes the full lap at 23 m/s.

![Velocity Error](./img/Es3/stanley/Speed1/VelocityError.png)

Peak velocity error is ~0.42 m/s (1.8% of target speed).

![Lateral Error](./img/Es3/stanley/Speed1/LateralError.png)

Lateral error peaks at ~0.70m, worse than both PP (0.42m) and MPC (0.60m) at the same speed.

#### Stanley at 25 m/s

![Trajectory](./img/Es3/stanley/Speed2/Trajectory.png)

The vehicle completes the full lap at 25 m/s.

![Velocity Error](./img/Es3/stanley/Speed2/VelocityError.png)

Peak velocity error is ~0.80 m/s (3.2% of target speed).

![Lateral Error](./img/Es3/stanley/Speed2/LateralError.png)

Lateral error peaks at ~1.0m, right at the boundary of the requirement. This is the worst result among all three algorithms at 25 m/s, and noteworthy less accurate behavuoir of stanley algorithm in faster simulations.

Stanley degrades significantly at high speed due to the `1/vx` term attenuating the cross-track correction, acting worse than MPC with kinetic model and pp, demonstrating how this solution is better for low speed scenarios rather than higher ones.

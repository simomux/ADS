# Assignment 5

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

import sys
sys.path.insert(0, '../Assignment_2')

from matplotlib.patches import Circle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
import time
from simulation import Simulation
import pid
import purepursuit
import stanley
from mpc import *
import cubic_spline_planner
import math
import frenet_optimal_trajectory as fp
matplotlib.use('TkAgg')  # Or 'Agg', 'Qt5Agg', etc.

# Simulation parameters
dt = 0.05         # Time step (s)
ax = 0.0            # Constant longitudinal acceleration (m/s^2)
steer = 0.0      # Constant steering angle (rad)
sim_time = 2000      # Simulation duration in seconds
steps = int(sim_time / dt)  # Simulation steps (30 seconds)

# Control references
target_speed = 28.0

# Vehicle parameters
lf = 1.156          # Distance from COG to front axle (m)
lr = 1.42           # Distance from COG to rear axle (m)
wheelbase = lf + lr
mass = 1200         # Vehicle mass (kg)
Iz = 1792           # Yaw moment of inertia (kg*m^2)
max_steer = 3.14  # Maximum steering angle in radians

# Create instance of PID for Longitudinal Control
long_control_pid = pid.PIDController(kp=1, ki=0.25, kd=0.01, output_limits=(-2, 2))

# Create instance of PurePursuit, Stanley and MPC for Lateral Control
k_pp = 0.3   # Speed proportional gain for Pure Pursuit
look_ahead = 1.0  # Minimum look-ahead distance for Pure Pursuit
k_stanley = 3  # Gain for cross-track error for Stanley
pp_controller = purepursuit.PurePursuitController(wheelbase, max_steer)
stanley_controller = stanley.StanleyController(k_stanley, lf, max_steer)

#Planning
# obstacle lists
ob = np.array([[100.0, -0.5],
                [400.0, 0.5],
                [570.0, 29.0],
                [600.0,100.0],
                [120.0, 200.0],
                [33.0, 200.0],
                [-70.0, 171.0],
                [-100.0, 100.0]
                ])

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

# Load path and create a spline
xs, ys = load_path("../Assignment_2/oval_trj.txt")
path_spline = cubic_spline_planner.Spline2D(xs, ys)

def point_transform(trg, pose, yaw):

    local_trg = [trg[0] - pose[0], trg[1] - pose[1]]

    return local_trg

def plot_comparison(results, labels, title, xlabel, ylabel):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_trajectory(x_vals, y_vals, labels, path_spline, frenet_x_results, frenet_y_results):
    """ Plot 2D trajectory (x vs y) for all simulation configurations and path_spline trajectory. """
    plt.figure(figsize=(10, 6))
    
    # Plot the simulation trajectories
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])

    # Plot the frenet planner trajectory
    for i in range(len(frenet_x_results)):
        plt.plot(frenet_x_results[i], frenet_y_results[i], linestyle="--", color="green")

    # Plot the path_spline trajectory
    spline_x = [path_spline.calc_position(s)[0] for s in np.linspace(0, path_spline.s[-1], 1000)]
    spline_y = [path_spline.calc_position(s)[1] for s in np.linspace(0, path_spline.s[-1], 1000)]
    plt.plot(spline_x, spline_y, label="Path Spline", linestyle="--", color="red")

    # Plot obstacles
    if(len(ob[0]) != 0):
        plt.scatter(ob[:, 0], ob[:, 1], c='black', label="Obstacles", marker='x')

    for obstacle in ob:
        ax = plt.gca()
        ax.add_patch(
            Circle(obstacle, fp.ROBOT_RADIUS, facecolor='None', alpha=0.5, linewidth=1.5, edgecolor='black')
        )
    
    # Customize plot
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def run_simulation(ax, steer, dt, integrator, model, steps=500):
    """ Run a simulation with the given parameters and return all states. """

    # Initialize the simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model, vx0=target_speed)

    # Storage for state variables and slip angles
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals = [], []  # Slip angles
    frenet_x, frenet_y = [], []
    ax_vals, vx_error_vals = [], []  # Longitudinal acceleration and velocity error
    lat_error_vals = []       # Lateral error w.r.t. Frenet path
    track_lat_error_vals = [] # Lateral error w.r.t. global path spline
    frenet_times = []         # Frenet planner execution time per step

    lap_started = False   # True once the vehicle has moved away from start


    # casadi_model()
    casadi_dynamic_model()

    # states for Frenet-planner
    c_speed = target_speed  # current speed [m/s]
    c_accel = 0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    fp.TARGET_SPEED = target_speed
    fp.MAX_SPEED = target_speed

    fp.SIM_LOOP = sim_time

    # DT=0.05 → path points ~0.5m apart at 10m/s → precise collision check
    fp.DT = 0.05

    # D_ROAD_W=0.8: np.arange(-4, 4, 0.8) = [-4.0, -3.2, -2.4, ...] never
    # samples d=-3.0 exactly, so the planner always keeps at least 3.2+0.5=3.7m
    # from obs at (400, 0.5). Stanley tracking error (~0.08m) leaves 3.6m clearance.
    fp.D_ROAD_W = 0.8

    # Narrow time horizon: fewer candidate paths → faster planner.
    # np.arange(2.5, 3.0, 0.05) = 6 horizons vs np.arange(4.5, 5.0, 0.05) = 10.
    # At 10m/s a 3s horizon covers 30m ahead — sufficient for static obstacles.
    fp.MIN_T = 2.8
    fp.MAX_T = 2.85

    # Reduce road width to ±4m: narrower scan, fewer candidate paths, faster planner.
    fp.MAX_ROAD_WIDTH = 4.0

    fp.K_J = 0.1
    fp.K_T = 0.1
    fp.K_D = 1.0
    fp.K_LAT = 1.0
    fp.K_LON = 1.0

    for step in range(steps):
    
        # Print time
        print("Time:", step*dt)

        # Calculate ax to track speed
        ax = long_control_pid.compute(target_speed, sim.vx, dt)

        ############# Frenet-planner

        t_frenet_start = time.time()
        frenet_path = fp.frenet_optimal_planning(
            path_spline, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        frenet_times.append(time.time() - t_frenet_start)

        if(frenet_path is None):
            print("\nNone available paths found from Frenet...")
            break

        frenetpath_spline = cubic_spline_planner.Spline2D(frenet_path.x, frenet_path.y)

        # Update actual frenet-frame position in the spline
        # aka longitudinal position and actual lateral error
        actual_position = sim.x, sim.y
        actual_pose = sim.x, sim.y, sim.theta
        path_spline.update_current_s(actual_position)
        frenetpath_spline.update_current_s(actual_position)

        # get actual position projected on the path/spline
        global_position_projected = path_spline.calc_position(path_spline.cur_s)
        prj = [ global_position_projected[0], global_position_projected[1] ]
        local_error = point_transform(prj, actual_position, sim.theta)

        local_position_projected = frenetpath_spline.calc_position(frenetpath_spline.cur_s)
        prj = [ local_position_projected[0], local_position_projected[1] ]
        frenetlocal_error = point_transform(prj, actual_position, sim.theta)

        nearest_idx = 0
        nearest_distance = abs(path_spline.cur_s - frenet_path.s[0])
        for i in range(len(frenet_path.s)):
            dist = abs(path_spline.cur_s - frenet_path.s[i])
            if(dist < nearest_distance):
                nearest_distance = dist
                nearest_idx = i

        s0 = frenet_path.s[nearest_idx]
        c_d = frenet_path.d[nearest_idx]
        c_d_d = frenet_path.d_d[nearest_idx]
        c_d_dd = frenet_path.d_dd[nearest_idx]
        c_speed = frenet_path.s_d[nearest_idx]
        c_accel = frenet_path.s_dd[nearest_idx]

        ################

        if(abs(local_error[1]) > 4.0):
            print("\nLateral error is higher than 4.0... ending the simulation")
            print("Lateral error: ", local_error[1])
            break

        # Check that the vehicle respects the 3m obstacle clearance
        collision = any(math.hypot(sim.x - obs[0], sim.y - obs[1]) < fp.ROBOT_RADIUS for obs in ob)
        if collision:
            min_dist = min(math.hypot(sim.x - obs[0], sim.y - obs[1]) for obs in ob)
            print(f"\nCollision! Vehicle at ({sim.x:.2f}, {sim.y:.2f}), closest obstacle {min_dist:.2f}m away (limit: {fp.ROBOT_RADIUS}m)")
            break

        # get target pose
        Lf = k_pp * sim.vx + look_ahead #Bonus: include here the curvature dependency
        s_pos = frenetpath_spline.cur_s + Lf # TO-DO: Extend it to depend on curvature and/or speed

        trg = frenetpath_spline.calc_position(s_pos)
        trg = [ trg[0], trg[1] ]
        pp_position = actual_position
        # Adjust CoG position to the rear axle position for PP
        pp_position = actual_position[0] + lr * math.cos(sim.theta), actual_position[1] + lr * math.sin(sim.theta)
        loc_trg = point_transform(trg, pp_position, sim.theta)

        # Calculate steer to track path
        
        ####### Pure Pursuit
        # steer = 0
        # Compute the look-ahead distance
        # steer = pp_controller.compute_steering_angle(loc_trg, sim.theta, Lf)
        
        ###### Stanley
        #TO-DO: Move actual position (CoG) to the front axle for stanley
        # Adjust CoG position to the front axle position
        # px_front = local_position_projected[0] + lf * math.cos(sim.theta)
        # py_front = local_position_projected[1] + lf * math.sin(sim.theta)
        # stanley_target = px_front, py_front, frenetpath_spline.calc_yaw(frenetpath_spline.cur_s)
        # steer = stanley_controller.compute_steering_angle(actual_pose, stanley_target, sim.vx)

        ###### MPC (disabled for Exercise 1, Stanley used instead)
        # get future horizon targets pose (use N_dyn steps matching the dynamic MPC horizon)
        targets = [ ]
        s_pos = frenetpath_spline.cur_s
        for i in range(N_dyn):
            step_increment = (sim.vx)*dt
            trg = frenetpath_spline.calc_position(s_pos)
            t_yaw = frenetpath_spline.calc_yaw(s_pos)
            trg = [ trg[0], trg[1], t_yaw ]
            targets.append(trg)
            s_pos += step_increment
        steer = float(opt_step_dynamic(targets, sim, ax))


        # Make one step simulation via model integration
        sim.integrate(ax, float(steer))
        
        # Append each state to corresponding list
        x_vals.append(sim.x)
        y_vals.append(sim.y)
        theta_vals.append(sim.theta)
        vx_vals.append(sim.vx)
        vy_vals.append(sim.vy)
        r_vals.append(sim.r)

        # Calculate slip angles for front and rear tires
        alpha_f = steer - np.arctan((sim.vy + sim.l_f * sim.r) / max(0.5, sim.vx))  # Front tire slip angle
        alpha_r = -np.arctan((sim.vy - sim.l_r * sim.r) / max(0.5, sim.vx))          # Rear tire slip angle

        alpha_f_vals.append(alpha_f)
        alpha_r_vals.append(alpha_r)

        frenet_x.append(frenet_path.x[0])
        frenet_y.append(frenet_path.y[0])

        ax_vals.append(ax)
        vx_error_vals.append(target_speed - sim.vx)
        lat_error_vals.append(frenetlocal_error[1])   # lateral error w.r.t. Frenet path (as required)
        track_lat_error_vals.append(local_error[1])   # lateral error w.r.t. global path spline

        # Stop simulation after one full lap when veichle is within 5 meters from the start.
        if path_spline.cur_s > path_spline.s[-1] * 0.1:
            lap_started = True
        if lap_started and path_spline.cur_s < 5.0:
            print("\nLap completed! s =", path_spline.cur_s)
            break

    # Frenet planner timing summary
    if frenet_times:
        print(f"\n[Frenet timing] steps: {len(frenet_times)} | "
              f"avg: {np.mean(frenet_times)*1000:.2f} ms | "
              f"total: {np.sum(frenet_times):.2f} s")

    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals, frenet_x, frenet_y, ax_vals, vx_error_vals, lat_error_vals, track_lat_error_vals

def main():

    # List of configurations
    configs = [
        ("rk4", "nonlinear"),
    ]

    # Run each simulation and store the results
    all_results = []
    actual_state = []
    labels = []
    for integrator, model in configs:
        actual_state = run_simulation(ax, steer, dt, integrator, model, steps)
        all_results.append(actual_state)
        labels.append(f"{integrator.capitalize()} - {model.capitalize()}")

    # Separate each state for plotting
    x_results = [result[0] for result in all_results]
    y_results = [result[1] for result in all_results]
    theta_results = [result[2] for result in all_results]
    vx_results = [result[3] for result in all_results]
    vy_results = [result[4] for result in all_results]
    r_results = [result[5] for result in all_results]
    alpha_f_results = [result[6] for result in all_results]
    alpha_r_results = [result[7] for result in all_results]
    frenet_x_results = [result[8] for result in all_results]
    frenet_y_results = [result[9] for result in all_results]
    ax_results = [result[10] for result in all_results]
    vx_error_results = [result[11] for result in all_results]
    lat_error_results = [result[12] for result in all_results]
    track_lat_error_results = [result[13] for result in all_results]

    # Plot comparisons for each state variable
    plot_trajectory(x_results, y_results, labels, path_spline, frenet_x_results, frenet_y_results)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)")
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_f_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_r_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")
    plot_comparison(ax_results, labels, "Longitudinal Acceleration", "Time Step", "Acceleration (m/s^2)")
    plot_comparison(vx_error_results, labels, "Velocity Error", "Time Step", "Velocity Error (m/s)")
    plot_comparison(lat_error_results, labels, "Lateral Error (w.r.t. Frenet path)", "Time Step", "Lateral Error (m)")
    plot_comparison(track_lat_error_results, labels, "Lateral Error (w.r.t. global path)", "Time Step", "Lateral Error (m)")


if __name__ == "__main__":
    main()

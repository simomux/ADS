from cubic_spline_planner import *
from casadi import *
from casadi.tools import *

# Kinematic MPC horizon (used by casadi_model / opt_step)
T =  1.0 # Horizon length in seconds
dt = 0.05 # Horizon timesteps
N = int(T/dt) # Horizon total points

# Dynamic MPC horizon (used by casadi_dynamic_model / opt_step_dynamic)
T_dyn  = 1.0
dt_dyn = 0.05
N_dyn  = int(T_dyn / dt_dyn)

max_steer = 3.14  # Maximum steering angle in radians
min_steer = -3.14  # Minimum steering angle in radians

solver_dyn = None   # built once in casadi_dynamic_model()
prev_u_dyn = None   # warm-start: optimal solution from previous step


def casadi_dynamic_model():
    global F_dyn, solver_dyn

    u  = MX.sym("u", 1)   # steer
    ax = MX.sym("ax", 1)  # longitudinal acceleration

    # Vehicle parameters
    Lf = 1.156
    Lr = 1.42
    L = Lf + Lr
    m  = 1200
    Iz = 1792

    # Pacejka Magic Formula coefficients (B, C, D, E)
    B = 7.1433; C = 1.3507; D = 1.0489; E = -0.0074722

    # Nominal vertical loads (static weight distribution)
    Fz_f = (Lr / L) * m * 9.81   # front axle
    Fz_r = (Lf / L) * m * 9.81   # rear axle

    # State variable
    x  = MX.sym("x", 6)
    theta = x[2]
    vx    = x[3]
    vy    = x[4]
    r     = x[5]
    delta = u[0]

    # Tire slip angles
    vx_safe = fmax(0.5, vx)
    alpha_f = delta - atan((vy + Lf * r) / vx_safe)
    alpha_r = -atan((vy - Lr * r) / vx_safe)

    # Pacejka lateral forces
    Fyf = Fz_f * D * sin(C * atan(B*alpha_f - E*(B*alpha_f - atan(B*alpha_f))))
    Fyr = Fz_r * D * sin(C * atan(B*alpha_r - E*(B*alpha_r - atan(B*alpha_r))))

    # Equations of motion
    xdot = vertcat(
        vx * cos(theta) - vy * sin(theta),              # dx/dt
        vx * sin(theta) + vy * cos(theta),              # dy/dt
        r,                                              # dtheta/dt = r
        ax + r * vy - Fyf * sin(delta) / m,             # dvx/dt
        (Fyf * cos(delta) + Fyr) / m - vx * r,          # dvy/dt
        (Fyf * Lf * cos(delta) - Fyr * Lr) / Iz         # dr/dt
    )

    f_dyn = Function('f_dyn', [x, u, ax], [xdot])

    # RK4 integration
    k1 = f_dyn(x,                   u, ax)
    k2 = f_dyn(x + dt_dyn/2 * k1,  u, ax)
    k3 = f_dyn(x + dt_dyn/2 * k2,  u, ax)
    k4 = f_dyn(x + dt_dyn   * k3,  u, ax)
    x_next = x + (dt_dyn / 6) * (k1 + 2*k2 + 2*k3 + k4)

    F_dyn = Function('F_dyn', [x, u, ax], [x_next])

    # Parametrising the initial state and targets avoids recompiling the symbolic graph and IPOPT solver at every step.
    P  = MX.sym("P", 6 + 1 + N_dyn * 3)
    Us = MX.sym("U", N_dyn)

    X_sym  = P[0:6]
    ax_sym = P[6]
    J = 0
    for k in range(N_dyn):
        X_sym = F_dyn(X_sym, vertcat(Us[k]), vertcat(ax_sym))
        tx   = P[7 + k*3]
        ty   = P[7 + k*3 + 1]
        tyaw = P[7 + k*3 + 2]
        J += 150.0 * (X_sym[0] - tx)**2
        J += 150.0 * (X_sym[1] - ty)**2
        J +=  20.0 * (X_sym[2] - tyaw)**2

    J += mtimes(Us.T, Us) * 100000.0

    # Penalty on steering rate between consecutive steps (smoothness).
    for k in range(N_dyn - 1):
        J += 5000.0 * (Us[k+1] - Us[k])**2

    nlp = {'x': Us, 'f': J, 'p': P}
    opts = {
        "ipopt.tol": 1e-3,
        "ipopt.acceptable_tol": 1e-2,
        "ipopt.acceptable_iter": 5,
        "ipopt.max_iter": 20,
        "ipopt.print_level": 0,
        "expand": True,
    }
    solver_dyn = nlpsol("solver_dyn", "ipopt", nlp, opts)


def opt_step_dynamic(targets, state, ax_input):
    global solver_dyn, prev_u_dyn

    # Numeric parameter vector
    p_num = np.concatenate([
        [state.x, state.y, state.theta, state.vx, state.vy, state.r],
        [ax_input],
        np.array(targets[:N_dyn]).flatten(),
    ])

    # Warm start
    # At the first step use zeros; afterwards shift the previous solution by
    # one step so IPOPT starts close to the solution → fewer iterations.
    if prev_u_dyn is None:
        x0 = np.zeros(N_dyn)
    else:
        x0 = np.roll(prev_u_dyn, -1)
        x0[-1] = 0.0

    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(N)) # lower bound for steer
    arg["ubx"] =  vertcat(max_steer*np.ones(N)) # upper bound for steer
    arg["x0"] =    0.0 # first guess 
    arg["p"] = p_num

    #These can be ingnored 
    # # Bounds on g
    # arg["lbg"] = lower_bound on a state
    # arg["ubg"] = inf

    res = solver_dyn(**arg)

    ctrls = reshape(res["x"], (N,1)).T #reshape to have a row for each step
    prev_u_dyn = ctrls

    return ctrls[0][0]

def casadi_model():
    global F

    # Control
    # Create 1r-1c matrix containing control inputs. 
    # Set steer as element
    u = MX.sym("u",1)
    steer = u[0]

    # Constants - Model parameters
    Lr = 1.42 
    Lf = 1.156
    L = Lf + Lr

    # State
    x = MX.sym("x",4)
    sx    = x[0]  # position x
    sy    = x[1]  # position y
    yaw   = x[2]  # yaw
    speed = x[3]

    # ODE right hand side
    sxdot    = speed*cos(yaw)
    sydot    = speed*sin(yaw)
    yawdot   = (speed/L)*tan(steer)
    speeddot = 0.0

    # Concatenate vertically the expressions creating a row vector
    xdot = vertcat(sxdot, sydot, yawdot, speeddot)

    # ODE right hand side function
    # as input are used the state and control inputs,
    # and as output the row vector containing the model expressions
    f = Function('f', [x,u],[xdot])

    # Integrate the step with Explicit Euler
    IN = 1
    xj = x
    for i in range(IN):
        fj = f(xj,u)
        xj += dt*fj/IN

    # Discrete time dynamics function
    F = Function('F', [x,u],[xj])

def opt_step(target, state):
    global F

    # Control for all segments
    nu = N #number of states
    Us = MX.sym("U",nu) #steer control input

    # Initial conditions
    x0 = [state.x, state.y, state.theta, state.vx]
    X0 = MX(x0) # vector containing the initial state

    J = 0 # objective function that should be minimized by the nlp solver

    # build graph
    X=X0
    G = None
    lbg = []
    ubg = []

    # For every temporal step
    for k in range(nu):
        X = F(X, vertcat(Us[k])) #Integrate the step
        gain_mult = 1
        # give more importance to the last step using a bigger gain
        if(k == nu-1):
            gain_mult=1 #You can use this multiplier as terminal cost
        J += 150.0*gain_mult*(X[0]-target[k][0])**2 #x error cost 
        J += 150.0*gain_mult*(X[1]-target[k][1])**2 #y error cost
        J += 20.0*gain_mult*(X[2] - target[k][2])**2 #heading error cost
    # G = X[index] #if you want to set a state to constrain in arg["lbg"] and arg["ubg"]. It can be ignored


    # Objective function and constraints
    J += mtimes(Us.T,Us)*60000.0 #Consider to set this weight dependent on speed

    # NLP
    nlp = {'x':vertcat(Us), 'f':J}
    
    # Allocate an NLP solver
    opts = {
    "ipopt.tol": 1e-3,
    "ipopt.acceptable_tol": 1e-2,
    "ipopt.acceptable_iter": 5,
    "ipopt.max_iter": 20,
    "ipopt.print_level": 1,
    "expand": True,
    }

    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(nu)) # lower bound for steer
    arg["ubx"] =  vertcat(max_steer*np.ones(nu)) # upper bound for steer
    arg["x0"] =    0.0 # first guess 

    #These can be ingnored 
    # # Bounds on g
    # arg["lbg"] = lower_bound on a state
    # arg["ubg"] = inf

    # Solve the problem
    res = solver(**arg)
    #print "f:", res["f"]
    ctrls = reshape(res["x"], (nu,1)).T #reshape to have a row for each step

    return ctrls[0][0]
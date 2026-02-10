import numpy as np

class Simulation:
    def __init__(self, lf, lr, mass, Iz, dt, integrator="euler", model="kinematic", vx0=10.0):
        """
        Initialize the simulation parameters.
        """
        self.l_f = lf                   # Distance to front axle (m)
        self.l_r = lr                   # Distance to rear axle (m)
        self.l_wb = lf + lr
        self.mass = mass                # Vehicle mass (kg)
        self.I_z = Iz                   # Yaw moment of inertia (kg*m^2)
        self.dt = dt                    # Time step (s)
        self.integrator = integrator    # Integrator choice
        self.model = model              # Model choice
        
        # Aerodynamic and rolling resistance parameters
        self.rho = 1.225               # Air density (kg/m^3)
        self.C_d = 0.3                 # Drag coefficient (typical for cars)
        self.A = 2.2                   # Frontal area (m^2)
        self.C_rr = 0.015              # Rolling resistance coefficient
        self.Fz = self.mass * 9.81 

        # Initialize states
        self.x = 0                      # X position (m)
        self.y = 0                      # Y position (m)
        self.theta = 0                  # Heading angle (rad)

        # Velocita iniziale
        self.vx = vx0                     # Longitudinal velocity (m/s)
        self.vy = 0                     # Lateral velocity (m/s)
        self.r = 0                      # Yaw rate (rad/s)
        self.alpha_f = 0                # Front sleep angle
        self.alpha_r = 0                # Rear sleep angle

        # Pacejka's Magic Formula coefficients
        self.B, self.C, self.D, self.E = 7.1433, 1.3507 , 1.0489, -0.0074722
        self.B_f, self.C_f, self.D_f, self.E_f = self.B, self.C, self.D, self.E
        self.B_r, self.C_r, self.D_r, self.E_r = self.B, self.C, self.D, self.E
        
        self.Cf, self.Cr = self.B_f*self.C_f*self.D_f, self.B_r*self.C_r*self.D_r  # Cornering stiffness front/rear (N/rad)

    def kinematic_model(self, ax, delta):
        """ Kinematic single-track model equations of motion. """
        
        # Aerodynamic drag and rolling resistance forces
        F_aero = 0.0
        F_roll = self.C_rr * self.Fz
        
        dx = np.array([
            self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),  # dx/dt
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),  # dy/dt
            self.vx/self.l_wb * np.tan(delta),  # dtheta/dt
            ax + self.r * self.vy,  # dvx/dt with resistive forces
            0,
            0
        ])
        return dx

    def linear_single_track_model(self, ax, delta):
        """ Linear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        self.alpha_f = delta - (self.vy + self.l_f * self.r) / self.vx 
        self.alpha_r = - (self.vy - self.l_r * self.r) / self.vx

        # Vertical forces (nominal vertical load)
        Fz_f_nominal = self.l_r/self.l_wb * self.Fz
        Fz_r_nominal = self.l_f/self.l_wb * self.Fz

        # Front and rear lateral forces
        Fyf = self.alpha_f * self.Cf * Fz_f_nominal
        Fyr = self.alpha_r * self.Cr * Fz_r_nominal

        # Aerodynamic drag and rolling resistance forces
        F_aero = 1/2 * self.rho * self.C_d * self.A * self.vx**2
        F_roll = self.C_rr * self.Fz

        # Dynamics equations
        dx = np.array([
            self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),  # dx/dt
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),  # dy/dt
            self.r,                                                      # dtheta/dt
            ax + self.r * self.vy - (F_aero + F_roll + Fyf * np.sin(delta)) / self.mass,       # dvx/dt with resistive forces
            # Substitute alpha_f, alhpa_r and then Fyf and Fyr into the equations of motion from slide 29
            1 / self.mass * (Fyr + Fyf * np.cos(delta))  - self.vx * self.r,                  # dvy/dt
            1 / self.I_z * (Fyf * self.l_f * np.cos(delta) - Fyr * self.l_r)                # dr/dt
        ])
        
        return dx

    def nonlinear_single_track_model(self, ax, delta):
        """ Nonlinear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        self.alpha_f = delta - np.arctan((self.vy + self.l_f * self.r) / self.vx)
        self.alpha_r = - np.arctan((self.vy - self.l_r * self.r) / self.vx)

        # Vertical forces (nominal vertical load)
        Fz_f_nominal = self.l_r/self.l_wb * self.Fz
        Fz_r_nominal = self.l_f/self.l_wb * self.Fz

        # Front and rear lateral forces
        Fyf = self.Fyf = Fz_f_nominal * self.D_f * np.sin(self.C_f * np.arctan(self.B_f * self.alpha_f - self.E_f * (self.B_f * self.alpha_f - np.arctan(self.B_f * self.alpha_f))))
        Fyr = self.Fyr = Fz_r_nominal * self.D_r * np.sin(self.C_r * np.arctan(self.B_r * self.alpha_r - self.E_r * (self.B_r * self.alpha_r - np.arctan(self.B_r * self.alpha_r))))

        # Aerodynamic drag and rolling resistance forces
        F_aero = 1/2 * self.rho * self.C_d * self.A * self.vx**2
        F_roll = self.C_rr * self.Fz

        # Dynamics equations
        dx = np.array([
            self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta),  # dx/dt
            self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta),  # dy/dt
            self.r,                                                      # dtheta/dt
            ax + self.r * self.vy - (F_aero + F_roll + Fyf * np.sin(delta)) / self.mass,       # dvx/dt with resistive forces
            # Substitute alpha_f, alhpa_r and then Fyf and Fyr into the equations of motion from slide 29
            1 / self.mass * (Fyr + Fyf * np.cos(delta))  - self.vx * self.r,                  # dvy/dt
            1 / self.I_z * (Fyf * self.l_f * np.cos(delta) - Fyr * self.l_r)              # dr/dt
        ])
        
        return dx

    def integrate(self, ax, delta):
        """ Select the integrator method and apply it to update the state. """
        if self.integrator == "euler":
            self.euler_step(ax, delta)
        elif self.integrator == "rk4":
            self.rk4_step(ax, delta)

    def euler_step(self, ax, delta):
        """ Euler integration method. """
        dx = self.compute_dx(ax, delta)
        self.update_state(dx)

    def rk4_step(self, ax, delta):
        """ Runge-Kutta 4th order integration method. """
        k1 = self.compute_dx(ax, delta)
        self.update_state(k1, scale=0.5)
        
        k2 = self.compute_dx(ax, delta)
        self.update_state(k2, scale=0.5, revert=k1, revert_scale=0.5)
        
        k3 = self.compute_dx(ax, delta)
        self.update_state(k3, scale=1, revert=k2, revert_scale=0.5)

        k4 = self.compute_dx(ax, delta)
        self.update_state(np.zeros(6), scale=1, revert=k3, revert_scale=1)
        
        # Combine k1, k2, k3, k4 for RK4 update
        dx = (k1 + 2*k2 + 2*k3 + k4) / 6
        self.update_state(dx)

    def compute_dx(self, ax, delta):
        """ Compute the state derivatives using the chosen model. """
        if self.model == "kinematic":
            return self.kinematic_model(ax, delta)
        elif self.model == "linear":
            return self.linear_single_track_model(ax, delta)
        elif self.model == "nonlinear":
            return self.nonlinear_single_track_model(ax, delta)

    def update_state(self, dx, scale=1, revert=None, revert_scale=1):
        """ Update state with scaled dx. Optionally revert previous state for RK4. """
        if revert is not None:
            self.x -= revert[0] * self.dt * revert_scale
            self.y -= revert[1] * self.dt * revert_scale
            self.theta -= revert[2] * self.dt * revert_scale
            self.vx -= revert[3] * self.dt * revert_scale
            self.vy -= revert[4] * self.dt * revert_scale
            self.r -= revert[5] * self.dt * revert_scale

        self.x += dx[0] * self.dt * scale
        self.y += dx[1] * self.dt * scale
        self.theta += dx[2] * self.dt * scale
        self.vx += dx[3] * self.dt * scale
        self.vy += dx[4] * self.dt * scale
        self.r += dx[5] * self.dt * scale

import random

class Fluid:
    """Class representing the fluid physics."""

    WATER_DENSITY = 997
    GRAVITY = 9.81
    TIME_INTERVAL = 0.05

    def __init__(self, 
                 env_depth=15,
                 fluid_density=WATER_DENSITY, 
                 gravity=GRAVITY,
                 dt=TIME_INTERVAL):
        self.rho = fluid_density
        self.g = gravity
        self.env_constraints = (-5, env_depth) # min depth 5m above water
        self.dt = dt
    
    def _constrain(self, val, constrains):
        low, high = constrains
        return min(high, max(low, val))
    
    def _sgn(self, val):
        if val > 0:
            return 1
        if val < 0:
            return -1
        return 0

    def update_vehicle(self, vehicle):
        y = vehicle.get_depth()
        vy = vehicle.get_velocity()
        mass = vehicle.get_mass()
        width = vehicle.get_width()
        volume = vehicle.get_volume_submerged()
        thrust = vehicle.get_thrust()
        drag_coeff = vehicle.get_drag_coeff()

        # buoyant force, gravity, drag, net force
        Fb = -self.rho * self.g * volume
        Fg = mass * self.g
        Fd = -0.5 * self._sgn(vy) * drag_coeff * width**2 * vy**2
        F = Fb + Fg + Fd + thrust * mass + random.normalvariate(0, 0.1)

        # new vy and y
        nv = vy + F / mass * self.dt
        ny = self._constrain(y + nv * self.dt, self.env_constraints)
        if self.env_constraints[0] > ny or self.env_constraints[1] < ny:
            nv = 0
        vehicle.set_velocity(nv)
        vehicle.set_depth(ny)
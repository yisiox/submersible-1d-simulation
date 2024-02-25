import fluid

class Vehicle:
    """Class representing a submersible with 1DOF in some fluid."""

    def __init__(self,
                 controller,
                 vehicle_width=0.2,
                 vehicle_height=0.6,
                 vehicle_mass=1,
                 y=0, # y coord
                 vy=0, # vertical velocity
                 drag_coeff=3,
                 thrust_limit=4,
                 tolerance=0.2):
        self.vehicle_width = vehicle_width
        self.vehicle_height = vehicle_height
        self.vehicle_mass = vehicle_mass
        self.drag_coeff = drag_coeff
        
        self.setpoint = 5 # default 5m below surface
        self.depth = y
        self.velocity = vy
        self.thrust_limit = thrust_limit
        
        self.tolerance = tolerance

        self.controller = controller
    
    def get_width(self):
        return self.vehicle_width
    
    def get_height(self):
        return self.vehicle_height
    
    def get_mass(self):
        return self.vehicle_mass
    
    def get_drag_coeff(self):
        return self.drag_coeff
    
    def get_thrust(self):
        thrust = self.controller.compute_control(self.setpoint, self.depth)
        return min(self.thrust_limit, max(0, thrust))

    def get_depth(self):
        return self.depth
    
    def get_setpoint(self):
        return self.setpoint
    
    def get_velocity(self):
        return self.velocity
    
    def get_volume_submerged(self):
        return self.vehicle_width**2 * (self.vehicle_height + min(self.depth, 0))
    
    def set_depth(self, depth):
        self.depth = depth

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def reached_goal(self):
        return abs(self.depth - self.setpoint) < self.tolerance
from controller import PidController


class Submersible:
    """Class simulating the submersible vehicle."""

    def __init__(
        self,
        width: float = 0.2,          # in m
        height: float = 0.6,         # in m
        mass: float = 12.0,          # in kg
        drag_coeff: float = 0.3,     # dimensionless
        init_setpoint: float = 5.0,  # in m
        init_depth: float = 0.0,     # in m
        thrust_limit: float = 300.0, # in N
        tolerance: float = 0.2,      # in m, for what considers as reached goal
        controller: PidController = PidController(0, 0, 0)
    ) -> None:
        self.width = width
        self.height = height
        self.mass = mass
        self.drag_coeff = drag_coeff

        self.setpoint = init_setpoint
        self.depth = init_depth
        self.velocity = 0.0
        self.thrust_limit = thrust_limit

        self.tolerance = tolerance

        self.controller = controller

    def reached_goal(self) -> bool:
        return abs(self.depth - self.setpoint) < self.tolerance

    def get_thrust(self) -> float:
        thrust = self.controller.compute_control(self.setpoint, self.depth)
        return min(self.thrust_limit, max(0, thrust)) # 0 when going up, assumption of +ve buoyancy
    
    def update_depth(
        self,
        dt: float,
        min_depth: float,
        max_depth: float
    ) -> None:
        self.depth += self.velocity * dt
        self.depth = max(min_depth, min(max_depth, self.depth))

    def update_velocity(self, dt: float, net_force: float) -> None:
        self.velocity += net_force / self.mass * dt

    def get_volume_submerged(self) -> float:
        return self.width * self.width * (self.height + min(self.depth, 0))

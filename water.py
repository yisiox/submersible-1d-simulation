from math import copysign


class Water:
    """Class simulating water and associated physics."""

    RHO = 997 # density of water in kgm^-3

    def __init__(
        self,
        env_depth: float = 15.0, # down is +ve
    ) -> None:
        self.env_depth = env_depth

    def get_bouyant_force(
        self,
        volume_submerged: float, # in m^3
        gravity: float = 9.81    # in ms^-2
    ) -> float:
        return -Water.RHO * gravity * volume_submerged

    def get_drag(self,
        velocity: float,
        drag_coefficient: float,
        area: float,    
    ) -> float:
        # the formula for drag
        return (-.5 
                * copysign(1, velocity) # to always oppose direction of motion
                * Water.RHO
                * velocity * velocity
                * drag_coefficient
                * area)

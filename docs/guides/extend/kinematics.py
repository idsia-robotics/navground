from navground import core

class MyKinematics(core.Kinematics):
    # set to whether there are wheels or not
    IS_WHEELED = False
    # set the number of dof
    DOF = 2

    # MUST override
    # return the nearest feasible twist to value
    def feasible(self, twist: core.Twist2) -> core.Twist2:
        ...

    # CAN override
    # return the nearest feasible twist to a target value from
    # the current value in a time step, taking into account dynamic constraints.
    # def feasible_from_current(self, twist: core.Twist2) -> core.Twist2: ...

    # SHOULD override
    # return whether we are using wheels
    # should be a constant
    def is_wheeled(self) -> bool:
        return self.IS_WHEELED

    # MUST override
    # return the number of degree of freedom
    # should be a constant
    def dof(self) -> int:
        return self.DOF

    # CAN override
    # return the number of degree of freedom
    # should be a constant
    # def get_max_angular_speed(self) -> float: ...

    # CAN override
    # return the number of degree of freedom
    # should be a constant
    # def get_max_speed(self) -> float: ...

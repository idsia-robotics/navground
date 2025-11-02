from typing import SupportsFloat

from navground import core

class MyBehaviorModulation(core.BehaviorModulation):

    # CAN override
    # modulates the behavior by setting parameters while caching their original value
    # def pre(self, behavior: core.Behavior, time_step: SupportsFloat) -> None: ...

    # CAN override
    # modulates the cmd and resets parameters to their original values
    # def post(self, behavior: core.Behavior, time_step: SupportsFloat, cmd: core.Twist2) -> core.Twist2: ...

    pass

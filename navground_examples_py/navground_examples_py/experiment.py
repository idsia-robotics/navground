from navground import sim

from .probes import CheckIfMoving, IsMovingProbe, IsMovingSparseProbe


def main():
    experiment = sim.load_experiment("""
steps: 300
time_step: 0.1
save_directory: ''
record_time: true
runs: 2
name: custom_probes
scenario:
  type: Cross
  agent_margin: 0.1
  side: 4
  target_margin: 0.1
  tolerance: 0.5
  groups:
    -
      type: thymio
      number: 20
      radius: 0.08
      control_period: 0.1
      speed_tolerance: 0.02
      kinematics:
        type: 2WDiff
        wheel_axis: 0.094
        max_speed: 0.166
      behavior:
        type: HL
        optimal_speed: 0.12
        horizon: 5.0
        safety_margin: 0.02
      state_estimation:
        type: Bounded
        range: 5.0
""")
    experiment.add_probe(CheckIfMoving)
    experiment.add_record_probe("is_moving", IsMovingProbe)
    experiment.add_group_record_probe("still_times", IsMovingSparseProbe)
    experiment.run()

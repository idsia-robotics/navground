from navground import sim
import numpy as np

from .probes import CheckIfMoving, IsMovingProbe, IsMovingSparseProbe


def main():
    world = sim.load_world("""
walls:
  - line: [[-1.0, -1.0], [-1.0, 1.0]]
obstacles:
  - position: [2.0, 0.0]
    radius: 0.5
agents:
  - kinematics:
      type: Omni
      max_speed: 1.0
    behavior:
      type: HL
    task:
      type: Waypoints
      waypoints: [[1.0, 0.0]]
      tolerance: 0.1
    radius: 0.1
    control_period: 0.1
""")
    run = sim.ExperimentalRun(world,
                              time_step=0.1,
                              steps=20,
                              record_config=sim.RecordConfig.all(False))
    run.add_probe(CheckIfMoving())
    probe = run.add_record_probe("is_moving", IsMovingProbe)
    _ = run.add_group_record_probe("still_times", IsMovingSparseProbe)
    run.run()
    data = np.asarray(probe.data)
    print(f"Recorded movements: {data.astype(bool).reshape((-1, ))}")
    record = run.get_records("still_times")
    data = {k: np.asarray(v) for k, v in record.items()}
    print(f"Recorded still times: {data}")

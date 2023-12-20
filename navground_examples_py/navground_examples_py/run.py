from navground import sim

from .probes import IsMovingProbe, IsMovingSparseProbe


def main():
    world = sim.load_world("""
walls:
  - [[-1.0, -1.0], [-1.0, 1.0]]
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
    probe = IsMovingProbe()
    run.add_probe("is_moving", probe)
    m_probe = IsMovingSparseProbe()
    run.add_probe("still_times", m_probe)
    run.run()
    print(f"Recorded {probe.steps} steps: {probe.data.astype(bool)}")
    print(f"Recorded times: {m_probe.data}")

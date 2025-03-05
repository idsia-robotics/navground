.. _kinematics:

==========
Kinematics
==========

The kinematics main functionality is computing *feasible* velocities (actually twists) from arbitrary velocities. 
They plays two roles in navground:

- they helps behaviors to output feasible commands,
- they acts as a lower level controller in simulation: commands outputted by a behavior are filtered through a kinematics before being actuated.

Kinematics are defined by the constraints they applies to velocities and match categories of agents, like wheeled robots, quadrotors, or humans,


------------

.. toctree::
   :maxdepth: 1

   omni
   ahead
   2wdiff
   2wdiffdyn
   4womni


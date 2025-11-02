.. _state_estimations:

=================
State Estimations
=================

State estimations job is to update the behavior local environment state.
They works with specific types of environment representation, for examples, updating the sensing readings or a symbolic list of obstacles. When a behavior uses an environment representation that is not yet supported by one of the state estimation, a new one as to be added in order to be able to simulate that behavior.

Sensors represent the class of state estimations that updates environment representation that are tabular data (actually a multi-dimensional homogeneous arrays, one for each type of reading), which are used in particular by Machine-Learning navigation policies, like the ones provided by `navground learning <https://github.com/idsia-robotics/navground_learning>`_.

Sensors can be combined together, each updating their part of the environment state. In this sense, they offers generic and modular state estimations.

------------

.. toctree::
   :maxdepth: 1

   bounded
   discs
   odometry
   lidar
   boundary
   local_gridmap
   marker
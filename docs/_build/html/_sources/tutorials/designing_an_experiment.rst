=======================
Designing an experiment
=======================

In this tutorials, you are learn how to design a navigation experiment.

Step by step, we will describe the :ref:`experiment in YAML <experiment yaml>`. This description can then be processed by ``navground`` to collect experimental data.

Scenario
========

First, we need to decide in which kind of navigation scenario we are going to experiment.
:ref:`Some scenarios <benchmark scenarios>` are already predefined. We can pick one of them, let's say :ref:`Corridor <corridor>`, or we can define our own scenario, which will be the subject of a different guide or tutorial.

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor

Scenarios may have specific configuration variables, which we list using :ref:`info`

.. code-block:: console

   $ info --scenario Corridor

   Corridor
     add_safety_to_agent_margin: 1 [bool]
     agent_margin: 0.1 [float]
     length: 10 [float]
     width: 1 [float]

Let's make the corridor wider and shorter:

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2

Agents
======

We now populate the scenario with groups of agents. Agents in the same group share the same configuration. To keep it simple, let's add a single group of 10 agents.


.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups: 
       - number: 10

Except ``number``, all the other configuration variables represents distributions: when the scenario is initialized, a sample from that distribution is used to initialize the agent. For example, let's say we want the agents to have random radii picked uniformly between 0.5 and 1.0 meters and a common control period of 0.1 s:

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: uniform
           from: 0.5
           to: 1.0
         control_step:
           sampler: constant
           value: 0.1

To avoid unnecessary verbose configurations, ``navground`` supports more compact notations for some distributions, like just providing the value for ``constant`` distributions, therefore we can simplify as


.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: uniform
           from: 0.5
           to: 1.0
         control_step: 0.1

To finalize the configuration of agents, we need to fix their kinematics, behaviors, tasks, state estimations and initial poses. In fact, some may be already configured by the scenario. For instance, ``Corridor`` initializes agents at random poses inside the corridor, therefore there is no need to configure their initial poses separately. Similarly, ``Corridor`` ask each agents to travel along the corridor, therefore we can skip ``task``. We still need to set the kinematics (here, omnidirectional), behavior (here, ``HL``) and state estimation (here, with a maximal range of 4 meters, i.e. half of a corridor):

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: uniform
           from: 0.5
           to: 1.0
         control_step: 0.1
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range_of_view: 4.0


Metrics
=======

What should we record? Let's say we want to plot the agents trajectories ... then we need to record their poses. We may want to record collisions too to perform some safety assessment. We should also set where to save data, for instance to the current directory.

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: uniform
           from: 0.5
           to: 1.0
         control_step: 0.1
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range_of_view: 4.0
   save_directory: '.'
   record_poses: true
   record_colllisions: true

Runs
====

Finally, we need to decide how many runs to execute and how long they are. Each run will be initialized from the same scenario. If the scenario has no randomization, all runs will result in the same results. In our case, ``Corridor`` does have random pose initialization and we also configured random radii, therefore each run will result in different trajectories (and possibly different number of collisions).

Let's say that we are good with a statistics collected from 100 runs, each 20 second long (i.e., with 200 steps of 0.1 s).

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: uniform
           from: 0.5
           to: 1.0
         control_step: 0.1
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range_of_view: 4.0
   save_directory: '.'
   record_poses: true
   record_colllisions: true
   runs: 100
   steps: 200
   time_step: 0.1



Now we are ready to put the configuration in a file like ``my_config.yaml`` and to make ``navground`` execute the experiment.



.. code-block:: console

   $ run my_config.yaml
   
   Experiment done
   Duration: 1.64725 s
   Saved to: "./experiment_2023-07-25_13-32-22/data.h5"







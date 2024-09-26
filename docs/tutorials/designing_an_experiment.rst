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

   $ info --scenario Corridor --properties

   Scenarios
   ---------

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

All configuration variables represent samplings: when the scenario is initialized, a sample from that sampling is used to initialize the agent. For example, let's say we want the agents to have individual random radii picked uniformly between 0.5 and 1.0 meters, a common type picked randomly as either ``"type1"`` or ``"type2"``, and a control period of 0.1 s:

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
         type: 
           sampler: choice
           values: ["type_1", "type_2"]
           once: true

.. note::

  Note how we specify ``once: true`` for ``type`` to sample once per run and assign the same ``type`` value to all agents in the group; without it, values (e.g., ``radius`` or ``type``) would sampled for each individual agent in the group. The scenario fields (e.g., ``radius``, ``length`` and ``width``) and the group ``number`` are always sampled per run instead.

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
         type: 
           sampler: choice
           values: ["type_1", "type_2"]
           once: true

To finalize the configuration of agents, we need to fix their kinematics, behaviors, tasks, state estimations and initial poses. In fact, some may be already configured by the scenario. For instance, ``Corridor`` initializes agents at random poses inside the corridor, therefore there is no need to configure their initial poses separately. Similarly, ``Corridor`` ask each agents to travel along the corridor, therefore we can skip ``task``. We still need to set the kinematics (here, omni-directional), behavior (here, ``HL``) and state estimation (here, with a maximal range of 4 meters, i.e. half of the corridor length):

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
         type: 
           sampler: choice
           values: ["type_1", "type_2"]
           once: true
         control_step: 0.1
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range: 4.0


We can try to sample a world from such a scenario. Save all but the root element ``scenario`` in ``my_scenario.yaml`` and run

.. code-block:: console

   $ sample --seed 0 my_scenario.yaml
   
   obstacles:
     []
   walls:
     - line:
       -
         - -8
         - 0
       -
         - 16
         - 0
       uid: 10
     - line:
       -
         - -8
         - 2
       -
         - 16
         - 2
       uid: 11
   agents:
     - behavior:
         type: HL
         aperture: 3.14159274
         barrier_angle: 1.57079637
         epsilon: 0
         eta: 0.5
         resolution: 101
         tau: 0.125
         optimal_speed: 1
         optimal_angular_speed: 1
         rotation_tau: 0.5
         safety_margin: 0
         horizon: 5
         radius: 0.774406791
         heading: idle
         kinematics:
           type: Omni
           max_speed: 1
           max_angular_speed: 1
         social_margin:
           modulation:
             type: constant
           default: 0
       kinematics:
         type: Omni
         max_speed: 1
         max_angular_speed: 1
       state_estimation:
         type: Bounded
         range: 4
       position:
         - 2.43710041
         - 0.875406802
       orientation: 0
       velocity:
         - 0
         - 0
       angular_speed: 0
       radius: 0.774406791
       control_period: 0
       type: type_2
       id: 0
       uid: 0
     - behavior:
         type: HL
         aperture: 3.14159274
         barrier_angle: 1.57079637
         epsilon: 0
         eta: 0.5
         resolution: 101
         tau: 0.125
         optimal_speed: 1
         optimal_angular_speed: 1
         rotation_tau: 0.5
         safety_margin: 0
         horizon: 5
         radius: 0.857594669
         heading: idle
         kinematics:
           type: Omni
           max_speed: 1
           max_angular_speed: 1
         social_margin:
           modulation:
             type: constant
           default: 0
       kinematics:
         type: Omni
         max_speed: 1
         max_angular_speed: 1
       state_estimation:
         type: Bounded
         range: 4
       position:
         - 1.05219924
         - 0.95859468
       orientation: 3.14159274
       velocity:
         - 0
         - 0
       angular_speed: 0
       radius: 0.857594669
       control_period: 0
       type: type_2
       id: 0
       uid: 1
    [other 8 agents omitted]


Metrics
=======

What should we record? Let's say we want to plot the agents trajectories ... then we need to record their poses. We may want to record collisions too to perform some safety assessment and the initial state of the world.
We should also set where to save data, for instance to the current directory.

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
         type: 
           sampler: choice
           values: ["type_1", "type_2"]
           once: true
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range: 4.0
   save_directory: '.'
   record_poses: true
   record_colllisions: true
   record_world: true

.. warning::

  Recordings are disabled by default to be as efficient as possible. You need to enabled the data you want to record.

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
         type: 
           sampler: choice
           values: ["type_1", "type_2"]
           once: true
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range: 4.0
   save_directory: '.'
   record_poses: true
   record_colllisions: true
   record_world: true
   runs: 100
   steps: 200
   time_step: 0.1



Now we are ready to put the configuration in a file like ``my_config.yaml`` and to make ``navground`` execute the experiment.



.. code-block:: console

   $ run my_config.yaml
   
   Experiment done
   Duration: 1.64725 s
   Saved to: "./experiment_2023-07-25_13-32-22/data.h5"


Sampling per run
================

If we want to perform an experiment where we measure the impact of different *group* radii, we should switch to a radius sampler that sample once *per run* instead of once *per agent*, by specifying ``once: true``. For instance, this experiment

.. code-block:: YAML

   # Experiment configuration
   scenario:
     type: Corridor
     length: 8
     width: 2
     groups:
       - number: 10
         radius:
           sampler: regular
           from: 0.5
           to: 1.0
           number: 11
           once: true
         control_step: 0.1
         behavior:
           type: HL
         kinematics:
           type: Omni
           max_speed: 1.0
           max_angular_speed: 1.0
         state_estimation:
           type: Bounded
           range: 4.0
   save_directory: '.'
   record_poses: true
   record_colllisions: true
   record_world: true
   runs: 11
   steps: 200
   time_step: 0.1


runs 11 times, assigning ``radius=0.5`` to all agent the first time, ``radius=0.6`` the second time and so on until  ``radius=1.0`` the last time. 



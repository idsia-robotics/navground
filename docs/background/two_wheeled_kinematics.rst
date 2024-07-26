======================
Two-wheeled kinematics
======================

In navground, kinematics are responsible to compute feasible commands (i.e., twists) from arbitrary commands. The most basic kinematics, called "Omni", just limits the norm of velocity to ``maximal_speed`` and of angular speed to ``maximal_angular_speed``. Here we focus instead on the kinematics of a robot with two differential drive wheels depicted below.

.. tikz:: Top view of a two-wheeled differential drive robot
   :libs: arrows.meta
   :align: center
   :xscale: 25


   \filldraw[color=black!25] (0, 0) circle (1.2);
   \draw[line width=2] (-1,-0.4) -- (-1,0.4) node [midway, left] {$l$};
   \draw[line width=2] (1,-0.4) -- (1,0.4) node [midway, right] {$r$};;
   \draw[dotted, color=green!50!black] (-1,0) -- (1,0) node [xshift=-10, below]    {$A$};
   \draw[dotted, color=blue!50!black] (0,0) -- (135:1.2) node [midway, above]    {$R$};
   \draw[-{Latex[length=5]}] (0,0) -- (0, 0.5) node [midway, right] {$\vec e_1$};
   \draw[-{Latex[length=5]}] (0,0) -- (-0.5, 0) node [midway, below] {$\vec e_2$};

The robot has radius :math:`R` and its two wheels (left :math:`l` and right :math:`r`) are placed on a common axis, distanced by :math:`A`. 


Kinematics
==========

Kinematics primary deals with velocity and in particular with the mapping between body and actuators velocities.

In this case, the robot move forwards at linear speed :math:`v` with respect to its reference frame :math:`\{\vec e_1, \vec e_2\}` which in turn rotates at angular speed :math:`\omega`.

.. math::

   \vec v & = v \vec e_1 \\
   \dot{\vec e_1} & = \omega \vec e_2
   

To convert between wheel speeds :math:`v_{l, r}` and body velocity :math:`(v, \omega)`

.. math::

	v = \frac{v_r + v_l}{2} \\
	\omega = \frac{v_r - v_l}{A} \\
	v_{r, l} = v \pm \frac{A \omega}{2}

If wheel speed in limited by :math:`v^\max`, not all body velocities are feasible anymore; for example, it is not possible to move and rotate at maximal speed at the same time. In the diagram below, we see in dark green the interval of body velocities :math:`[-v^\max, v^\max] \times [-\omega^\max, \omega^\max]`, with  

.. math::

   \omega^\max = \frac{2v^\max}{A}

and in bright green the feasible set of velocities that corresponds to wheel velocities in :math:`[-v^\max, v^\max]`.

.. tikz:: Feasible velocity
   :libs: arrows.meta
   :align: center
   :xscale: 40

   \draw[->] (-1.5,0)--(1.5,0) node[below]{$v$};
   \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{A \omega}{2}$};
   \draw[->] (-1.5,-1.5)--(1.5,1.5) node[below, rotate=60]{$v_l$};
   \draw[->] (1.5,-1.5)--(-1.5, 1.5) node[left, rotate=60]{$v_r$};
 
   \draw[fill=green!10!black, fill opacity=0.5] (-1, -1) -- (1, -1) -- (1, 1)  -- (-1, 1) -- cycle;
   \draw[fill=green!75!black, fill opacity=0.5] (-1, 0) -- (0, -1) -- (1, 0) --  (0, 1) -- cycle;

There are many ways to match a twist that lies outside of the feasible set to a twist inside the feasible test. The navground "2WDiff" kinematics uses the following strategy that privileges angular over linear speed:

1. If the twist lies outside of dark green area, it is clipped, i.e., 

   .. math::
	
	  (v, \omega) \leftarrow (v|_{[-v^\max, -v^\max]}, \omega|_{[-\omega^\max, -\omega^\max]}) 

2. If the twist still lies outside of the bright area, we move it along the x-axis (i.e., we change :math:`v` while keeping :math:`\omega` until it meets the bright area). This is the same as first clipping the wheel speed that lies outside of the feasible interval and then setting the other wheel speed so that their difference is maintained. For example, if :math:`|v_l| > v^\max`, then

   .. math::

   	  \delta &\leftarrow v_r - v_l \\
      v_l &\leftarrow v_l|_{[-v^\max, -v^\max]} \\
      v_r &\leftarrow v_l + \delta


Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(max_speed=1, axis=1)
   >>> kinematics.max_angular_speed
   2.0

   >>> kinematics.feasible(core.Twist2((2, 0), 1.5))
   Twist2((0.250000, 0.000000), 1.500000, frame=Frame.relative)

   # the corresponding (left, right) wheel speeds
   >>> kinematics.wheel_speeds(core.Twist2((2, 0), 1.5))
   [-0.5, 1.0]

Acceleration
============

Moving to second order, we can compute the acceleration:

.. math::

   \frac{d(\vec v, \omega)}{dt} = (\dot v \vec e_1 + v \omega \vec e_2, \dot \omega)

We can ignore the transversal component :math:`\vec e_2` as it is sonely due to the lateral friction between wheel and floor that avoids that the robot slips and focus just on linear and angular accelerations:

.. math::

   a & \doteq \dot v  \\
   \alpha & \doteq \dot \omega


If we want to limit accelerations, we can add a "LimitAcceleration" modulation to the behavior that is computing the commands. This will simply compute the acceleration required to actuate the command over a given time step :math:`\Delta t`, clip it and returns the command obtained by applying the (clipped) acceleration on the current velocity :math:`(v_0, \omega_0)`:

.. math::

	(a, \alpha) & \leftarrow \left(\frac{v - v_0}{\Delta t}, \frac{\omega - \omega_0}{\Delta t}\right) \\
	(a, \alpha) & \leftarrow \left(a|_{[-a^\max, -a^\max]}, \alpha|_{[-\alpha^\max, -\alpha^\max]}\right)  \\
	(v, \omega) & \leftarrow \left(v_0 + a \Delta t, \omega_0 + \alpha \Delta t \right)

The same functionality is exposed by the :py:meth:`navground.core.Twist2.interpolate`.

Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(  max_speed=1, axis=1)
   >>> cmd = kinematics.feasible(core.Twist2((2, 0), 1.5))
   >>> current = core.Twist2((0, 0), 0, frame=core.Frame.relative)
   >>> current.interpolate(cmd, time_step=0.1, max_acceleration=1.0,   max_angular_acceleration=1.0)
   Twist2((0.100000, 0.000000), 0.100000, frame=Frame.relative)


Dynamics
========

Let's say we want to simulate a robot having motors. The simplest assumption we can make is that the motor torque is limited, which in turn limits accelerations. To understand this relationships, let's us compute the dynamic of the system. Let us assume that the robot as mass :math:`m`, vertical-component of moment of inertia :math:`I` and that the two motors apply traction :math:`F_{r,l}` on the ground without slipping.


.. tikz:: Top view of a two-wheeled differential drive robot
   :libs: arrows.meta
   :align: center
   :xscale: 25


   \filldraw[color=black!25] (0, 0) circle (1.2);
   \draw[line width=2] (-1,-0.4) -- (-1,0.4);
   \draw[line width=2] (1,-0.4) -- (1,0.4);
   \draw[-{Latex[length=5]}] (-1,0.4) -- (-1, 1) node [left] {$F_l \vec e_1$};
   \draw[-{Latex[length=5]}] (1,0.4) -- (1, 0.8) node [right] {$F_r \vec e_1$};


These forces will cause the robot to accelerate as

.. math::

	m a &= F_l + F_r \\
	I \alpha &= (F_l - F_r) \frac{A}{2}

The reverse is given by

.. math::

	F_{r,l} = (m a \pm 2 I \alpha / A) / 2 

To simplify the expressions, we introduce the unit-less (scaled) moment of inertia as the ratio between :math:`I` and the moment of inertial of an homogeneous disc of diameter :math:`A`, and :math:`f` as the ratio between wheel force and mass:

.. math::

   i & = \frac{I}{m A^2/8} \\
   f_{r, l} & = \frac{F_{r, l}}{m} \\

then

.. math::

	f_{r,l} &= (a \pm i A \alpha / 4) / 2  \\
	a &= f_{r} + f_{l}\\
	\alpha &= 4 (f_{r} - f_{l}) / (i A) \\

From these, we can compute the mapping between wheel acceleration and forces

.. math::

   f_{r,l} & = \left((2 + i) \dot v_r  + (2 - i) \dot v_l\right) / 8  \\
   \dot v_{r, l} & = \left((2+i) f_{r, l} - (2-i) f_{l, r}\right) / i


Maximal body acceleration is obtained when both forces are maximal in the same direction, while maximal body angular acceleration when they act in opposite directions:

.. math::

   a^\max &= 2f^\max \\
   \alpha^\max &= 4 a^\max / (i A) \\

We can specify the dynamic by :math:`a^\max` and :math:`i` (or :math:`\alpha^\max` instead of mass, and maximal motor torque.

The relationship between wheel forces and speeds is similar to the relationship between body and wheel speeds and creates further constrains on feasible accelerations.

As before, given an arbitrary acceleration :math:`(a, \alpha)` there may be different ways to compute a feasible acceleration (which do not require excessive motor torque). The navground kinematics "2WDiffDyn" first compute the required torques,  and then compute the corresponding accelerations.

1. first computes the required torques

   .. math::

   	  (f_r, f_l) \leftarrow (a \pm i A \alpha / 4) / 2

2. then clip them
	
   .. math::

	  f_{r, l} \leftarrow f_{r, l} |_{[-f^\max, f^\max]}

3. before computing feasible acceleration 

   .. math::

	  (a, \alpha) \leftarrow \left(f_{r} + f_{l}, 4 (f_{r} - f_{l}) / (i A)\right)


.. tikz:: Feasible accelerations
   :libs: arrows.meta
   :align: center
   :xscale: 40

   \draw[->] (-1.5,0)--(1.5,0) node[below]{$a$};
   \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{i A \alpha}{4}$};
   \draw[->] (-1.5,-1.5)--(1.5,1.5) node[below, rotate=45]{$\frac{f_l}{2}$};
   \draw[->] (1.5,-1.5)--(-1.5, 1.5) node[left, rotate=45]{$\frac{f_r}{2}$};
 
   \draw[fill=green!10!black, fill opacity=0.5] (-1, -1) -- (1, -1) -- (1, 1)  -- (-1, 1) -- cycle;
   \draw[fill=green!75!black, fill opacity=0.5] (-1, 0) -- (0, -1) -- (1, 0) --  (0, 1) -- cycle;


This dynamic-aware differential drive kinematics, when computing a feasible twist, at first it perform the same as the not-dynamic-aware differential drive kinematics, but in addition makes sure that the motors are able to actuate the command. To summarize, given a twist, it compute the feasible twist achievable over a time_step by:

1. computing a feasible twist ignoring accelerations,
2. computing the required motor torque to achieve this twist over a time step,
3. applies the (potentially clipped) torques for a time step


Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.DynamicTwoWheelsDifferentialDriveKinematics(max_speed=1, axis=1, max_acceleration=1, moi=1)
   >>> kinematics.max_angular_speed
   2.0
   >>> kinematics.max_angular_acceleration
   4.0
   >>> kinematics.feasible(core.Twist2((2, 0), 1.5), current=core.Twist2((0, 0), 0), time_step=0.1)
   Twist2((0.050000, 0.000000), 0.200000, frame=Frame.relative)



Experiment
==========

Let us do a short experiment that test the three solutions.


We simulate one run of the same scenario as in:doc:`tutorials/tour`, with a single static obstacle that obstruct a target.

.. tabs::

   .. tab:: No dynamic, no acceleration limits

      .. code-block:: yaml

         steps: 2000
         time_step: 0.1
         record_pose: true
         record_actuated_cmd: true
         scenario:
           obstacles:
             - radius: 1
               position: [5, 0.1]
           groups:
             -
               number: 1
               radius: 1
               control_period: 0.1
               speed_tolerance: 0.02
               kinematics:
                 type: 2WDiff
                 wheel_axis: 1
                 max_speed: 1
               behavior:
                 type: ORCA
               state_estimation:
                 type: Bounded
                 range: 10.0
               task:
                 type: Waypoints
                 waypoints: [[10, 0]]
                 loop: false
                 tolerance: 1

   .. tab:: No dynamic, acceleration limits

      .. code-block:: yaml

         steps: 2000
         time_step: 0.1
         record_pose: true
         record_actuated_cmd: true
         scenario:
           obstacles:
             - radius: 1
               position: [5, 0.1]
           groups:
             -
               number: 1
               radius: 1
               control_period: 0.1
               speed_tolerance: 0.02
               kinematics:
                 type: 2WDiff
                 wheel_axis: 1
                 max_speed: 1
               behavior:
                 type: ORCA
                 modulations:
                   - type: LimitAcceleration
                     max_acceleration: 1
                     max_angular_acceleration: 4
               state_estimation:
                 type: Bounded
                 range: 10.0
               task:
                 type: Waypoints
                 waypoints: [[10, 0]]
                 loop: false
                 tolerance: 1
         
         
   .. tab:: Dynamic
   
      .. code-block:: yaml
   
         steps: 2000
         time_step: 0.1
         record_pose: true
         record_actuated_cmd: true
         scenario:
           obstacles:
             - radius: 1
               position: [5, 0.1]
           groups:
             -
               number: 1
               radius: 1
               control_period: 0.1
               speed_tolerance: 0.02
               kinematics:
                 type: 2WDiffDyn
                 wheel_axis: 1
                 max_speed: 1
                 max_acceleration: 1
                 moi: 1
               behavior:
                 type: ORCA
               state_estimation:
                 type: Bounded
                 range: 10
               task:
                 type: Waypoints
                 waypoints: [[10, 0]]
                 loop: false
                 tolerance: 1


Let's compare the trajectories,

.. image:: 2wk_trajectory.pdf
   :width: 800

the linear accelerations,

.. image:: 2wk_actuated_lin_acc.pdf
   :width: 800

the angular accelerations,

.. image:: 2wk_actuated_ang_acc.pdf
   :width: 800


and the forces acting on the motors (dashed = left motor).

.. image:: 2wk_actuated_force.pdf
   :width: 800


Because the acceleration limits are large enough, the trajectories are similar. Yet, we observe how the forces required by the robot wheels in case of no acceleration limits are much larger and exceed the feasible band ([-0.5, 0.5]) defined by the "2WDiffDyn" kinematics.


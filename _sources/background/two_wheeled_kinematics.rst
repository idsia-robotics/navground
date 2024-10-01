======================
Two-wheeled kinematics
======================

In navground, kinematics are responsible to compute feasible commands (i.e., twists) from arbitrary commands. The most basic kinematics, called "Omni", simply limits the norm of velocity to ``maximal_speed`` and the absolute value of angular speed to ``maximal_angular_speed``. In this section, we focus on the kinematics of a robot with two differential drive wheels as depicted below.

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

The robot has radius :math:`R` with two wheels (left :math:`l` and right :math:`r`), positioned on a common axis, distanced by :math:`A`. We introduce a robot-fixed, positively oriented, orthonormal reference frame :math:`\{\vec e_1, \vec e_2\}`, positioned at the center of the wheel axis.


Kinematics
==========

Kinematics primary deals with velocities and in particular with the mapping between body velocity and actuators velocities.
In our case, the robot moves forwards at linear speed :math:`v` with respect to its reference frame :math:`\{\vec e_1, \vec e_2\}`, which in turn rotates at angular speed :math:`\omega`

.. math::

   \vec v & = v \vec e_1, \\
   \dot{\vec e_1} & = \omega \vec e_2.
   

To convert between wheel speeds :math:`v_{l, r}` and body velocity :math:`(v, \omega)`, we can use

.. math::

	v = \frac{v_r + v_l}{2}, \\
	\omega = \frac{v_r - v_l}{A}, \\
	v_{r, l} = v \pm \frac{A \omega}{2}.

If wheel speed is limited by :math:`v^\max`, not all body velocities are feasible anymore; for example, it is not possible to move and rotate at maximal speed at the same time. In the diagram below, we represent in dark green the interval of body velocities :math:`[-v^\max, v^\max] \times [-\omega^\max, \omega^\max]` with  

.. math::

   \omega^\max = \frac{2v^\max}{A},

and in bright green the feasible set of velocities that corresponds to wheel velocities in :math:`[-v^\max, v^\max]`.

.. tikz:: Feasible velocity set
   :libs: arrows.meta
   :align: center
   :xscale: 40

   \draw[->] (-1.5,0)--(1.5,0) node[below]{$v$};
   \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{A \omega}{2}$};
   \draw[->] (-1.5,-1.5)--(1.5,1.5) node[below, rotate=60]{$v_r$};
   \draw[->] (1.5,-1.5)--(-1.5, 1.5) node[left, rotate=60]{$v_l$};
 
   \draw[fill=green!10!black, fill opacity=0.5] (-1, -1) -- (1, -1) -- (1, 1)  -- (-1, 1) -- cycle;
   \draw[fill=green!75!black, fill opacity=0.5] (-1, 0) -- (0, -1) -- (1, 0) --  (0, 1) -- cycle;

There are different ways to match a twist that lies outside of the feasible set to a twist inside the feasible test. The navground "2WDiff" kinematics uses the following strategy that privileges angular over linear speed:

1. first, angular speed is clipped

   .. math::
	
	  \omega \leftarrow \omega \left|_{[-\omega^\max, -\omega^\max]} \right.

   .. tikz:: Clipping angular speed
      :libs: arrows.meta
      :align: center
      :xscale: 30
   
      \draw[->] (-1.5,0)--(1.5,0) node[below]{$v$};
      \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{A \omega}{2}$};
    
      \draw[dashed] (-2, -1) -- (2, -1) node[right]{$-v^{\max}$};
      \draw[dashed] (-2, 1) -- (2, 1) node[right]{$+v^{\max}$};

      
      \draw[color=green!10!black] (-1, -1) -- (1, -1) -- (1,    1)  -- (-1, 1) -- cycle;
      \draw[color=green!75!black] (-1, 0) -- (0, -1) -- (1, 0)    --  (0, 1) -- cycle;

      \coordinate (A) at (0.8, 1.5) ;
      \coordinate (A1) at (0.8, 1.0) ;
      \coordinate (B) at (0.6, -0.7) ;
      \coordinate (C) at (-0.3, -0.2) ;

      \draw [-{Latex[length=5]}, thick](A) -- (A1) ;
      \filldraw[red!50] (A1) circle (2pt);
      \filldraw[green!10!black] (A) circle (2pt);
      \filldraw[green!10!black] (B) circle (2pt);
      \filldraw[green!10!black] (C) circle (2pt);
      

2. then, linear speed is clipped so that the twist is contained in the feasible set:

   .. math::

      v \leftarrow v \left|_{[-v^\max - |\omega \frac{A}{2}|, v^\max + |\omega \frac{A}{2}|]}  \right.

   .. tikz:: Clipping linear speed
      :libs: arrows.meta
      :align: center
      :xscale: 18
   
      \draw[->] (-1.5,0)--(1.5,0) node[below]{$v$};
      \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{A \omega}{2}$};
    
      \draw[color=green!10!black] (-1, -1) -- (1, -1) -- (1,    1)  -- (-1, 1) -- cycle;
      \draw[color=green!75!black] (-1, 0) -- (0, -1) -- (1, 0)    --  (0, 1) -- cycle;

      \coordinate (A) at (0.8, 1.0) ;
      \coordinate (A1) at (0.0, 1.0) ;
      \coordinate (B) at (0.6, -0.7) ;
      \coordinate (B1) at (0.3, -0.7) ;
      \coordinate (C) at (-0.3, -0.2) ;

      \draw [-{Latex[length=5]}, thick](A) -- (A1) ;
      \draw [-{Latex[length=5]}, thick](B) -- (B1) ;
      \filldraw[green!10!black] (A) circle (2pt);
      \filldraw[green!10!black] (B) circle (2pt);
      \filldraw[green!75!black] (A1) circle (2pt);
      \filldraw[green!75!black] (B1) circle (2pt);
      \filldraw[green!75!black] (C) circle (2pt);

This is the same as first clipping the largest wheel speed and then setting the other wheel speed so that their difference is maintained.

Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(max_speed=1, axis=1)
   >>> kinematics.max_angular_speed
   2.0

   >>> cmd = kinematics.feasible(core.Twist2((2, 0), 1.5, frame=Frame.relative))
   >>> cmd
   Twist2((0.250000, 0.000000), 1.500000, frame=Frame.relative)

   # the corresponding (left, right) wheel speeds
   >>> kinematics.wheel_speeds(cmd)
   [-0.5, 1.0]

Acceleration
============

Moving to second order, we can compute the acceleration:

.. math::

   \frac{d(\vec v, \omega)}{dt} = (\dot v \vec e_1 + v \omega \vec e_2, \dot \omega)

We can ignore the transversal component :math:`\vec e_2` as it is sonely due to the lateral friction between wheel and floor that avoids that the robot slips. Therefore, we focus on linear and angular accelerations:

.. math::

   a & \doteq \dot v  \\
   \alpha & \doteq \dot \omega


If we want to limit accelerations, we can add a "LimitAcceleration" modulation to the behavior that is computing the commands. This will simply compute the acceleration required to actuate the command over a given time step :math:`\Delta t`, clip it and return the command obtained by applying the (clipped) acceleration on the current velocity :math:`(v_0, \omega_0)`:

.. math::

	(a, \alpha) & \leftarrow \left(\left. \frac{v - v_0}{\Delta t} \right|_{[-a^\max, -a^\max]},\left.\frac{\omega - \omega_0}{\Delta t} \right|_{[-\alpha^\max, -\alpha^\max]} \right) \\
	(v, \omega) & \leftarrow \left(v_0 + a \Delta t, \omega_0 + \alpha \Delta t \right)

The same functionality is exposed by :py:meth:`navground.core.Twist2.interpolate`.

Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.TwoWheelsDifferentialDriveKinematics(  max_speed=1, axis=1)
   >>> cmd = kinematics.feasible(core.Twist2((2, 0), 1.5))
   >>> current = core.Twist2((0, 0), 0, frame=core.Frame.relative)
   >>> current.interpolate(cmd, time_step=0.1, max_acceleration=1.0, max_angular_acceleration=1.0)
   Twist2((0.100000, 0.000000), 0.100000, frame=Frame.relative)


Dynamics
========

Let's say we want to simulate a robot having motors. The simplest assumption we can make is that the motor torque is limited, which in turn would limit acceleration. To understand this relationship, we compute the dynamics of the system. Let us assume that the robot has mass :math:`m`, vertical-component of moment of inertia :math:`I` and that the two motors apply forces :math:`F_{r,l}` to the robot body without slipping.


.. tikz:: Top view of a two-wheeled differential drive robot with motors
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

   i & \doteq \frac{I}{m A^2/8} \\
   f_{r, l} & \doteq \frac{2 F_{r, l}}{m} \\

then

.. math::

	f_{r,l} &= a \pm i A \alpha / 4  \\
	a &= \frac{f_{r} + f_{l}}{2}\\
	\alpha &= 2 \frac{f_{r} - f_{l}}{i A} \\

The mapping between wheel accelerations and forces shows that one wheel motor impacts also the other wheel when :math:`i \neq 2`:

.. math::

   f_{r,l} & = \frac{(2 + i) \dot v_{r, l}  + (2 - i) \dot v_{l, r}}{4}  \\
   \dot v_{r, l} & = \frac{(2+i) f_{r, l} - (2-i) f_{l, r}}{2i}

Maximal body acceleration is obtained when both forces are maximal in the same direction, while maximal body angular acceleration when they act in opposite directions:

.. math::

   a^\max &= f^\max \\
   \alpha^\max &= 4 a^\max / (i A) \\

We can fully specify the dynamics with :math:`a^\max` and :math:`i` (or :math:`\alpha^\max`) instead of mass and maximal motor torque.

The moment of inertia can be computed from the wheel accelerations when the robot accelerates at maximum moving straight (:math:`a^\max_{\mathit{fwd}} = a^\max`) and rotating in place (:math:`a^\max_{\mathit{rot}} = \alpha^\max \frac{A}{2}`):

.. math::
 
   i & = \frac{4 a^\max}{A \alpha^\max} \\
     & = 2 \frac{a^\max_{\mathit{fwd}}}{a^\max_{\mathit{rot}}}


The relationship between wheel forces and body acceleration is similar to the relationship between wheel speeds and body velocity and adds a constrain on feasible accelerations represented in the diagram below:

.. tikz:: Feasible acceleration set
   :libs: arrows.meta
   :align: center
   :xscale: 40

   \draw[->] (-1.5,0)--(1.5,0) node[below]{$a$};
   \draw[->] (0,-1.5)--(0,1.5) node[left]{$\frac{i A \alpha}{4}$};
   \draw[->] (-1.5,-1.5)--(1.5,1.5) node[below, rotate=45]{$f_r$};
   \draw[->] (1.5,-1.5)--(-1.5, 1.5) node[left, rotate=45]{$f_l$};
 
   \draw[fill=blue!10!black, fill opacity=0.5] (-1, -1) -- (1, -1) -- (1, 1)  -- (-1, 1) -- cycle;
   \draw[fill=blue!75!white, fill opacity=0.5] (-1, 0) -- (0, -1) -- (1, 0) --  (0, 1) -- cycle;


As before, given an arbitrary velocity :math:`(v, \omega)` there may be different ways to compute a feasible velocity from the current velocity :math:`(v_0, \omega_0)` (the dot below) over one time step :math:`\Delta t`, respecting both constraints: maximal torque (blue) and maximal velocity (green).

.. tikz:: Respecting dynamics, the feasible velocity set is the intersection of green and blue sets.
   :libs: arrows.meta
   :align: center
   :xscale: 40

   \draw[->] (-3,0)--(3,0) node[below]{$v$};
   \draw[->] (0,-4)--(0,4) node[left]{$\omega$};
 
   \draw[fill=green!75!black, fill opacity=0.5] (-2, 0) -- (0, -3) -- (2, 0) --  (0, 3) -- cycle;
   \draw[fill=blue!75!white, fill opacity=0.5] (0.6, -0.1) -- (1.2, 0.4) -- (0.6, 0.9)  -- (0.0, 0.4) -- cycle;

   \draw[->] (-0.2,0.4)--(1.4,0.4) node[below]{$a \Delta t$};
   \draw[->] (0.6,-0.4)--(0.6,1.2) node[left]{$\alpha \Delta t$};
   \filldraw[black] (0.6, 0.4) circle (2pt);

Similarly to "2WDiff", the navground kinematics "2WDiffDyn" privileges angular speed:

1. First, a feasible velocity is computed ignoring dynamics, i.e., clipping it inside the green set using the same strategy as "2WDiff".

2. Then, the angular speed is clipped to respect maximal angular acceleration
   
   .. math::
   
     \Delta \omega^\max & \leftarrow \alpha^\max \Delta t \\
     \omega & \leftarrow \omega \left|_{[\omega_0 -\Delta \omega^\max, \omega_0 + \Delta \omega^\max]} \right.

3. Finally, the linear speed is clipped, so that the twist is contained in the blue set:

   .. math::

      \Delta v^\max & \leftarrow a^\max \Delta t - |\omega - \omega_0| \frac{A i}{4} \\
      v & \leftarrow v \left|_{[v_0 -\Delta v^\max, v_0 + \Delta v^\max]} \right.

These three steps are illustrated below: 

.. tikz:: Computing a feasible twist respecting maximal torque.
   :libs: arrows.meta
   :align: center
   :xscale: 30

   \draw[->] (-3,0)--(3,0) node[below]{$v$};
   \draw[->] (0,-4)--(0,4) node[left]{$\omega$};
 
   \draw[color=green!75!black] (-2, 0) -- (0, -3) -- (2, 0) --  (0, 3) -- cycle;
   \draw[color=blue!75!white] (0.6, -0.1) -- (1.2, 0.4) -- (0.6, 0.9)  -- (0.0, 0.4) -- cycle;

   \coordinate (A) at (0.8, 3.5) ;
   \coordinate (A1) at (0.0, 3.0) ;
   \coordinate (A2) at (0.0, 0.9) ;
   \coordinate (A3) at (0.6, 0.9) ;
   \coordinate (B) at (-0.5, 0.2) ;
   \coordinate (B1) at (0.25, 0.2) ;
   \coordinate (C) at (0.8, 0.3) ;

   \draw [-{Latex[length=5]}, thick](A) -- node[above]{1} (A1);
   \draw [-{Latex[length=5]}, thick](A1) -- node[left]{2} (A2);
   \draw [-{Latex[length=5]}, thick](A2) -- node[above]{3} (A3);
   \draw [-{Latex[length=5]}, thick](B) -- node[above]{3} (B1) ;
   
   
   \filldraw[red!50] (A) circle (2pt);
   \filldraw[green!75!black] (A1) circle (2pt);
   \filldraw[green!75!black] (A2) circle (2pt);
   \filldraw[blue!75!white] (A3) circle (2pt);
   \filldraw[green!75!black] (B) circle (2pt);
   \filldraw[blue!75!white] (B1) circle (2pt);
   \filldraw[blue!75!white] (C) circle (2pt);

Alternatively, we could compute and clip the motor torques independently.

Example
-------

.. code-block:: python
  
   >>> from navground import core
   >>> kinematics = core.kinematics.DynamicTwoWheelsDifferentialDriveKinematics(
           max_speed=1, axis=1, max_acceleration=1, moi=1)0
   >>> kinematics.max_angular_acceleration
   4.0
   >>> kinematics.feasible(core.Twist2((2, 0), 1.5), current=core.Twist2((0, 0), 0), time_step=0.1)
   Twist2((0.050000, 0.000000), 0.200000, frame=Frame.relative)



Motor controller
================

If we want to apply a more complex motor controller compared to clipping torque to the feasible range, we can either create a new kinematics (possibly sub-classing "2WDiffDyn") or, preferably as kinematics should focus on constrains, introduce a new behavior modulation. 
Here we show an example of the latter, for a proportional torque controller that updates like:

.. math::

   e & \leftarrow f_{\mathrm{des}} - f \\
   f & \leftarrow f + k e

.. note::

   A similar PID controller is offered by :py:class:`navground.core.behavior_modulations.MotorPIDModulation`.

Example
-------

.. code-block:: python
  
   from navground import core
   import numpy as np
      
   class MotorController(core.BehaviorModulation, name="MotorController"):
       def __init__(self, k: float = 0.1):
           super().__init__()
           self._k = k
           self.torques: np.ndarray = np.zeros(2)
   
       def post(self, behavior: core.Behavior, time_step: float, 
                cmd: core.Twist2) -> core.Twist2:
           # We assume that the kinematics supports dynamics
           # Let's compute a feasible control
           current = behavior.get_twist(core.Frame.relative)
           cmd = behavior.kinematics.feasible(cmd, current, time_step)
           # and torques, which are also feasible
           target_torques = behavior.kinematics.wheel_torques(
               cmd, current, time_step)
           # We then apply a simple P-controller to the torques
           e = target_torques - self.torques
           max_torque = behavior.kinematics.max_wheel_torque
           self.torques = np.clip(self.torques + self.k * e, -max_torque, max_torque)
           # and return the twist obtained by applying these torques. 
           return behavior.kinematics.twist_from_wheel_torques(
               self.torques, current, time_step)
       
       @property
       @core.register(0.2, "P factor")
       def k(self) -> float:
           return self._k
      
       @k.setter
       def k(self, value: float) -> None:
           self._k = value

Experiment
==========

Let us do a short experiment that tests the four alternatives.
We simulate one run of the same scenario as in :doc:`../tutorials/tour`, with a single static obstacle to pass before reaching the target.

.. tabs::

   .. tab:: No dynamic, no acceleration limits

      .. code-block:: yaml

         steps: 140
         time_step: 0.1
         record_pose: true
         record_time: true
         record_actuated_cmd: true
         terminate_when_all_idle_or_stuck: false
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

         steps: 140
         time_step: 0.1
         record_pose: true
         record_time: true
         record_actuated_cmd: true
         terminate_when_all_idle_or_stuck: false
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
   
         steps: 140
         time_step: 0.1
         record_pose: true
         record_time: true
         record_actuated_cmd: true
         terminate_when_all_idle_or_stuck: false
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

   .. tab:: Dynamic with motor controller
   
      .. code-block:: yaml
   
         steps: 140
         time_step: 0.1
         record_pose: true
         record_time: true
         record_actuated_cmd: true
         terminate_when_all_idle_or_stuck: false
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
                 modulations:
                   - type: MotorController
                     k: 0.2
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

the wheel speeds,

.. image:: 2wk_actuated_wheel_speed.pdf
   :width: 800

the linear accelerations,

.. image:: 2wk_actuated_lin_acc.pdf
   :width: 800

the angular accelerations,

.. image:: 2wk_actuated_ang_acc.pdf
   :width: 800

and the forces required by the motors (dashed = left motor).

.. image:: 2wk_actuated_force.pdf
   :width: 800


Because the acceleration limits are large enough, the trajectories are similar. All controllers respect the wheel speed limits. We observe how the forces required by the robot wheels in case of no acceleration limits are much larger and exceed the feasible band ([-1, 1]) defined by the "2WDiffDyn" kinematics.


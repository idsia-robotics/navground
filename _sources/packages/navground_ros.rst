=============
navground_ros
=============


.. ros:currentnode:: RoboMasterROS

.. ros:package:: navground_ros
   :summary:

   This ROS2 provides a ROS2 compliant interface to ``navground_core``.


Executables
-----------

.. ros:executable:: controller

  Executes a ``ROSControllerNode`` using a single threaded executor.


Launch files
------------

.. ros:autolaunch_file:: navground_ros navigation.launch


.. ros:currentnode:: RoboMasterROS

Nodes
------

ROSControllerNode
~~~~~~~~~~~~~~~~~

.. ros:node:: RoboMasterROS
   :summary:

   This ROS2 node exposes a ROS2-compliant interface to a navigation controller :cpp:struct:`navground::core::Controller3`.


Parameters
----------

.. ros:parameter:: rate float
   :default: 10.0

   Control update rate in Hz. Should be positive.

.. ros:parameter:: frame_id str
   :default: world

   Name of the TF frame where to perform navigation computations.

.. ros:parameter:: publish_cmd_stamped bool
   :default: false

   Whether to publish a TwistStamped in :ros:param:`frame_id` instead of a Twist in body frame.

.. ros:parameter:: drawing bool
   :default: false
   :dynamic:

   Whether to publish visual markers to display debug information to RViz.

Kinematics
~~~~~~~~~~~

.. ros:parameter:: kinematics.type str
   :default: "HL"

   Name of the kinematics, see :cpp:class:`navground::core::Kinematics`. Should be one of the registered names.

.. ros:parameter:: kinematics.max_speed float
   :default: 1.0

   Maximal speed, see :cpp:func:`navground::core::Kinematics::get_max_speed`. Should be positive.

.. ros:parameter:: kinematics.max_angular_speed float
   :default: 1.0

   Maximal angular speed, see :cpp:func:`navground::core::Kinematics::get_max_angular_speed`.  Should be positive.

.. ros:parameter:: kinematics.wheel_axis float
   :default: 1.0

   Wheel axis, see :cpp:func:`navground::core::WheeledKinematics::get_axis`.  Should be positive. Only relevant for wheeled kinematics.


Behavior
~~~~~~~~~

.. ros:parameter:: radius float
   :default: 0.0

   The radius of the agent, see :cpp:func:`navground::core::Behavior::get_radius`. Should be positive.

.. ros:parameter:: behavior str
   :default: "HL"
   :dynamic:

   Name of the navigation behavior, see :cpp:class:`navground::core::Behavior`. Should be one of the registered names.

.. ros:parameter:: heading str
   :default: idle
   :dynamic:

   Heading behavior, see :cpp:func:`navground::core::Behavior::get_heading_behavior`. One of "idle", "target_point", "target_angular_speed", or "velocity".

.. ros:parameter:: horizon float
   :default: 1.0
   :dynamic:

   Horizon, see :cpp:func:`navground::core::Behavior::get_horizon`. Should be positive.

.. ros:parameter:: optimal_angular_speed float
   :default: 0.3
   :dynamic:

   Optimal angular speed, see :cpp:func:`navground::core::Behavior::get_optimal_angular_speed`. Should be positive.

.. ros:parameter:: optimal_speed float
   :default: 0.3
   :dynamic:

   Optimal speed, see :cpp:func:`navground::core::Behavior::get_optimal_speed`. Should be positive.

.. ros:parameter:: rotation_tau float
   :default: 0.5
   :dynamic:

   Rotation relaxation time, see :cpp:func:`navground::core::Behavior::get_rotation_tau`. Should be positive.

.. ros:parameter:: safety_margin float
   :default: 0.1
   :dynamic:

   Safety margin, see :cpp:func:`navground::core::Behavior::get_safety_margin`. Should be positive.

HL behavior
"""""""""""

.. ros:parameter:: hl.aperture float
   :default: 3.141592741012
   :dynamic:

   Aperture, see :cpp:func:`navground::core::HLBehavior::get_aperture`. Should be positive.

.. ros:parameter:: hl.eta float
   :default: 0.5
   :dynamic:

   Eta, see :cpp:func:`navground::core::HLBehavior::get_eta`. Should be positive.

.. ros:parameter:: hl.resolution int
   :default: 101
   :dynamic:

   Resolution, see :cpp:func:`navground::core::HLBehavior::get_resolution`. Should be positive.

.. ros:parameter:: hl.tau float
   :default: 0.125
   :dynamic:

   Tau, see :cpp:func:`navground::core::HLBehavior::get_tau`. Should be positive.

ORCA behavior
"""""""""""""

.. ros:parameter:: orca.effective_center bool
   :default: false
   :dynamic:

   Whether to use an effective center, see :cpp:func:`navground::core::ORCABehavior::is_using_effective_center`.

.. ros:parameter:: orca.time_horizon float
   :default: 10.0
   :dynamic:

   Time horizon, see :cpp:func:`navground::core::ORCABehavior::get_time_horizon`. Should be positive.

Other behaviors
"""""""""""""""

All properties of registered behavior are exposed as parameters

.. ros:parameter:: <behavior_name>.<property_name> <property_type>
   :default: <default value of the property>
   :dynamic:


Controller
~~~~~~~~~~

.. ros:parameter:: speed_tolerance float
   :default: 0.05
   :dynamic:

   Speed below of which the agent is considered as stopped, see :cpp:func:`navground::core::Controller::get_speed_tolerance`. Should be positive.

Vertical motion
"""""""""""""""

.. ros:parameter:: altitude.enabled bool
   :default: false

   Whether to consider vertical information for state and control, see :cpp:func:`navground::core::Controller3::is_limited_to_2d`.

.. ros:parameter:: altitude.optimal_speed float
   :default: 0.1

   Optimal speed of the vertical motion, see :cpp:func:`navground::core::Controller3::get_altitude_optimal_speed`. Should be positive.

.. ros:parameter:: altitude.tau float
   :default: 1.0
   :dynamic:

   Relaxation time of the vertical motion, see :cpp:func:`navground::core::Controller3::get_altitude_tau`. Should be positive.


Subscriptions
-------------

State
~~~~~

.. ros:subscription:: odom nav_msgs/Odometry

   Own odometry is used to set the behavior state, see :cpp:func:`navground::core::Behavior::set_pose` and :cpp:func:`navground::core::Behavior::set_twist`.

.. ros:subscription:: neighbors navground_msgs/Neighbors

   Dynamic Neighbors. Used by behaviors that have a :cpp:struct:`geometric environment state <navground::core::GeometricState>`, see :cpp:func:`navground::core::GeometricState::set_neighbors`.

.. ros:subscription:: obstacles navground_msgs/Obstacles

   Static obstacles. Used by behaviors that have a :cpp:struct:`geometric environment state <navground::core::GeometricState>`, see :cpp:func:`navground::core::GeometricState::set_static_obstacles`.

Target
~~~~~~

.. ros:subscription:: target_point geometry_msgs/PointStamped

   Set a target point. If an action is being executed, aborts it if is not following a point/pose, else updates the target.

.. ros:subscription:: target_pose geometry_msgs/PoseStamped

   Set a target pose. If an action is being executed, aborts it if is not following a point/pose, else updates the target.

.. ros:subscription:: target_twist geometry_msgs/TwistStamped

   Set a target twist. If an action is being executed, aborts it if is not following a twist, else updates the target.

.. ros:subscription:: stop std_msgs/Empty

   Makes the agent stop. Aborts any action that is being executed.


Action Servers
---------------

.. ros:action_server:: go_to navground_msgs/GoToTarget

   Initiate an action towards a target pose or position, see :cpp:func:`navground::core::Controller::go_to_position` and :cpp:func:`navground::core::Controller::go_to_pose`.
   Ignores new requests if an action is already being executed.


Publishers
----------

.. ros:publisher:: cmd_vel geometry_msgs/Twist

   Publishes the command returned by :cpp:func:`navground::core::Controller3::update_3d`.
   Only published if :ros:param:`publish_cmd_stamped` is not set.

.. ros:publisher:: cmd_vel_stamped geometry_msgs/TwistStamped

   Publishes the command returned by :cpp:func:`navground::core::Controller3::update_3d`.
   Only published if :ros:param:`publish_cmd_stamped` is set.












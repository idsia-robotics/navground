==========
Components
==========

Navground divide functionality into six main abstract *components*, for which it also provides concrete sub-class listed in this section, together with examples and videos.

The different type of components share some common functionality:

- they keep a register of concrete sub-classes,
- they have named parameters (*properties*),
- they can be represented through YAML,
- they can be :doc:`extended <../guides/extend/index>` using plugins.

The core library defines :doc:`behaviors/index`, :doc:`behavior_modulations/index`, and :doc:`kinematics/index`, which can be used with simulated *and* real-world agents.
The simulation library adds :doc:`state_estimations/index`, :doc:`tasks/index`, :doc:`scenarios/index` which are specific for simulations.

You can list all components installed in your system using

.. code-block:: console

   $ navground_py info 

and the subset of components implemented in C++ using

.. code-block:: console

   $ navground info 

.. seealso::

   :doc:`../reference/index`
      The components's API (C++, Python, and YAML)

   :doc:`../guides/extend/index`
      How to extend navground with new implementations in C++ or Python.


.. toctree::
   :maxdepth: 2

   behaviors/index
   behavior_modulations/index
   kinematics/index
   state_estimations/index
   tasks/index
   scenarios/index

   

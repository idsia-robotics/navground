=========
Scenarios
=========

Override ``init_world`` (:cpp:func:`C++ <navground::sim::Scenario::init_world>`, :py:meth:`Python <navground.sim.Scenario.init_world>`) with the logic to initialize a world: create/modify/delete entities, set termination criteria, set the bounding box, setup a lattice, ...).
If you need to sample from random distribution, use the provided random generator (:cpp:func:`C++ <navground::sim::World::get_random_generator>`, :py:data:`Python <navground.sim.World.random_generator>`), that will guarantee that the world initialization will be deterministic.


Virtual methods
===============

.. list-table::
   :widths: 30 30 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`init_world <navground::sim::Scenario::init_world>` 
     - :py:meth:`init_world <navground.sim.Scenario.init_world>` 
     - should


Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: scenario.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: scenario.py
         :language: Python


.. _groups:

Groups
======

Another way to customize a scenario, without subclassing, is to add a group of agent, i.e., a sub-class of ``Group`` (:cpp:class:`C++ <navground::sim::Scenario::Group>`, :py:class:`Python <navground.sim.Scenario.Group>`), that has a virtual method ``add_to_world``

.. list-table::
   :widths: 30 30 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`add_to_world <navground::sim::Scenario::Group::add_to_world>` 
     - :py:meth:`add_to_world <navground.sim.Scenario.Group.add_to_world>` 
     - must

which should spawn the group to the world, but is not restricted to that.
if required, define a new type of group

.. tabs::

   .. tab:: C++

      .. code-block:: C++

         #include "navground/sim/scenario.h"

         namespace sim = navground::sim;

         struct MyGroup : public sim::Scenario::Group {
           // add here the logic to populate the world
           void add_to_world(sim::World *world, seed = std::nullopt) override {
             // use the world random generator to sample random variable. 
             auto & rng = world->get_random_generator();
             // manipulate the world, in particular create and add agents
             // but you are not restricted to that.
           }
         };

   .. tab:: Python

      from navground import sim

      .. code-block:: Python

         class  MyGroup(core.PyGroup):
             
             def add_to_world(self, world: sim.World) -> None:
                 # use the world random generator to sample random variable.
                 rng = world.random_generator
                 # manipulate the world, in particular create and add agents
                 # but you are not restricted to that.


Then, you add the group to your scenario
  
.. tabs::

   .. tab:: C++

      .. code-block:: C++

         MyScenario scenario;
         scenario.groups.push_back(std::make_shared<MyGroup>());
         // now scenario.init_world(...) will also call this group ``add_to_world``

   .. tab:: Python

      scenario = MyScenario()
      scenario.groups.append(MyGroup())
      # now scenario.init_world(...) will also call this group ``add_to_world``


.. note::

   This is a weaker way to extend a scenario than registering a sub-class as the new groups will not be exposed to YAML.


.. _initializers:

Initializers
============

Similar to groups, you can add an initializer (:cpp:func:`C++ <navground::sim::Scenario::add_init>`, :py:meth:`Python <navground.sim.Scenario.add_init>`) to your scenario, which is a procedure taking a mutable world at argument.


.. tabs::

   .. tab:: C++

      .. code-block:: C++

         MyScenario scenario;
         scenario.add_init([](World * world, std::optional<unsigned> seed) {
            ...
         });
         // now scenario.init_world(...) will also call this function.

   .. tab:: Python

      .. code-block:: python
      
         scenario = MyScenario()
         scenario.add_init(lambda world, seed: ...)
         # now scenario.init_world(...) will also call this function

.. note::
   
   Like for groups, initializers are not exposed to YAML and must therefore be added through the API.
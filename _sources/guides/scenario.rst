=========
Scenarios
=========

:cpp:func:`navground::sim::Scenario::init_world`

:cpp:func:`navground::sim::World::get_random_generator`


.. code-block:: c++

   #include "navground/sim/scenario.h"

   namespace sim = navground::sim;

   struct MyScenario : public sim::Scenario {
     // add here the logic to initialize the world
     void init_world(sim::World *world, seed = std::nullopt) override {
       // call the super class: when the scenario is configured thought YAML, 
       // it will add agents and obstacles as specified in the configuration.
       sim::Scenario::init_world(world, seed);
       // use the world random generator to sample random variable. 
       auto & rng = world->get_random_generator();
       // manipulate the world: create/add/modify/delete agents and obstacles
     }
   };

:py:meth:`navground.sim.Scenario.init_world`
:py:meth:`navground.sim.World.random_generator`

.. code-block:: python

   from navground import sim

   class MyScenario(sim.PyScenario):

       # add here the logic to initialize the world
       def init_world(word: sim.World, seed: int | None = None) -> None:
           # call the super class: when the scenario is configured thought YAML, 
           # it will add agents and obstacles as specified in the configuration.
           super().init_world(world, seed)
           # use the world random generator to sample random variable. 
           rng = world.random_generator
           # manipulate the world: create/add/modify/delete agents and obstacles


Groups
======

:cpp:func:`navground::sim::Scenario::Group`
:cpp:func:`navground::sim::Scenario::Group::add_to_world`

.. code-block:: c++

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

:py:meth:`navground.sim.Scenario.Group`
:py:meth:`navground.sim.Scenario.Group.add_to_world`

.. code-block:: python

   from navground import sim

   class MyGroup(sim.Group):

       # add here the logic to initialize the world
       def add_to_world(word: sim.World, seed: int | None = None) -> None:
           # use the world random generator to sample random variable. 
           rng = world.random_generator
           # manipulate the world, in particular create and add agents
           # but you are not restricted to that


Then you can add the group to your scenario
  
:cpp:member:`navground::sim::Scenario::groups`

.. code-block:: c++

   MyScenario scenario;
   scenario.push_back(std::make_shared<MyGroup>());
   // now scenario.init_world(...) will also call this group ``add_to_world``


:py:attr:`navground.sim.Scenario.groups`

.. code-block:: python

   scenario = MyScenario()
   scenario.groups.append(MyGroup())
   # now scenario.init_world(...) will also call this group ``add_to_world``

.. todo:: tell them that this is a weaker way to extend as not exposed to YAML nor registered.


Initializers
============

Then you can add the group to your scenario
  
:cpp:member:`navground::sim::Scenario::add_init`

.. code-block:: c++

   MyScenario scenario;
   scenario.add_init([](World * world, std::optional<unsigned> seed) {
   ...
   });
   // now scenario.init_world(...) will also call this function.

:py:meth:`navground.sim.Scenario.add_init`

.. code-block:: python

   scenario = MyScenario()
   scenario.add_init(lambda world, seed: ...)
   # now scenario.init_world(...) will also call this function


.. todo:: tell them that this is a similar to a group but without a class (or just a callable)


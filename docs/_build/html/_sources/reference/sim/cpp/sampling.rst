=========
Sampling
=========

Samplers
========

.. code-block:: cpp
   
   #include "navground/sim/sampling/sampler.h"

.. doxygenfunction:: navground::sim::set_random_seed

.. doxygenfunction:: navground::sim::get_random_seed

.. doxygenfunction:: navground::sim::random_generator

Base class
----------

.. doxygenstruct:: navground::sim::Sampler
    :members:

.. doxygenenum:: navground::sim::Wrap


Generic
-------

.. doxygenstruct:: navground::sim::ConstantSampler
    :members:

.. doxygenstruct:: navground::sim::SequenceSampler
    :members:

.. doxygenstruct:: navground::sim::ChoiceSampler
    :members:

Numbers and Vectors
-------------------

.. doxygenstruct:: navground::sim::RegularSampler
    :members:

Vectors
-------

.. doxygenstruct:: navground::sim::GridSampler
    :members:

Numbers
-------

.. doxygenstruct:: navground::sim::UniformSampler
    :members:

.. doxygenstruct:: navground::sim::NormalSampler
    :members:

Properties
----------

.. doxygenstruct:: navground::sim::PropertySampler
    :members:

Registered components
=====================

Base class
----------

.. doxygenstruct:: navground::sim::SamplerFromRegister
    :members:

Registers
---------

.. doxygenstruct:: navground::sim::BehaviorSampler
    :members:
    :undoc-members:

.. doxygenstruct:: navground::sim::KinematicsSampler
    :members:
    :undoc-members:

.. doxygentypedef:: navground::sim::TaskSampler

.. doxygentypedef:: navground::sim::StateEstimationSampler

Agents
======

.. doxygenstruct:: navground::sim::AgentSampler
    :members:
    :undoc-members:

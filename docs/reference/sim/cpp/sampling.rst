=========
Sampling
=========

Samplers
========

.. code-block:: cpp
   
   #include "navground/sim/sampling/sampler.h"


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

.. doxygenstruct:: navground::sim::TaskSampler
    :members:
    :undoc-members:

.. doxygenstruct:: navground::sim::StateEstimationSampler
    :members:
    :undoc-members:

Agents
======

.. doxygenstruct:: navground::sim::AgentSampler
    :members:
    :undoc-members:


Obstacles
=========

.. code-block:: cpp
   
   #include "navground/sim/sampling/geometry.h"

.. doxygenfunction:: navground::sim::sample_discs

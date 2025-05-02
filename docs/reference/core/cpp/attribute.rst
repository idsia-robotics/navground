.. _attributes cpp:

==========
Attributes
==========

Attributes are similar to properties but are dynamic, i.e., they are not specified by the class and can change type. Like properties, values are restricted to the types in :cpp:type:`navground::core::Attribute`.
Classes with attributes save/load their attributes from YAML.

.. code-block:: cpp
   
   #include "navground/core/attribute.h"

.. doxygentypedef:: navground::core::Attribute

.. doxygentypedef:: navground::core::Attributes

.. doxygenstruct:: navground::core::HasAttributes
   :members:

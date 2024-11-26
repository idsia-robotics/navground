=================
Auto registration
=================

Sub-classes with properties and registers share a YAML field ``type: string``
that should match the registered type name. 
Loading fails if no registered sub-class matches the field value.
 
Moreover, they expose all their properties. 
When loading an object, specifying it's properties is optional: 
when non provided, they are set to the property :cpp:member:`navground::core::Property::default_value`.

When loading a property from YAML field, their types should be compatible. For instance, loading a list of floats
from a YAML bool will fails. Properties are not included in the schema but are represented by the line

.. code-block:: yaml

   additionalProperties: {}

that accepts any property of any value.

.. note::

   This apply to user defined components too. Any property the expose it automatically available to YAML too.

Example
^^^^^^^

The :cpp:class:`navground::core::HLBehavior` has four properties --- ``tau`` (float), ``eta`` (float), ``aperture`` (float), ``resolution`` (int) --- that can be read from the YAML. Loading the following YAML

.. code-block:: yaml

  type: HL
  tau: 0.25
  aperture: 1.0
  # this would fail, as types float for the property and
  # sequence of numbers for YAML are not compatible
  # eta: [1.0]

sets ``tau=0.25`` and ``aperture=1.0``.



   

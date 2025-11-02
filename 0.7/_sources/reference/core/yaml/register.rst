=====================
Registered components
=====================

The YAML representation of components with a register, i.e., :doc:`behavior`, :doc:`behavior_modulation`, and :doc:`kinematics`, are objects with a field ``type`` that corresponds to the name associated to the registered property:

.. code-block:: yaml

   $id: http://navground/<component>
   $schema: https://json-schema.org/draft/2020-12/schema
   type: object
   # properties of the base class
   properties:
      type: string
      ...
   # specific registered sub-classes 
   $ref: /schemas/<component>_register
   unevaluatedProperties: false

.. warning:: 

   Loading will fail if no registered sub-class matches the specified type.

Moreover, registered sub-classes included with ``$ref: <componet>_register`` contain all their (navground) properties. 

.. code-block:: yaml

   $id: http://navground/<component>_register
   $schema: https://json-schema.org/draft/2020-12/schema
   # list of registered sub-classes
   anyOf:
      - properties:
          type:
            const: <type>
          <property>:
            type: ...
            default: ...
          ...
      - ...
  
When loading an object from YAML, specifying its properties is optional: when non provided, they are set to ``default``. When loading properties from YAML, their types should be compatible. For instance, loading a list of floats from a YAML bool will fails.

.. note::

   This applies as well to user defined sub-classes: any property the expose it automatically available to YAML too and be included in the schema.

Example
^^^^^^^

The :cpp:class:`navground::core::HLBehavior` has properties --- ``tau`` (float), ``eta`` (float), ``aperture`` (float), ``resolution`` (int) --- that can be read from the YAML. Loading the following YAML

.. code-block:: yaml

   type: HL
   tau: 0.25
   aperture: 1.0
   # this would fail, as types float for the property and
   # sequence of numbers for YAML are not compatible
   # eta: [1.0]

sets ``tau=0.25`` and ``aperture=1.0``.



   

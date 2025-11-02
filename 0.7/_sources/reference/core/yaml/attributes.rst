==========
Attributes
==========

The YAML representation of classes with attributes contains a field ``attributes`` with a map of all attributes, if not empty.
Each attribute lists its value and type.

Schema
======

.. schema:: navground.core.schema.bundle()['$defs']['attributes']

Example
=======

.. code-block:: yaml

   key: 
     value: 1
     type: int
   another_key:
     value: ['core', 'sim']
     type: '[str]' 



   

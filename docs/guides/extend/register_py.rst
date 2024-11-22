===================================
How to integrate a Python component
===================================

You have developed a new navground component in Python (i.e., one of :py:class:`navground.core.Behavior`, :py:class:`navground.core.BehaviorModulation`, :py:class:`navground.core.Kinematics`, :py:class:`navground.sim.Scenario`, :py:class:`navground.sim.StateEstimation`, or :py:class:`navground.sim.Task`).

.. code-block:: python

   from navground import core
   

   class MyComponent(PyComponent):
      ...

You can already use as it is but if you *register* it, you will integrate it better  
in the navground API and CLI, and you will simplify its usage by others.

For example, you have developed a new navigation behavior called "MyBehavior" (no trivial task, congrats!) and you want to run experiments to compare its performance with other navigation algorithms (which is actually one of the main design goals of navground).

After

- registering the type  
- exposing as a property any parameter to be configure from YAML 
- specialize the YAML encoding/decoding if the basics functionality is not sufficient
- installing it as a plugin

you can use the behavior in an experiment, e.g., configured for like this

.. code-block:: yaml

   scenario:
      type: Cross
      groups:
        - number: 20
          behavior: 
            type: MyBehavior
            my_param: 1
          kinematics: ...
          state_estimation: ...


Each step adds functionality, summarized in the table below, that is not needed to compile or use the component but that provides a more complete integration with navground.



+---------------------+-------------------------------+----------------------------------------------+
|         add         |               to              |                   example                    |
+=====================+===============================+==============================================+
| `Py Register type`_ | instantiate by name           | ``comp = Component.make_type("MyName")``     |
+---------------------+-------------------------------+----------------------------------------------+
| `Py Properties`_    | access parameters by name     | ``comp.set("value", 1)``                     |
+---------------------+-------------------------------+----------------------------------------------+
| `Py YAML`_          | customize the YAML conversion | ``comp = YAML.load_<component>()``           |
+---------------------+-------------------------------+----------------------------------------------+
| `Py Plugin`_        | share the extension           | ``core.load_plugins()``                      |
+---------------------+-------------------------------+----------------------------------------------+


.. _Py Register type: 

Register the type
=================

Register your component to the base class register by adding ``name="MyName"`` to the class definition, to be able to instantiate it by name

.. code-block:: python
   
   class MyComponent(Component, name="MyName"):
      ...


Once a class has been registered, it can be instantiated using a generic factory method ``Component.make_type`` by providing its name:

.. code-block:: python

   c = Component.make_type("MyName")
   # c.type returns "MyName"

Moreover, the type will also appear in the YAML representation and 

.. code-block:: python
   
   core.dump(c)
   
as field "type"

.. code-block:: yaml

   type: MyName
   ...

and it will be possible to load the component from yaml

.. code-block:: python

   c = core.load_<component>(c)


.. _Py Properties: 

Define Properties
=================

Add a Python property for any parameter you want to expose. 
Below :py:obj:`property`, add the :py:meth:`navground.core.register` decorator
with the default value and an optional description


.. code-block:: python

   class MyComponent(Component, name="MyName"):
      
      @property
      @core.register(True, "my description")
      def my_param(self) -> bool:
         return True

In the trivial example above, the property returns a constant value and has no setter. In general, properties will be get/set attributes of the class, like

.. code-block:: python

   class MyComponent(Component):
      
      def __init__(self):
         self._value = True

      @property
      @core.register(True, "my description")
      def value(self) -> bool:
          return self._value
      
      @value.setter
      def value(self, value: bool) -> None:
          self._value = value   


Once properties are registered, the class gains generic accessors ``get`` and ``set`` that uses names to identify properties.

.. code-block:: python

   c = MyComponent()
   value = c.get("value")
   c.set("value", not value)

Moreover, properties will also appear in the YAML representation

.. code-block:: python
   
   core.dump_<component>(c)
      
as additional fields
   
.. code-block:: yaml

   ...
   value: false
   ...

.. note::

   When working with components defined in Python, navground properties are not very useful, as you could directly inspect the Python class and use its accessors, or even generic accessors like :py:func:`getattr` and :py:func:`setattr`. Instead, when the component is loaded from YAML or C++, properties offer a generic way to access to instance attributes.


.. _Py YAML:

YAML 
====

In case the conversion from/to YAML provided by navground is not sufficient, specialize the methods ``encode`` and ``decode``. There is no need to call the base implementation as it is empty.

.. code-block:: python

   class MyComponent(Component, name="MyName"):
      
      # ... properties

      def encode(self) -> str:
         ...
      def decode(self, yaml: str) -> None:
         ...

Through these methods you can read more complex parameters from the YAML than :py:type:`navground.core.PropertyField`. For example, you can configure a value of type ``dict[str, int]`` from a YAML such as

.. code-block:: yaml

   my_complex_param:
      a: 1
      b: false

if you implement the custom logic in the decoder and the encoder, for example, like

.. code-block:: python

   class MyComponent(Component, name="MyName"):
      
       def encode(node: dict[str, typing.Any]) -> None:
           node['my_complex_param'] = {
               'a': self.my_int_a, 
               'b': self.my_bool_b
           }
         
       def decode(node: dict[str, typing.Any]) -> None:
           if 'my_complex_param' in node:
               param = node['my_complex_param']
               if 'a' in param:
                   self.my_int_a = int(param['a'])
               if 'b' in param:
                   self.my_bool_a = bool(param['b'])


.. warning::

   Properties are treated as random variables in a navground scenario. For example:

   .. code-block:: yaml
  
      scenario:
        groups:
          - number: 10
            behavior:
              type: MyBehavior 
              my_param:
                sampler: uniform
                from: 1
                to: 2

   defines a group of agents whose behavior "my_param" parameter has a random value. 
   This does *not* extend to parameters read using custom YAML decoders. 
   In case this is required, users will need to implement this logic in a scenario.

   Therefore, we suggest to restrict parameters exposed to YAML to properties, so to get
   the treatment as random variable for free. 


Schema
------

If your class defines a custom YAML representation, it should also register the related JSON-schema, as a function of type :py:type:`typing.Callable[[dict[str, typing.Any]], None]` that modify the default schema of the class.

In the example above, we add the appropriate schema

.. code-block:: python

   class MyComponent(Component, name="MyName"):
      
       @core.register_schema
       def schema(node: dict[str, typing.Any]) -> None:
           my_complex_param = {
               'type': 'object',
               'properties': {
                   'a': {
                       'type': 'integer'
                   },
                   'b': {
                       'type': 'boolean'
                   }
               },
               'additionalProperties': False
           }
           if not node["properties"]:
               node["properties"] = {}
           node["properties"]["my_complex_param"] = my_complex_param


Class skelethon
================

Using the appropriate macro, the class skeleton simplifies to


.. code-block:: python

   class MyComponent(Component, name="MyName"):
      
      @property
      @core.register(default_value, "description")
      def name(self) -> Type:
          return ...
      
      @name.setter
      def name(self, value: Type) -> None:
          ...       

      # def encode(self) -> str: ...

      # def decode(self, yaml: str) -> None: ...
      
      # @core.register_schema
      # def schema(node: dict[str, typing.Any]) -> None: ...


.. _Py Plugin: 

Install as a plugin
===================

This is a install-time step. Wraps one or more components in a shared library and install it as a plugin to integrate it in the navground CLI and API.


Define an entry point for each component you want to export in the ``setup.cfg`` or ``setup.py`` file of the package.

For example, to install behavior ``MyBehavior``, add 

.. code-block::

   [options.entry_points]
   navground_behaviors = 
       my_behavior = <my_packages>.<my_module>:MyBehavior

to your ``setup.cfg``. The actual key "my_behavior" is currently ignored.

.. note::
    
   Following end-points are available ``navground_behaviors``, ``navground_kinematics``, ``navground_modulations``, ``navground_tasks``, 
   ``navground_state_estimations``, and ``navground_scenarios`` to install components of the respective type.

Once installed, the behavior will be automatically discovered when calling :py:func:`navground.core.load_plugins`.

.. code-block:: python

   >>> from navground import core
   >>> core.load_plugins()
   >>> print(core.Behavior.types)

   [..., 'MyBehavior']

   >>> behavior = core.Behavior.make_type("MyBehavior")
   <my_packages>.<my_module>.MyBehavior object ...>


.. note::

   Calling :py:func:`navground.core.load_plugins`, C++ plugins are imported too and can then be instantiated from Python.


Complete example
================

See :ref:`Python example <py_component_example>` for an example where we implement and register a new (dummy) navigation behavior in Python.

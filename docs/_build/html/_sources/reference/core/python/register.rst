========
Register
========

For each C++ template class :cpp:class:`navground::core::HasRegister`, we expose a Python class
with name ending by ``Register``, which serves as super-classes for their respective ``T`` class. 

Python sub-classes of ``T`` by adding ``name=<name>`` to the class, where the ``<name>`` is an arbitrary string that will index the sub-class in the register. For example,

.. code-block:: python

   SubClassOfT(T, name="SubClassOfT"):
      ...

will register type ``SubClassOfT`` under the name ``"SubClassOfT"``.


.. autodecorator:: navground.core.register


.. autoclass:: navground.core._navground.BehaviorRegister
   :exclude-members: __new__, __init__

   This register holds any behavior registered in :cpp:class:`navground::core::HasRegister<Behavior>` as well
   as registered Python sub-classes of :py:class:`navground.Behavior`.


   .. py::method:: make_type(name: str) -> Behavior

      Create a behavior of a sub-class selected by name.

      :param type:
         The associated type name.

      :return:
         A behavior from a registered sub-class or ``None`` in case the desired name is not found.

   .. py:property:: types
      :classmethod:
      :type: List[str]

      The names of all registered behaviors

   .. py:property:: type_properties
      :classmethod:
      :type: Dict[str, Dict[str, Property]]

      The dictionary of registered properties with registered names as keys


.. autoclass:: navground.core._navground.KinematicsRegister
   :exclude-members: __new__, __init__

   This register holds any kinematics registered in :cpp:class:`navground::core::HasRegister<Kinematics>` as well
   as registered Python sub-classes of :py:class:`navground.Kinematics`.

   .. py::method:: make_type(name: str) -> Kinematics

      Create a kinematics of a sub-class selected by name.

      :param type:
         The associated type name.

      :return:
         A kinematics from a registered sub-class or ``None`` in case the desired name is not found.

   .. py:property:: types
      :classmethod:
      :type: List[str]

      The names of all registered kinematics.

   .. py:property:: type_properties
      :classmethod:
      :type: Dict[str, Dict[str, Property]]

      The dictionary of registered properties with registered names as keys

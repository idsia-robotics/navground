===========================
How to register a component
===========================

Source code
===========

C++
---

Registering a C++ sub-class ``MyClass`` in the parent class ``T``  register, requires to

#. Optionally, in case ``MyClass`` should exposed properties, register all properties.

   #. add getters and setters for all properties, e.g.:

      .. code-block:: c++

         bool get_my_property() const { return _my_field; }
         void set_my_property(bool value) { _my_field = value; }

   #. expose the properties, overriding at the same time 
      :cpp:func:`navground::core::HasProperties::get_properties`

      .. code-block:: c++

         const Properties &get_properties() const override { return properties; };
         inline const static std::map<std::string, Property> properties = Properties{
             {"my_property", make_property<bool, MyClass>(
                                 &MyClass::get_my_property, &MyClass::set_my_property,
                                 False, "my property description")}};


#. override :cpp:func:`navground::core::HasRegister::get_type`

   .. code-block:: c++
        
      std::string get_type() const override { return type; }
    

#. call :cpp:func:`navground::core::HasRegister::register_type` 

   .. code-block:: c++

      static inline const std::string type = register_type<MyClass>("MyClass");


Complete example
^^^^^^^^^^^^^^^^

.. code-block:: c++
    :linenos:

    class MyClass(T) {
     public:
      MyClass() : T(), _my_field(False) {}
      bool get_my_property() const { return _my_field; }
      void set_my_property(bool value) { _my_field = value; }
      const Properties &get_properties() const override { return properties; };
      inline const static std::map<std::string, Property> properties = Properties{
          {"my_property", make_property<bool, MyClass>(
                              &MyClass::get_my_property, &MyClass::set_my_property,
                              False, "my property description")}};
      std::string get_type() const override { return type; }
      static inline const std::string type = register_type<MyClass>("MyClass");
    
     private:
      bool _my_field;
    };


Python
------

Registering a Python sub-class ``MyClass`` in the parent class ``T``  register, requires to

#. Optionally, in case ``MyClass`` should exposed properties, register all properties using :py:func:`navground.core.register`:

   .. code-block:: python
   
      @property
      @register(False, "my property description")
      def my_property(self) -> bool:
          return self._my_field;
    
      @my_property.setter
      def my_property(self, value: bool) -> None:
          self._my_field = value;

#. add ``name="..."`` to the class definition

   .. code-block:: python

      class MyClass(T, name="MyClass"):
          ...

Complete example
^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from navground.core import register
    
    
    class MyClass(T, name="MyClass"):
    
        def __init__(self):
            super().__init__()
            self._my_field = False
    
        @property
        @register(False, "my property description")
        def my_property(self) -> bool:
            return self._my_field
       
        @my_property.setter
        def my_property(self, value: bool) -> None:
            self._my_field = value


Installation
============

If the registered component has to be discoverable outside of the code where it is defined, like when you want to run an experiment using it, you have to install the component so that it can be discovered.

.. 
   warning::

    Currently installing is only supported for Python components. 
    Support for C++ components is in progress. 


C++
---

Wrap the extensions in one or more shared libraries. Then, 
in `CMakeLists.txt`, add an entry to the ament index ``navground_plugins`` with
the paths of the installed libraries.

For example, to build and install behavior ``MyBehavior`` implemented in file ``my_behavior.cpp``, add

.. code-block:: cmake

   add_library(my_behavior SHARED my_behavior.cpp)
   ament_target_dependencies(my_behavior navground_core)
   ament_export_dependencies(my_behavior)
   ament_export_targets(my_behaviorTargets HAS_LIBRARY_TARGET)
   ament_index_register_resource(navground_plugins CONTENT $<TARGET_FILE:my_behavior>)  

   install(
     TARGETS my_behavior
     EXPORT my_behaviorTargets
     LIBRARY DESTINATION lib/${PROJECT_NAME}
     ARCHIVE DESTINATION lib/${PROJECT_NAME}
     RUNTIME DESTINATION bin
     INCLUDES DESTINATION include/${PROJECT_NAME}
   )

Then, the behavior will be automatically discovered when calling :cpp:func:`navground::core::load_plugins`.

.. code-block:: C++

   #include "navground/core/plugins.h"
   #include "navground/core/behavior.h"

   int main(int argc, char* argv[]) {
      navground::core::load_plugins();
      auto behavior = navground::core::Behavior::make_type("MyBehavior");
   }

   
Python
------

Define an entry for each component you want to export in the ``setup.cfg`` or ``setup.py`` file of the package.

For example, to install behavior ``MyBehavior``, add 

.. code-block::

   [options.entry_points]
   navground_behaviors = 
       my_behavior = <my_packages>.<my_module>:MyBehavior

to your ``setup.cfg``. The name ``my_behavior`` is currently ignored.

.. note::
    
   Following end-points are available ``navground_behaviors``, ``navground_kinematics``, ``navground_tasks``, 
   ``navground_state_estimations``, and ``navground_scenarios`` to install components of the respective type.



Then, the behavior will be automatically discovered when calling :py:func:`navground.core.load_plugins`.

.. code-block:: python

   >>> from navground import core
   >>> core.load_plugins()
   >>> print(core.Behavior.types)

   [..., 'MyBehavior']

   >>> behavior = core.Behavior.make_type("MyBehavior")
   <my_packages>.<my_module>.MyBehavior object ...>


.. note::

   Using :py:func:`navground.core.load_plugins`, C++ plugins are imported too and available to use from Python.



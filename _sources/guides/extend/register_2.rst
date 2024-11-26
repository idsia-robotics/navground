================================
How to integrate a C++ component
================================

.. see https://github.com/SublimeText-Markdown/TableEditor

You have developed a new navground component in C++ (i.e., one of :cpp:class:`navground::core::Behavior`, :cpp:class:`navground::core::BehaviorModulation`, :cpp:class:`navground::core::Kinematics`, :cpp:class:`navground::sim::Scenario`, :cpp:class:`navground::sim::StateEstimation`, or :cpp:class:`navground::sim::Task`).

.. code-block:: c++

   #include "navground/core/<component>.h"
   
   namespace core = navground::core;
   
   struct MyComponent : public Component {
     // ...
   };

You can already use as it is but if you *register* it, you will integrate it better  
in the navground API and CLI, and you will simplify its usage by others.

For example, you have developed a new navigation behavior called "MyBehavior" (no trivial task, congrats!) and you want to run experiments to compare its performance with other navigation algorithms (which is actually one of the main design goals of navground).

After

- exposing as a property any parameter to be configure from YAML 
- registering the type  
- adding introspection
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



+------------------+-------------------------------+-------------------------------------------------+
|       add        |               to              |                     example                     |
+==================+===============================+=================================================+
| `Properties`_    | access parameters by name     | ``comp.set("value", 1);``                       |
+------------------+-------------------------------+-------------------------------------------------+
| `Register type`_ | instantiate by name           | ``auto comp = Component::make_type("MyName");`` |
+------------------+-------------------------------+-------------------------------------------------+
| `Introspection`_ | inspect                       | ``comp.get_type();``                            |
+------------------+-------------------------------+-------------------------------------------------+
| `YAML`_          | customize the YAML conversion | ``auto comp = YAML::load<Component>();          |
+------------------+-------------------------------+-------------------------------------------------+
| `Plugin`_        | share the extension           | ``load_plugins();``                             |
+------------------+-------------------------------+-------------------------------------------------+


.. _Properties: 

Define Properties
=================

Add to ``MyComponent::properties`` any parameter you want to expose.
Configure the property with a type, a default value, and an optional description 
and a pair of functions to get/set the value of the parameter.

In practice, you should set the class variable :cpp:var:`navground::core::HasProperties::properties`
to a map of named properties created with :cpp:func:`navground::core::make_property`, like:

.. code-block:: c++

   struct MyComponent : public Component {
     // ... 
     inline static const  core::Properties properties{
         {"my_param", core::make_property<int, MyComponent>(
                          [](const MyComponent *) { return 1; }, nullptr, 1,
                          "my description")}};
   };

In the trivial example above, the property uses a lambda function for the getter that returns
a constant value, and has no setter. In general, properties may use getters and setters to attributes of the class, like

.. code-block:: c++

   struct MyComponent : public Component {
     // ...
     bool get_value() const { return _value; }
     void set_value(bool value) { _value = value; }
     inline static const  core::Properties properties{
         {"value", core::make_property<bool, MyComponent>(
                       &MyComponent::get_value, &MyComponent::set_value, true,
                       "my description")}};
   
   private:
     bool _value;
   };


In a more realistic example, the class definition would be separated from the implementation where setting ``properties`` would go, like


.. code-block:: c++

   // declaration

   struct MyComponent : public Component {
     // ...
     static const core::Properties properties;
   };

   // definition

   const core::Properties properties = core::Properties{...};


Once properties are registered, the class gains generic accessors :cpp:func:`navground::core::HasProperties::get`, :cpp:func:`navground::core::HasProperties::set`, and :cpp:func:`navground::core::HasProperties::set_value`, that uses names to identify properties.

.. code-block:: c++

   MyComponent c;
   bool value = c.get("value").get<bool>();
   c.set("value", !value);

.. note::

   The main advantages of exposing properties are realized only once the component 
   is registered. In fact, if you have access to the class definition, being able to get/set properties by name does not bring much as you could directly use the class own accessors. Instead, when you don't have access to the class definition, like when loading from a shared library or using it from Python, properties allows a generic way to access to instance attributes (from C++, Python, and YAML).


.. _Register type: 

Register the type
=================

Register your component to the base class register using :cpp:func:`navground::core::HasRegister::register_type` to be able to instantiate it by name.


.. code-block:: c++
   
   struct MyComponent : public Component {
     // ... 
     // ... properties
   
     inline static const std::string type = register_type<MyComponent>("MyName");
   };

Similar to `Properties`_, in a more realistic example, you would register the class in the  implementation, like:

.. code-block:: c++

   // declaration

   struct MyComponent : public Component {
     // ... 
     // ... properties
     static const std::string type;
   };

   /// definition

   const std::string type = register_type<MyComponent>("MyName");


Once a class has been registered, it can be instantiated using a generic factory method :cpp:func:`navground::core::HasRegister::make_type` by providing its name:

.. code-block:: c++

   std::shared_ptr<Component> c = Component::make_type("MyName");




.. _Introspection:

Introspection
=============

Override the type and properties accessors to let instances be aware of their type and properties. This step is not required when you create a component in Python.


.. code-block:: c++

   struct MyComponent : public Component {
     // ...
     // ... properties
     // ... type
     std::string get_type() const override { return type; }
     const navground::core::Properties &get_properties() const override {
       return properties;  
     }
   };


Then, you can use them as expected:

.. code-block:: c++

   MyComponent c;
   // Same as MyComponent::type
   c.get_type();
   // Same as MyComponent::properties
   c.get_properties();

.. note::

   This is useful if you are working with components whose type we don't know at compile-time.
   For instance, when the component are loaded from YAML:

   .. code-block:: c++

      auto c = YAML::load<Component>("...");
      // We loaded a component of type ...
      c->get_type();

Moreover, type and properties will also appear in the YAML representation

.. code-block:: c++
   
   YAML::dump<Component>(&c);
      
as additional fields (one for each property)
   
.. code-block:: yaml

   type: MyName
   value: false

and it is possible to load an instance from YAML

.. code-block:: c++
   
   YAML::Node node(...);
   auto c = YAML::load<Component>(node);

.. _YAML:

YAML 
====

In case the conversion from/to YAML provided by navground is not sufficient, specialize the methods 
:cpp:func:`navground::core::HasRegister::encode` and :cpp:func:`navground::core::HasRegister::decode`.
There is no need to call the base implementation as it is empty.

.. code-block:: c++

   struct MyComponent : public Component {
     // ...
     // ... properties
     // ... type
     // ... introspection

     void encode(YAML::Node &node) const override {
       // put additional information int the YAML node;
     }

     void decode(const YAML::Node &node) override {
       // extract additional information from the YAML node;
     }
   };

Through these methods you can read more complex parameters from the YAML than :cpp:type:`navground::core::Property::Field`. For example, you can configure a value of type ``std::map<std::string, int>`` from a YAML such as

.. code-block:: yaml

   my_complex_param:
      a: 1
      b: 2
      c: 3

if you implement the custom logic in the decoder and the encoder.

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

Macros
======

Following macros reduce boilerplate when *declaring* a component.

When declaring ``type`` and/or ``properties`` without defining them, use:

- :c:macro:`DECLARE_TYPE`
- :c:macro:`DECLARE_PROPERTIES`
- :c:macro:`DECLARE_TYPE_AND_PROPERTIES`

When defining ``type`` and/or ``properties`` inline, use:

- :c:macro:`OVERRIDE_TYPE`
- :c:macro:`OVERRIDE_PROPERTIES`
- :c:macro:`OVERRIDE_TYPE_AND_PROPERTIES` combines both to add

Using the appropriate macro, the class skeleton simplifies, for example, to


.. code-block:: c++

   // declaration

   struct MyComponent : public Component {
     // ...
     DECLARE_TYPE_AND_PROPERTIES
     // void encode(YAML::Node &node) const override;
     // void decode(const YAML::Node &node) override;
   };

   // definition

   static const core::Properties MyComponent::properties = core::Properties{...};
   static const std::string type = register_type<MyComponent>("MyName");

.. _Plugin: 

Install as a plugin
===================

This is a build-time step. Wraps one or more components in a shared library and install it as a plugin to integrate it in the navground CLI and API.

In the project ``CMakeLists.txt``, pass the list of shared libraries you want to install as plugins to  ``register_navground_plugins``.

For example, to build and install component ``MyComponent`` implemented in file ``my_component.cpp``, add

.. code-block:: cmake

   add_library(my_component SHARED my_component.cpp)
   target_link_libraries(my_component navground_core::navground_core ...)
   register_navground_plugins(
      TARGETS my_component
      DESTINATION $<IF:$<BOOL:${WIN32}>,bin,lib>
   )

   install(
     TARGETS my_component
     EXPORT my_componentTargets
     LIBRARY DESTINATION lib
     ARCHIVE DESTINATION lib
     RUNTIME DESTINATION bin
     INCLUDES DESTINATION include
   )

.. note:: 

   We use ``DESTINATION $<IF:$<BOOL:${WIN32}>,bin,lib>`` instead of ``DESTINATION lib`` because on Windows shared libraries are installed  to ``bin``.

Once installed, the components will be automatically discovered when calling :cpp:func:`navground::core::load_plugins`.

.. code-block:: C++

   #include "navground/core/plugins.h"
   #include "navground/core/<component>.h"

   namespace core = navground::core;

   int main() {
      // will load the shared library with MyComponent
      // which in turn will call register_type<MyComponent>("MyName");
      // and make the component discoverable in this process.
      core::load_plugins();
      auto c = Component::make_type("MyName");
      // do something with this component.
   }

.. note::

   This will also make the component discoverable and available in the navground CLI,
   provided that the option "--no-plugins" is not set.

If you share it with them, the library can be installed by other users too, which just need to copy add edit add the library location to the navground plugin index.


Complete example
================

See :ref:`C++ example <component_example>` for an example where we implement and register a new (dummy) navigation behavior in C++.
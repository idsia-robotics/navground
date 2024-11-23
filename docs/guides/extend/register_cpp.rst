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

+------------------+-------------------------------+-------------------------------------------------+
|       add        |               to              |                     example                     |
+==================+===============================+=================================================+
| `Register type`_ | instantiate by name           | ``auto comp = Component::make_type("MyName");`` |
+------------------+-------------------------------+-------------------------------------------------+
| `Properties`_    | access parameters by name     | ``comp.set("value", 1);``                       |
+------------------+-------------------------------+-------------------------------------------------+
| `YAML`_          | customize the YAML conversion | ``auto comp = YAML::load<Component>(node);``    |
+------------------+-------------------------------+-------------------------------------------------+
| `Plugin`_        | share the extension           | ``load_plugins();``                             |
+------------------+-------------------------------+-------------------------------------------------+



.. _Register type: 

Register the type
=================

Register your component to the base class register using :cpp:func:`navground::core::HasRegister::register_type` to be able to instantiate it by name.

.. code-block:: c++

   // declaration

   struct MyComponent : public Component {
     // ... 
     static const std::string type;
   };

   /// definition

   const std::string type = register_type<MyComponent>("MyName");

The name of the static member (``type``, in this case) used to hold ``register_type`` can be anything.

.. note::

   You can also register the class inside the class declaration:

   .. code-block:: c++
      
      struct MyComponent : public Component {
        // ...       
        inline static const std::string type = register_type<MyComponent>("MyName");
      };


Once a class has been registered, it can be instantiated using a generic factory method :cpp:func:`navground::core::HasRegister::make_type` by providing its name

.. code-block:: c++

   std::shared_ptr<Component> c = Component::make_type("MyName");
   const auto type = c->get_type(); 
   // type is equal to "MyName"
   // equivalently using types
   // const auto type = Component::get_type<MyComponent>();

and loaded from a YAML such

.. code-block:: yaml

   type: MyName

using

.. code-block:: c++

   YAML::Node node(...);
   std::shared_ptr<Component> c = YAML::load<Component>(node);
   const auto type = c->get_type(); 
   // type is equal to "MyName"


.. _Properties: 

Define Properties
=================

When registering the class, expose any configuration parameter as a *property*, providing a 
getter, an optional setter, a default value, and an optional description. 

Getters can be

- functions/lambdas of type ``std::function<T(const MyComponent *)>``
- methods of type ``T MyComponent::*()``
- members of type ``T MyComponent::*``

while setters can be

- ``nullptr`` to define readonly properties
- functions/lambdas of type ``std::function<void (const MyComponent *, const & T)>`` or ``std::function<void (const MyComponent *, T)>``
- methods of type ``void MyComponent::*(const & T)`` or ``void MyComponent::*(T)``


As (optional) second argument of :cpp:func:`navground::core::HasRegister::register_type`, pass a map of type :cpp:type:`navground::core::Properties` associating names to properties,  instantiated using a combination of :cpp:func:`navground::core::Property::make`, :cpp:func:`navground::core::Property::make_readwrite`, and :cpp:func:`navground::core::Property::make_readonly`:

.. code-block:: c++

   /// definition

   const std::string type = register_type<MyComponent>(
       "MyName", {{"my_param", core::Property::make(...)}});

In practice, if the class defines accessors like

.. code-block:: c++

   struct MyComponent : public Component {
     // ...
     Component() : _value(true) {}
     bool get_value() const { return _value; }
     void set_value(bool value) { _value = value; }
   private:
     bool _value;
   };

you can define properties like

.. code-block:: c++

   /// definition

   const std::string type = register_type<MyComponent>(
       "MyName", {{"my_param", core::Property::make(&MyComponent::get_value,
                                                    &MyComponent::set_value, true,
                                                    "my description")}});

.. warning::

   Not all compilers support defining the properties inline such as in

   .. code-block:: c++
      
      struct MyComponent : public Component {
        // ...
        bool get_value() const { return _value; }
        void set_value(bool value) { _value = value; }
        inline static const std::string type = register_type<MyComponent>(
            "MyName", {{"my_param", core::Property::make(&MyComponent::get_value,
                                                         &MyComponent::set_value,
                                                         true, "my description")}});
      private:
        bool _value;
      };

   because the initialization order in not guaranteed. To be safe, instantiate your properties outside of the class declaration.

Once the class has been registered, it gains generic accessors :cpp:func:`navground::core::HasProperties::get`, :cpp:func:`navground::core::HasProperties::set`, and :cpp:func:`navground::core::HasProperties::set_value`, that uses names to identify properties.

.. code-block:: c++

   MyComponent c;
   bool value = c.get("value").get<bool>();
   c.set("value", !value);

.. note::

   When we have access to the class declaration, being able to get/set properties by name is not very useful, as we could directly use the class own accessors. 

   Instead, this become very useful when we don't have access to the class definition, like when loading from a shared library, from YAML or when using it in Python. In this case, properties allows a generic way to access to instance attributes (from C++, Python, and YAML).

We can also query for all properties

.. code-block:: c++

   MyComponent c;
   core::Properties & properties = c.get_properties();
   // equivalently using types
   // core::Properties & properties = Component::get_properties<MyComponent>();


Moreover, properties will also appear in the YAML representation

.. code-block:: c++
   
   YAML::dump<Component>(&c);
      
as additional fields (one for each property)
   
.. code-block:: yaml

   type: MyName
   my_param: false

and, similarly, will be loaded from YAML.

.. _YAML:


Property Schema
---------------

Pass an optional argument of type :cpp:expr:`void(YAML::Node &)` to methods like :cpp:func:`navground::core::Property::make` to add validation constrains to the property. For example, to mark an integer property as strictly positive, add

.. code-block:: c++
       
   core::Property::make(&MyComponent::get_value, &MyComponent::set_value,
                        10, "my description", &YAML::schema::strict_positive)}});


YAML 
====

In case the conversion from/to YAML provided by navground is not sufficient, specialize the methods 
:cpp:func:`navground::core::HasRegister::encode` and :cpp:func:`navground::core::HasRegister::decode`.
There is no need to call the base implementation as it is empty.

.. code-block:: c++

   struct MyComponent : public Component {
     // ...

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
      b: false

if you implement the custom logic in the decoder and the encoder, like

.. code-block:: c++

   void encode(YAML::Node &node) const override {
     node["my_complex_param"]["a"] = my_int_a;
     node["my_complex_param"]["b"] = my_bool_b;
   }

   void decode(const YAML::Node &node) override {
     if (node["my_complex_param"]) {
       auto param = node["my_complex_param"];
       if (param["a"]) {
         my_int_a = param["a"]..as<int>();
       }
       if (param["a"]) {
         my_bool_b = param["b"]..as<bool>();
       }
     }
   }

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


Class Schema
------------

If your class defines a custom YAML representation, it should also register the related JSON-schema, by passing a function of type :cpp:expr:`void(YAML::Node &)` as last argument to :cpp:func:`navground::core::HasRegister::register_type`.

In the example above, we add the appropriate schema

.. code-block:: c++

   static void schema(YAML::Node &node) {
     Node my_complex_param;
     my_complex_param["type"] = "object";
     my_complex_param["properties"]["a"]["type"] = "integer";
     my_complex_param["properties"]["b"]["type"] = "boolean";
     my_complex_param["additionalProperties"] = false;
     node["properties"]["my_complex_param"] = my_complex_param;
   }

   const std::string type = register_type<MyComponent>(
       "MyName", {{"my_param", core::Property::make(...)}}, &schema);


Class skelethon
================

Using the appropriate macro, the class skeleton simplifies to


.. code-block:: c++

   // declaration
   
   struct MyComponent : public Component {
     // ...
     static const std::string type;
     // void encode(YAML::Node &node) const override;
     // void decode(const YAML::Node &node) override;
     // static void schema(YAML::Node &node);
   };
   
   // definition
   
   const std::string type = register_type<MyComponent>(
       "MyName",
       {
           {name, core::Property::make(&MyComponent::getter, &MyComponent::setter,
                                       default_value, "description")},
       }
       // , &MyComponent::schema
   );

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
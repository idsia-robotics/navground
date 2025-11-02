========================
Attributes vs Properties
========================

We added two ways to access values by name in the navground C++ library inspired by Python features: (dynamic) attributes and properties. 

For both values are restricted to booleans, integers, floating-point numbers, strings, 2-D vectors and homogeneous sequences of these types  (:cpp:class:`std::vector` in C++, :py:class:`list` in Python) and are saved/loaded from/to YAML.

We expose a similar APIs to set/get them:

.. tabs:: 

   .. tab:: C++

      .. code-block:: c++
   
         SupportsAttributes obj;
         obj.set("key", value);
         obj.get("key");

   .. tab:: Python
   
      .. code-block:: Python
         
         obj = SupportsAttributes()
         obj.set("key", value)
         obj.get("key")


The main differences between them are that:

- the list of properties is pre-defined by the class and cannot be modified by instances.

  .. tabs:: 
  
     .. tab:: C++
  
        .. code-block:: c++
     
           class SupportsProperties : public HasProperties {
              int my_param_getter();
              void my_param_setter();
              inline const static core::Properties properties{
                {"my_param", 
                 core::make_property<int,SupportsProperties>(
                   &my_param_getter, &my_param_setter, 0,
                   "my param description")}};
           };

     .. tab:: Python
     
        .. code-block:: Python
           
           class SupportsProperties(HasProperties):

               @property
               @sim.register
               def my_param(self) -> int: ...

               @my_param.setter
               def my_param(self, value: int) -> None: ...

  On the contrary, attributes can be freely added/deleted by each instance

  .. tabs:: 
  
     .. tab:: C++
  
        .. code-block:: c++
     
           SupportsAttributes obj;
           obj.set("my_param", 1);
           obj.remove("my_param");
 
  
     .. tab:: Python
     
        .. code-block:: Python
           
           obj = SupportsAttributes()
           obj.set("my_param", 1)
           obj.remove("my_param")

- properties are typed, while attributes values can change type.


  .. tabs:: 
  
     .. tab:: C++
  
        .. code-block:: c++
     
           SupportsAttributes obj;
           obj.set("my_param", 1);
           obj.set("my_param", 1.234)
           obj.set("my_param", "xxx");

  
     .. tab:: Python
     
        .. code-block:: Python
           
           obj = SupportsAttributes()
           obj.set("my_param", 1)
           obj.set("my_param", 1.234)
           obj.set("my_param", "xxx")


- properties have a default value while attributes do not.

  .. tabs:: 
  
     .. tab:: C++
  
        .. code-block:: c++
     
           SupportsProperties obj1;
           // returns a value of the property type 
           // (integer in this case)
           auto value1 = obj1.get("my_param"); 
           SupportsAttributes obj2;
           // return std::nullopt as the attributes is not defined
           auto value2 = obj2.get("my_param");
           
     .. tab:: Python
     
        .. code-block:: Python
           
           obj1 = SupportsProperties()
           # returns a value of the property type
           # (integer in this case)
           value1 = obj1.get("my_param")
           obj2 = SupportsAttributes()
           # returns None as the attribute is not defined
           value2 = obj2.get("my_param")

They also serve different goals:

- properties are used to expose (to C++, Python and YAML) parameters of registered sub-classes. In the normal case, properties are solely read by the class itself and are not used to share data between different objects. 
  If the sub-classes are implemented in C++, we can access those parameters from Python and YAML too (and vice-versa), without explicitly binding the accessors.

  For example, when defining a new behavior, we expose specific configurations as properties, which we then used to configure the behavior through YAML.

- attributes are used to append (from C++/Python/YAML) data to an object, in particular to an instance of a (non-registered) base class. When reading an attribute, users should check if the attribute is defined and has the expected type.

  For example, we set the attribute "floor_color" of a world to specify the color of the floor. Then, we can write a renderer that consumes that attribute if it is defined.
 
  .. code-block:: Python
     
     from navground import sim
     from navground.sim.ui.render import pdf_for_world
     from navground.sim.ui import svg_color

     def floor(world: sim.World) -> str:
         x, _, y, _ = world.bounding_box.to_tuple()
         height = world.bounding_box.height
         width = world.bounding_box.width
         rgb = world.get("floor_color")
         try:
            color = svg_color(*rgb)
         except:
            color = "white"
         return f'<rect x="{x}" y="{y}" height="{height}" width="{width}" color="{color}"/>'

     world = sim.World()
     world.bounding_box = sim.BoundingBox(0, 1, 0, 1)
     world.set("floor_color", [0.3, 0.7, 1.0])
     image = pdf_for_world(world, extras=[floor])

.. note::
   
   In the example above, we could have used Python attributes directly (i.e., ``__dict__``), which :py:class:`navground.sim.World` supports.
   By using navground own attributes instead, we can read/write the attributes from C++ and YAML too. For example, we could write a C++ state estimation that consumes such attribute, see :doc:`../tutorials/world_attributes`.


Currently, attributes are supported by:

- ``World`` (:cpp:class:`C++ <navground::sim::World>`, :py:class:`Python <navground.sim.World>`)
- ``Agent`` (:cpp:class:`C++ <navground::sim::Agent>`, :py:class:`Python <navground.sim.Agent>`)  

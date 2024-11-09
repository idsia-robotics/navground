=========
Accessors
=========

We do our best to expose coherent C++ and Python interfaces. Yet, the languages have some peculiarities that may present some pitfalls, one of which are accessors to mutable fields.

C++
===

Given a C++ class ``A`` with two private attributes (one mutable, one immutable) and public accessors to them

.. code-block:: cpp

   #include <string>

   class MutableInt {
    public: 
     MutableInt(int value) : _value(value) {} 
     void set(int value) {_value = value};
     int get() const {return _value};
     std::string repr() const {
       return "MutableInt(" + std::to_string(_value) +  ")";
     }
    private:
     int _value;
   }
  
   class A {
     A() : _i(0), _mi(0) {}
     // returns a copy;
     int get_i() const { return _i; }
     void set_i(int value) { _i = value; }
     // returns a copy;
     MutableInt get_mi() const { return _mi; }
     // returns a reference;
     MutableInt & get_mi_ref() { return _mi; }
     void set_mi(const MutableInt & value) { _mi = value; }

    private:
     // immutable type;
     int _i;
     // mutable type;
     MutableInt _mi;
   };

   A a;

there are two ways to modify the attributes:

1. setting new values

   .. code-block:: cpp

      a.set_i(2);
      a.set_mi(MutableInt{2});

2. call a modifier on the mutable attribute,

   .. code-block:: cpp

      a.get_mi_ref().set(2);

   provided that the class does expose an accessors that returns a reference.

Note that accessors that returns a reference have the further effect to track changes:

.. code-block:: cpp

   auto mi = a.get_mi();
   auto & mi_ref = a.get_mi_ref();
   a.set_mi(MutableInt{3});

now ``mi_ref`` is equal to ``Mutable(3)``, while ``mi`` is still equal to ``Mutable(2)`` as it has been copied.


Pybind11
========

When these accessors are exposed as Python properties (or as Python methods), e.g. through

.. code-block:: cpp

   #include <pybind11/pybind11.h>
   #include <string>
   
   namespace py = pybind11;
   
   PYBIND11_MODULE(my_module, m) {
     py::class_<MutableInt>(m, "MutableInt")
        .def(py::init<int>())
        .def("set", &MutableInt::get)
        .def("get", &MutableInt::set)
        .def("__repr__", &MutableInt::repr);
     py::class_<A>(m, "A")
        .def(py::init<int>())
        .def_property("i", &A::get_i, &A::set_i)
        .def_property("mi", &A::get_mi, &A::set_mi)
        .def_property("mi_ref", &A::get_mi_ref, nullptr);

some methods do not modify ``A`` as a Python user may instead expect (see below)

.. code-block:: python
   
   from my_module import A, MutableInt
   
   a = A(1)

- do modify ``a``:

  >>> a.i = 2
  >>> a.mi = MutableInt(2)
  >>> a.i, a.mi
  2, MutableInt(2)
  >>> a.mi_ref.set(3)
  >>> a.mi
  MutableInt(3)

- does not modify ``a`` as ``a.mi`` returns a copy:

  >>> a.mi.set(4)
  >>> a.mi
  MutableInt(3)
   
Python
======

The equivalent Python class, implemented like

.. code-block:: python

   class PyA:

       def __init__(self):
           self.i = 0
           self.mi = MutableInt(0)

   py_a = PyA()

uses references to holds/pass objects, copying only if asked explicitly, for instances using a property like

.. code-block:: python

   @property
   def mi_copy(self):
       return MutableInt(self.mi.get())


Therefore both methods to modify the (mutable) attribute are available:

1. setting a value

   >>> py_a.i = 2
   >>> py_a.mi = MutableInt(2)
   >>> i, py_a.mi
   2, MutableInt(2)

2. modifying the existing value

   >>> py_a.mi.set(3)
   MutableInt(3)

This difference may confuse users, as they may consider the classes ``A`` and ``PyA`` 
to be equivalent, as they expose the same interface (ignoring ``A.mi_ref``).

.. code-block:: python

   py_a = PyA()
   # this has an effect
   py_a.mi.set(2)
   a = A()
   # while this does not
   a.mi.set(2)

Note that this difference does not apply to immutable attributes.

Navground
=========

Our solution (which follow the same logic as this document) is to

- document if the Python accessors to *mutable* attributes return a copy or a reference.

  Copies are used for POD/small classes, while reference for large classes or classes that are exchanged through (shared) pointers.

- add, when it makes sense, an second accessor to *mutable* attributes, suffixed by ``_ref`` or by ``_copy``, depending on the case.

For examples, the primary property :py:meth:`navground.core.Behavior.target` returns the target by copy, while the secondary
:py:meth:`navground.sim.Behavior.target_ref` returns it by reference. Using them, setting the orientation of a behavior's target can be done by assigning a new target

.. code-block:: python

   # would not changes the agent's pose, as it operates on a copy
   # agent.pose.orientation = 1.0

   target = behavior.target
   target.orientation = 1.0
   behavior.target = target

or just a new orientation

.. code-block:: python

   behavior.target_ref.orientation = 1.0

.. note::

   Currently this property has two accessors:

   - :py:meth:`navground.core.Behavior.target` (read-write) and :py:meth:`navground.core.Behavior.target_ref` (read-only)

   but this could change in future versions.

.. warning::

   Because of the way Pybind11 binds Eigen objects, it is not possible to modify any :py:class:`navground.core.Vector2` in place, like

   .. code-block:: python

      agent.position[0] = 1.0

   Instead, we need to assign a new position, like

   .. code-block:: python

      agent.position = (1.0, agent.position[1]) 


   The same applies to :py:meth:`navground.core.Buffer.data`:

   .. code-block:: python

      buffer = core.Buffer([0, 0, 0])
      # does not change the buffer
      buffer.data[0] = 1
      # does change the buffer
      value = buffer.data
      value[0] = 1
      buffer.data = value






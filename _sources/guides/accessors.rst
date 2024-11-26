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

now mi_ref is equal to ``Mutable(3)``, while mi is still equal to ``Mutable(2)`` as it has been copied.


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

>>> from my_module import A, MutableInt
>>> a = A(1)

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

uses reference to holds/pass objects, copying only if asked explicitly, for instances using a property like

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

2. modify the existing value

   >>> py_a.mi.set(3)
   MutableInt(3)

This difference may confuse users, as they may consider the classes ``A`` and ``PyA`` 
to be equivalent, as they expose the same interface.

.. code-block:: python

   py_a = PyA()
   # this has an effect
   py_a.mi.set(2)
   a = A()
   # while this does not.
   a.mi.set(2)

Note that this difference does not apply to immutable attributes.

Navground
=========

Our solution is to

1. we document if the Python accessors to *mutable* attributes return a copy or a reference

   Copies are normally used for POD and small classes, while reference for large classes or classes that are anyway exchanged through (smart) pointers.

2. when it makes sense, we add an accessor to *mutable* attributes of the other type, suffixed by ``_ref`` or by ``_copy``, depending on the case.

For examples, the primary property :py:meth:`navground.sim.Agent.pose` is returned by copy, while the secondary
:py:meth:`navground.sim.Agent.pose_ref` returns the pose by reference. Using them, changing the ``orientation`` of an ``agent`` can be done in the way the user prefer, i.e., through an assignment/copy

.. code-block:: python

   pose = agent.pose
   pose.orientation = 1.0
   agent.pose = pose

   # or more directly
   agent.pose = core.Pose2(agent.pose.position, 1.0)

or through a reference

.. code-block:: python

   agent.pose_ref.orientation = 1.0

In this particular example, users can also use a property, to indirectly set the orientation through :py:meth:`navground.sim.Agent.orientation`:

.. code-block:: python

   agent.orientation = 1.0

yet navground classes may not exposes accessors for all sub-attributes.
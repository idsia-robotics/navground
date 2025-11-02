=================
Environment State
=================

LineSegment
===========

Schema
------

.. schema:: navground.core.LineSegment.schema()

Example
-------

.. code-block:: yaml

   [[1.2, -3.4], [-4.5, 6.7]]

Disc
====

Schema
------

.. schema:: navground.core.Disc.schema()

Example
-------

.. code-block:: yaml

   position: [1.2, -3.4]
   radius: 0.5


Neighbor
========

Schema
------

.. schema:: navground.core.Neighbor.schema()

Example
-------

.. code-block:: yaml

   id: 0
   position: [1.2, -3.4]
   velocity: [0.1, 0.3]
   radius: 0.5

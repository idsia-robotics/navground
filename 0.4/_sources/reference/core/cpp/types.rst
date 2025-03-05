=====
Types
=====

.. code-block:: cpp
   
   #include "navground/core/types.h"

.. cpp:type:: ng_float_t = float;

   Navground floating points numbers use this type alias to enable selecting globally between ``float`` and ``double``. Define the macro definition ``NAVGROUND_USES_DOUBLE`` to select ``double``.

.. cpp:type:: RandomGenerator = std::mt19937;

   Random generator.

   
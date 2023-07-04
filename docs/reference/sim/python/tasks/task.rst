==========
Base class
==========

.. py:class:: TaskCallback
   :module: navground.sim

   A type alias to ``Callable[List[float], None]``.

   The type of callbacks called when the task publishes data related to an event.
   The task must publish data of the same size, see :py:attr:`navground.sim.Task.log_size`.
   

.. autoclass:: navground.sim.Task
   :members:
   :exclude-members: __new__, __init__
   :inherited-members:


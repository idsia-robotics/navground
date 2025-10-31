==========
Base class
==========

.. py:currentmodule:: navground.sim

.. py:type:: TaskCallback
   :canonical: collections.abc.Callable[list[float], None]

   The type of callbacks called when the task publishes data related to an event.
   The task must publish data of the same size, see :py:attr:`navground.sim.Task.log_size`.
   

.. autoclass:: Task
   :members:
   :exclude-members: __new__, __init__
   :inherited-members:


==========
Controller
==========

.. autoclass:: navground.core.Action.State
   :members: __init__, name, value
   :exclude-members: __new__
   :undoc-members:

   .. autoattribute:: idle

      the action has not started

   .. autoattribute:: running

      the action is running

   .. autoattribute:: failure

      the action failed

   .. autoattribute:: success

      the action succeeded


.. autoclass:: navground.core.Action
    :members:
    :exclude-members: State

.. autoclass:: navground.core.Controller
    :members:


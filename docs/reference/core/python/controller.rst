==========
Controller
==========

.. py:currentmodule:: navground.core

.. autoclass:: navground.core.Action.State

   .. autoattribute:: idle

      the action has not started

   .. autoattribute:: running

      the action is running

   .. autoattribute:: failure

      the action failed

   .. autoattribute:: success

      the action succeeded


.. autoclass:: Action
    :members:
    :exclude-members: State, __init__

.. autoclass:: Controller
    :members:


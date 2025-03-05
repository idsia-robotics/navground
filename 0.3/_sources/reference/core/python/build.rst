==========
Build info
==========

.. py:currentmodule:: navground.core

.. autoclass:: BuildInfo
   :members:
   :exclude-members: __init__

.. autoclass:: DependencyInfo
   :members:
   :exclude-members: __init__

.. py:type:: BuildDependencies
   :module: navground.core
   :canonical: dict[str, DependencyInfo]

.. py:type:: PkgDependencies
   :module: navground.core
   :canonical: dict[str, dict[pathlib.Path, BuildDependencies]]

.. autofunction:: get_build_info

.. autofunction:: get_build_dependencies

.. autofunction:: get_plugins_dependencies
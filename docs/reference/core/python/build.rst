==========
Build info
==========

.. py:currentmodule:: navground.core

.. autoclass:: BuildInfo
   :members:

.. py:type:: BuildDependencies
   :module: navground.core
   :canonical: dict[str, list[navground.core.BuildInfo]]

.. py:type:: PkgDependencies
   :module: navground.core
   :canonical: dict[str, dict[pathlib.Path, navground.core.BuildDependencies]]

.. autofunction:: get_build_info

.. autofunction:: get_build_dependencies

.. autofunction:: get_plugins_dependencies
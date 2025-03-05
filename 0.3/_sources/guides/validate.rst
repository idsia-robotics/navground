====================
How to validate code
====================

You can interact with navground through C++, Python, and YAML.
While C++ is statically typed and therefore the compiler helps to (partially) validate the code,
we suggest to use the following tools to validate Python and YAML too. 

Python
======

Packages :py:mod:`navground.core` and  :py:mod:`navground.sim` have type hints. For the pybind11 parts, we provide stub files, while Python code is fully annotated with types. Therefore you can use tools such as `mypy <https://mypy.readthedocs.io>`_, to check you code.

To check the packages themselves, you can run

.. command-output:: mypy -p navground.core --strict

and

.. command-output:: mypy -p navground.core --strict


To check your package, you can run

.. code-block:: console 

   $ mypy <path>
   

YAML
====

YAML representations in navground are compatible with JSON and we provide `JSON-schemas <https://json-schema.org>`_ to validate YAML.  You can get the full schema of ``navground_{core|sim}`` as a YAML string using

.. code-block:: console 

   $ navground_py schema {core|sim}
   ...
   

To validate your YAML code, you can use any `tool <https://json-schema.org/tools>`_ supporting JSON-schema, like command :ref:`validate_sim_py` that uses `python-jsonschema <https://python-jsonschema.readthedocs.io/en/stable>`_:

.. code-block:: console

   $ navground_py validate experiment install/share/navground_examples_yaml/experiment/cross.yaml
   
which will complain if the file does not corresponds to the :ref:`schema of an experiment<experiment yaml>`.

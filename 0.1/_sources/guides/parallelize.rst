.. _parallelize_guide:

=================================================
How to parallelize the execution of an experiment
=================================================

Navground uses a single thread to simulates a world. Nonetheless, we can distribute the execution of *multiple* simulations, like when running multiple runs in an experiment, over several threads or processes, which will in general decrease computational times linearly, as long as there is enough computational power available.

In this guide will illustrate the differences between parallelizing over threads vs processes, and between using Python vs C++.


Python API
==========

Multi-threading
---------------

The Python interpreter has a global lock (the "GIL") that let only one thread executes the Python instructions at the same time. Therefore, it is not possible to truly parallelize experiments that requires Python. More precisely, if any simulated agents has at least one component (behavior, kinematics, state estimation or task) instantiated from a *Python* class, we cannot use multiple threads. This limitation does not applies to scenarios, which *can* be instances of Python classes, as the creation of worlds is not parallelized, even in parallelized experiments.

.. note::

   To check if an experiment, scenario, world, or agent does use Python, run :py:func:`navground.sim.uses_python`. 

If the experiment uses only C++ components (except the scenario), we can distribute :py:meth:`navground.sim.Experiment.run` over ``N`` threads by passing ``number_of_threads=N``:

.. code-block:: python

   experiment.run(number_of_threads=N)

Multi-processing
----------------

Any experiment can be distributed over multiple processes, that is also experiments that do use Python instances of behaviors, kinematics, state estimations or tasks.

We can distribute an experiment over ``N`` processes using :py:meth:`navground.sim.Experiment.run_mp` instead of :py:meth:`navground.sim.Experiment.run`:

.. code-block:: python

   experiment.run_mp(number_of_processes=N)

When launched like this, the experiment does not store runs in memory but only to HDF5 files.
In fact, :py:meth:`navground.sim.Experiment.run_mp` will split an experiment in ``N`` sub-experiments, each running a separate set of runs and saving the data to a separate ``data_<i>.h5``, with ``i=0, 1, ..., N-1``, HDF5 files which the main HDF5 file ``data.h5`` links together. 

.. warning::

   Do not rename/remove the individual ``data_<i>.h5``  files or else ``data.h5`` will not find its data anymore.


C++
====

Multi-threading
---------------

From C++, we can distribute experiments over ``N`` threads by specifying the last argument of :cpp:func:`navground::sim::Experiment::run`, like

.. code-block:: cpp

   experiment.run(..., N);

Multi-processing
----------------

Executing an experiment over multiple processes from C++ is not supported.
Use the Python interface or command line.

Executables
===========

Automatic parallelization
-------------------------

The executables :ref:`run` exposes the arguments ``--threads`` and ``--processes`` to specify either the number of threads or of processes, with default values of 1 (i.e., running single threaded). 

.. note::

   Do not specify non-default values for both arguments as we do not support running multi-threaded in multiple processes.

As mentioned above, multi-processing is currently only available in Python, and therefore only in ``run_py``.

Manual parallelization
----------------------

We can also parallelizing experiments by dividing them manually and launching several instances of ``run`` or ``run_py`` at the same time.

For example, if the experiment described in ``experiment.yaml`` needs 1000 runs, you can split them over 10 experiments with  ``run_index=0, 100, 200, ...`` and ``runs=100``, each running in a separate process:

.. code-block:: console

    $ for i in {0..9}; do navground run experiment.yaml --runs 100 --run_index $((i * 100)) &; done

This will results in 10 directories, one for each sub-experiment: you can load their data and aggregate it.  


Performance
===========

For multi-threading, the creation and saving of an experimental run is not parallelized as it is protected using locks. Only :cpp:func:`navground::sim::ExperimentalRun::run` are run in parallel and which generally have the dominant computational costs. Still, if your experimental runs are very short, it may not be worth parallelizing them.

The overhead of multi-processing is higher than for multi-threading but multi-processing also parallelizes the creation and saving of runs, therefore in the not-so-plausible case when creation/saving has significant computational cost, it may be worth using multi-processing over multi-threading, even when running C++-only experiments.

From a memory point-of-view, there should be no significant differences when using the same number ``N``  of threads or cores, and which in general will requires ``N`` times more memory than running the experiment without parallelization.

In general, using more threads or processes than the number of available cores is not advised.


Summary
=======

The following table summarizes the options for parallelizing in C++ and Python.

.. list-table:: Parallelizing in C++ and Python
   :widths: 25 15 15 15
   :header-rows: 1

   * - 
     - Sequential  
     - Multi-threading
     - Multi-processing
   * - **C++**
     - ✅
     - ✅
     - 🚫
   * - **Python**, without Python agents' components
     - ✅
     - ✅
     - ✅
   * - **Python**, with Python agents' components
     - ✅
     - 🚫
     - ✅

How data is recorded also depends on how we choose to parallelize.

.. list-table:: Recording data and parallelization
   :widths: 25 15 15 15
   :header-rows: 1

   * - 
     - Sequential  
     - Multi-threading
     - Multi-processing
   * - **In-memory**
     - ✅
     - ✅
     - 🚫
   * - **H5DF**
     - ✅ 1 file
     - ✅ 1 file
     - ✅ N+1 files

.. note::

   Of course, after performing a multi-process experiment, we can *load* the saved HDF5 back in memory, like for any other experiment. It's the Python method :py:meth:`navground.sim.Experiment.run_mp` that is not directly collecting data from runs executed in differences processes. 
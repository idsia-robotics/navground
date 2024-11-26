========================================
How to record custom data in experiments
========================================

You can configure experiments and experimental runs to record
different data such as time, collisions, deadlocks, poses, velocities, targets and commands of all agents, safety margin violations, efficacy, and data logged from tasks.

You can record additional data through custom probes.
Probes are callbacks called during experimental runs. Depending on the use case,
you can subclass one of ``RecordProbe`` or ``GroupRecordProbe`` to record data in a single dataset or in a map of datasets. In case you need more flexibility, subclass instead the base class ``Probe`` and create/write any number of datasets.


A Single Dataset
================

When the data you want to record fits a single dataset, you can use ``RecordProbe``.

This is the case for instance, when you want to record the same number of items per step and per agent, and all have the same type.


#. Add a subclass of ``RecordProbe``

   .. tabs::
   
      .. tab:: C++

         Specialize :cpp:class:`navground::sim::RecordProbe`.
   
         .. code-block:: c++
      
            #include "navground/sim/experimental_run.h"
            #include "navground/sim/probe.h"

            namespace sim = navground::sim;
         
            class MyProbe : public sim::RecordProbe {
   
      .. tab:: Python

         Specialize :py:class:`navground.sim.RecordProbe`.
   
         .. code-block:: python
      
            from navground import sim
            import numpy as np
         
            class MyProbe(sim.RecordProbe):

#. Specify the type
   
   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            using Type = uint8_t;
   
      .. tab:: Python

         .. code-block:: c++
      
            dtype = np.uint8
    
#. Overwrite the method that define the shape of the dataset

   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            sim::Dataset::Shape get_shape(const sim::World &world) const override {
                return ...;
            }
      .. tab:: Python
      
         .. code-block:: Python
            
            def get_shape(self, world: sim.World) -> Tuple[int, ...]:
                return ...

#. Overwrite the method that fill the dataset

   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            void update(sim::ExperimentalRun *run) override {
              // call get_data()->push(...) or get_data()->append(...)
            }
   
      .. tab:: Python
   
         .. code-block:: Python
   
            def update(self, run: sim.ExperimentalRun) -> None:
                # call self.data.push(...) or self.data.append(...)

#. Add the probe to the experiment or the experimental run

   .. tabs::

      .. tab:: C++
   
         .. code-block:: c++
      
            experiment.add_record_probe<MyProbe>("my_key");
   
      .. tab:: Python
   
         .. code-block:: Python
   
            experiment.add_record_probe("my_key", MyProbe)
         

#. Run the experiment (or the experimental run)


A Group of Datasets
===================

When the data you want to save does not fit a single dataset, like, for example, when data associated with different agents may have different shapes, you can split it up in multiple datasets indexed by string-valued keys, which may be the UID or the index of the agents.

In this case, you can use ``GroupRecordProbe`` to setup and manage a dictionary of datasets.

#. Add a subclass of ``GroupRecordProbe``

   .. tabs::
   
      .. tab:: C++

         Specialize :cpp:class:`navground::sim::GroupRecordProbe`.
   
         .. code-block:: c++
      
            #include "navground/sim/experimental_run.h"
            #include "navground/sim/probe.h"

            namespace sim = navground::sim;
         
            class MyProbe : public sim::GroupRecordProbe {
   
      .. tab:: Python

         Specialize :py:class:`navground.sim.GroupRecordProbe`.
   
         .. code-block:: python
      
            from navground import sim
            import numpy as np
         
            class MyProbe(sim.GroupRecordProbe):

#. Specify the type
   
   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            using Type = uint8_t;
   
      .. tab:: Python

         .. code-block:: c++
      
            dtype = np.uint8
    
#. Overwrite the method that define the shape of the datasets

   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            sim::Dataset::Shape get_shapes(const World &world, bool use_uid) const override {
                // return ...;
            }
      .. tab:: Python
      
         .. code-block:: Python
            
            def get_shapes(self, world: sim.World, use_uid: bool) -> Dict[str, Tuple[int, ...]]:
                # return ...

#. Overwrite the method that fill the datasets

   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            void update(sim::ExperimentalRun *run) override {
                // get the datasets using 
                // data = get_data(key)
                // call get_data()->push(...) or get_data()->append(...)
            }
   
      .. tab:: Python
   
         .. code-block:: Python
   
            def update(self, run: sim.ExperimentalRun) -> None:
                # get the datasets using 
                # data = self.get_data(key)
                # call data.push(...) or data.append(...)

#. Add the probe to the experiment or the experimental run

   .. tabs::

      .. tab:: C++
   
         .. code-block:: c++
      
            experiment.add_group_record_probe<MyProbe>("my_key");
   
      .. tab:: Python
   
         .. code-block:: Python
   
            experiment.add_group_record_probe("my_key", MyProbe)
         

#. Run the experiment (or the experimental run)

A Custom Probe
==============

When a group of datasets does not fit your needs, you can subclass the base class ``Probe`` to implement custom callbacks.

This is the case for instance when the datasets should belong to a hierarchy of groups. In this case, you need to explicitly add the datasets to the ``ExperimentalRun``.


#. Add a subclass of ``Probe``

   .. tabs::
   
      .. tab:: C++

         Specialize :cpp:class:`navground::sim::Probe`.
   
         .. code-block:: c++
      
            #include "navground/sim/experimental_run.h"
            #include "navground/sim/probe.h"
         
            class MyProbe : public navground::sim::Probe {
   
      .. tab:: Python

         Specialize :py:class:`navground.sim.Probe`.
   
         .. code-block:: python
      
            from navground import sim
            import numpy as np
         
            class MyProbe(sim.Probe):

#. Overwrite the callbacks to create and update the datasets

   .. tabs:: 

      .. tab:: C++
   
         .. code-block:: c++
      
            void prepare(sim::ExperimentalRun *run) override {
                // create one or more datasets
                auto data = run->add_record("group/subgroup/key");
                // set their type and shape
                get_data()->set_type<float>();
                get_data()->set_item_shape({3, 4});
            }

            void update(sim::ExperimentalRun *run) override {
               auto data = run->get_record("group/subgroup/key");
               // call get_data()->push(...) or get_data()->append(...);
            }

            void finalize(sim::ExperimentalRun *run) override {
               // last chance to create/modify datasets
            }

      .. tab:: Python
      
         .. code-block:: Python
            
            def prepare(self, run: sim.ExperimentalRun) -> None:
                # create one or more datasets
                run.add_record("group/subgroup/key", 
                               np.zeros((0, 3, 4), dtype=np.float32))

            def update(self, run: sim.ExperimentalRun) -> None:
                data = run.get_record("group/subgroup/key")
                # call data.push(...) or data.append(...)


            def finalize(self, run: sim.ExperimentalRun) -> None:
                # last chance to create/modify datasets
                ...

#. Add the probe to the experiment or the experimental run

   .. tabs::

      .. tab:: C++
   
         .. code-block:: c++
      
            experiment.add_probe([](){return std::make_shared<MyProbe>();});
   
      .. tab:: Python
   
         .. code-block:: Python
   
            # Python classes are callable, 
            # so we can pass the class directly
            experiment.add_probe(MyProbe)
         

#. Run the experiment (or the experimental run)

Complete example
================

Have at look at the :ref:`C++ <custom_recordings>` and :ref:`Python <custom_recordings_py>` examples to see complete examples of performing experiments using custom probes.

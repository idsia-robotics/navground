=============
Local GridMap
=============

A state estimation that integrates lidar scans and optional odometry readings in an local occupancy map. Uses one of the simplest mapping algorithms, following the implementation of the `obstacle layer in nav2 local costmap <https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/obstacle_layer.hpp>`_.

Updates fields "local_gridmap" with a local 8-bit gridmap (0 = obstacle, 128 = unknown, 255 = free). Optionally also update the transformation between map frame and world (which is different than identity when using an odometry source).


Example
=======

.. video:: local_gridmap.mp4
   :loop:
   :width: 780

The video has been recorded in the :doc:`../../tutorials/sensors` notebook with the following configuration

.. literalinclude :: local_gridmap.yaml
   :language: YAML









"""Standalone ROS 2 map-orientation estimation and map-saving support.

The package provides reusable quaternion/circular-statistics math, a continuous
IMU/TF estimator, and a Nav2 saver wrapper that attaches orientation metadata
to raster maps. It has no dependency on a particular robot bringup project.
Importing this package does not initialize ROS or launch any processes.

Installed executables are map_orientation_node and map_saver_with_orientation;
launch/map_orientation.launch.py configures the estimator. See README.md for
heading conventions, simulation configuration, and sidecar usage.
"""

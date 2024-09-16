# Demonstrate the use of the tools with different ways to perform the same demonstration

## Demonstration

- Two Thymios starts at ((-0.5, 0.1), 0.0) and ((0.5, -0.1), 0.0) respectively.
- There is an obstacles of size 0.1 at (0, 0) 
- There is a wall around the area a square of size 3, centered at (0, 0)
- The Thymios have to travel between (1, 0) and (-1, 0) with a tolerance of 0.1 at 0.12 m/s


### Different implementations

### Naked C++ exec

#### Limitations
- no collisions
- perfect sensing
- not real-time
	
#### Depends on
- navground


### Naked Python exec

#### Limitations
- no collisions
- perfect sensing
- not real-time
	
#### Depends on
- navground
- navground_core_py

### C++ simulation

#### Limitations
- not real-time
	
#### Depends on
- navground
- navground_sim

### Python simulation

#### Limitations
- not real-time
	
#### Depends on
- navground
- navground_sim
- navground_sim_py (TBD)

### PyEnki simulation

#### Limitations
- perfect sensing
	
#### Depends on
- navground
- navground_core_py

### CoppeliaSim internal simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- navground
- navground_coppeliasim


### CoppeliaSim ROS2 simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- navground
- navground_ros


### Real robot ROS2 simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- navground
- navground_ros

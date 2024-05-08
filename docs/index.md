# CNearest


## Deliverable

This delivers source code suitable to compile a static library, a set of unit tests, and a number of executable examples ....

## Literature

This implements the methodology described in the paper the paper TODO: reference paper

## Precursors

This follows on from an earlier matlab code TODO: reference matlab code base

## Physics assumptions

TODO: list physics assumptions, eg constant temperature? fixed fluid viscosity? low reynolds number?
Maybe just tell people to refer to the paper above

## Numerical assumptions

### Units of measurement

We don't specify the units used in the code, assuming coherant input values and results are being used at runtime.
While using it be sure to use a consistent set of base units and derived units.
For example, if you specify the distances to be in mm, do not attempt to measure the forces in Newtons (kg m/s^2), as this involves differeing units of length.
If unsure, enter and real all values in SI units.

### Frames of reference

All locations and directions are assumed to be in a single globally applicable reference frame.
The frame of reference is globally carteasian.
The axis of the reference frame obay a right-hand-rule.

### Angles

All angles are in radians, all angular velocites are in rad/s.

## Coding overview

The code is written entirely in C++.
Our linear algebra is delegated to the Eigen library, which in turn makes use of blas and lapack.

### Naming Conventions

The title "vector" risks being used in 2 different ways,

+ The std c++ libraries use "std::vector" as an ordered collection of any object
+ The eigen library the term Eigen::VectorXd for a ordered list of doubles, these can then be used in numerical calculations such as linear algebra

To avoid confusion in code, we will give an alias to one of these 2 types of "vector". 
Since the primary users will be mathematicians and physicists, we'll keep the name vecor to denote the numerical structure with the aliases:

	
	using cnearest::Vector = Eigen::VectorXd;
	
We will give an alias to the std::vector, and will call it our Collection. 
While computer scientists may know that collections can be depicted in may alternative ways, we intend to use this versitile class for the majority of collections throughout this codebase, justifing the alias.

	using cnearest::Collection = std::vector;
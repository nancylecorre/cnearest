# CNearest

## Summary


This package implements a method for solving problems in very viscous fluid dynamics. The package provides a method for calculating the drag force and moment on rigid spheres and spheroids  immersed in a 'simple' (Newtonian) viscous fluid, in the limit where inertia is negligible relative to viscosity (zero Reynolds number / Stokes flow). 

This system is particularly relevant in microscale biological fluid dynamics systems and colloids; one specific application is the calculation of the diffusion tensor of a specified structure. 

The underlying numerical algorithm is the nearest-neighbour implementation of regularised stokeslets; this approach enables efficient and accurate calculations requiring only surface point discretisations rather than surface or volumetric meshes. 

The calculated flow field is also ensured to be regular throughout the interior, exterior and surface of the body. The package also serves as a library that can be utilised with the user's own surface discretisation to calculate the force and moment on any given three dimensional body undergoing rigid body motion. 

The library also forms an extensible toolkit for problems involving calculations of flow fields due to flexible motions, such as cilia-driven transport, motility / swimming due to flagellar movement or surface deformations, and integration with models of actively bending elastic bodies. 

## Documentation

For documentation see our gitlab pages <https://djsmithbham.gitlab.io/cnearest/>. 

In particular, to set up build environment including pre-requisite packages (BLAS, LAPACK, Eigen, gtest), visit <https://djsmithbham.gitlab.io/cnearest/d9/d44/md_docs_2dev-environment.html>.

## Deliverable

This delivers source code suitable to compile a static library, a set of unit tests, and a number of executable examples.

## Precursors

This library re-implements the earlier Matlab/GNU Octave code <https://github.com/djsmithbham/NearestStokeslets> and is planned in due course to include features of later extensions:

+ <https://gitlab.com/meuriggallagher/NEAREST>
+ <https://gitlab.com/meuriggallagher/passively-parallel-regularized-stokeslets>
+ <https://gitlab.com/djsmithbham/nearest-d>
+ <https://gitlab.com/meuriggallagher/the-art-of-coarse-stokes>

## Physics assumptions

CNearest solves viscous-dominated, typically microscale flows [1-3]. The underlying physical assumptions are that the system has Reynolds number much less than 1, and the fluid is Newtonian and has constant  dynamic viscosity. The present library focuses on systems involving volume-conserving particles, with no-slip, no-penetration boundary conditions. The method requires only a point cloud rather than a true boundary element mesh.

## Numerical assumptions

### Units of measurement

Physical variables are position, velocity and force, assuming consistent units (e.g. all SI, or all dimensionless).

### Frames of reference

All locations and directions are assumed to be in a single globally applicable reference frame.
The frame of reference is globally cartesian.
The axis of the reference frame obey a right-hand-rule.

### Angles

All angles are in radians, all angular velocities are in rad/s.

## Coding overview

The code is written entirely in C++.
Our linear algebra is delegated to the Eigen library, which in turn makes use of BLAS and LAPACK.

### Naming Conventions

The title "vector" risks being used in 2 different ways,

+ The std c++ libraries use "std::vector" as an ordered collection of any object
+ The Eigen library the term Eigen::VectorXd for a ordered list of doubles, these can then be used in numerical calculations such as linear algebra

To avoid confusion in code, we will give an alias to one of these 2 types of "vector".
Since the primary users will be mathematicians and physicists, we'll keep the name vector to denote the numerical structure with the aliases:

    using cnearest::Vector = Eigen::VectorXd;

We will give an alias to the std::vector, and will call it our Collection.
While computer scientists may know that collections can be depicted in may alternative ways, we intend to use this versatile class for the majority of collections throughout this codebase, justifying the alias.

    using cnearest::Collection = std::vector;

## References

1. Smith, D. J. (2018). A nearest-neighbour discretisation of the regularized stokeslet boundary integral equation. Journal of Computational Physics, 358, 88-102.
2. Gallagher, M. T., & Smith, D. J. (2018). Meshfree and efficient modeling of swimming cells. Physical Review Fluids, 3(5), 053101.
3. Gallagher, M. T., Choudhuri, D., & Smith, D. J. (2019). Sharp quadrature error bounds for the nearest-neighbor discretization of the regularized stokeslet boundary integral equation. SIAM Journal on Scientific Computing, 41(1), B139-B152.


---
title: 'CNearest: A C++ toolkit to use the nearest-neighbour method of regularised stokeslets algorithm to solve viscous flow problems'
tags:
  - C++
  - fluid dynamics
  - regularised stokeslets
  - stokes flow
  - discretisation
authors:
  - name: Neil A. Butcher
    equal-contrib: true
    affiliation: 1
  - name: James Tyrrell
    equal-contrib: true 
    affiliation: 1
  - name: David Smith
    equal-contrib: true 
    affiliation: 2
affiliations:
 - name: ARC, University of Birmingham, Edgbaston, Birmingham, UK
   index: 1
 - name: School of Mathematics and Centre for Systems Modelling and Quantitative Biomedicine, University of Birmingham, Edgbaston, Birmingham, UK
   index: 2
date: 18 September 2024
bibliography: paper.bib

---

# Summary

This package implements a method for solving problems in very viscous fluid dynamics, in the limit where inertia is negligible relative to viscosity. This system is particularly relevant in microscale biological systems and colloids; one specific application is the calculation of the diffusion tensor of a specified structure. The underlying algorithm is the nearest-neighbour implementation [@smith2018] of the method of regularised stokeslets [@cortez2001;@cortez2005]; this approach enables efficient and accurate calculations requiring only surface point discretisations rather than surface or volumetric meshes and avoids singular integrals. It is a feature-complete reimplementation and further development of the key test cases in @smith2018. The package also serves as an extensible library that can be utilised with the user's own surface discretisation to calculate the force and moment on any given three dimensional body undergoing rigid body motion, and provides a building block to model flexible swimming bodies. 

# Statement of need

Microscale biological flow problems typically involve complex-shaped bodies undergoing large deformations, which renders analytical calculations or methods based on volumetric meshing (finite element, finite difference) challenging to implement. However, in the case where the fluid has Newtonian or approximately Newtonian properties, following non-dimensionalisation the flow can be modelled through the linear Stokes flow equations,
$$
{-} \pmb{\nabla} p + \nabla^2{\pmb{u}} = 0, \quad \pmb{\nabla} \cdot \pmb{u} = 0, \quad (1)
$$
where $p(\pmb{x})$ and $\pmb{u}(\pmb{x})$ are respectively pressure and velocity at the point $\pmb{x}$.

One of the basic problems of Stokes flow is the calculation of the force and moment on a body immersed in a fluid and subject to no-slip, no-penetration boundary conditions, due to specified rigid body motion. Mathematically, for a body with surface $B$ subject to translational velocity $\pmb{U}$ and angular velocity $\pmb{\Omega}$ about the origin, the problem is to solve equation (1) subject to the boundary condition,
$$
\pmb{u}(\pmb{x}) = \pmb{U} + \pmb{\Omega}\times\pmb{x}, \quad \mathrm{for}\, \mathrm{all} \quad \pmb{x}\in B, \quad (2)
$$
and thereby calculate the force and moment
$$
\pmb{F}=\iint_B \pmb{\sigma} \cdot \pmb{n} \, dS(\pmb{x}), \quad \pmb{M}=\iint_B\pmb{x}\times\pmb{\sigma}\cdot\pmb{n} \, dS(\pmb{x}), \quad (3)
$$
where the stress tensor is given in terms of the pressure and velocity by,
$$
\pmb{\sigma} = -p\pmb{I}+\pmb{\nabla}\pmb{u}+(\pmb{\nabla}\pmb{u})^T. (4)
$$
The system (1--4) forms the basis for more complex problems in which the body motion is not prescribed, or the surface motion is non-rigid. Moreover due to the linearity and instantaneity of the equations, the solution for translational velocity $\pmb{U}$ and angular velocity $\pmb{\Omega}$ can be determined from a linear combination of the 'basic' modes corresponding to unit velocity along each Cartesian axis and unit angular velocity about each Cartesian axis; the basic functionality of CNearest is to perform these calculations. The $6\times 6$ matrix of force and moment coefficients corresponding to each basic rigid body mode is referred to as the *grand resistance matrix* [@pozrikidis2002] or *frictional tensor* [@wegener1981]. The matrix inverse can be used to calculate the diffusion tensor quantifying the translational and rotational Brownian motion of the body [@wegener1981].

Linearity means that (1-4) can be solved through techniques such as the boundary element method, which results in a linear algebra problem of the form,
$$
\left[{\sf A}\right] {\sf f} = {\sf u},
$$
where $\left[{\sf A}\right]$ is a $3N\times 3N$ matrix formed by evaluating or integrating fundamental solutions of (1), ${\sf f}$ is a $3N$-vector of unknown traction or force values at each degree of freedom, and ${\sf u}$ is a $3N$-vector of velocities at each point of the discretisation. For more details, see for example Pozrikidis [-@pozrikidis1992;-@pozrikidis2002]. However these techniques involve singular solutions which present challenges for implementation. Moreover, despite its high numerical efficiency, the boundary element method requires the user to supply a true surface mesh with connectivity table and local basis system. The method of regularised stokeslets [@cortez2001;@cortez2005] has achieved significant success in biological fluid dynamics (see for example the Special Issue edited by Cortez and Olson [@cortez2021]) by removing both the mathematically singular behaviour, and avoiding the need to construct a true mesh (see also @smith2009). This package implements an extension of the method of regularised stokeslets which utilises separate discretisations for the surface traction and quadrature. The method, previously implemented in Matlab [@neareststokeslets2018] is based on nearest-neighbour interpolation, and it removes the dependence of the linear system size on the quadrature error, greatly improving accuracy and efficiency. The method distinguishes the size of the force discretisation ($N$) from the larger sized quadrature discretisation ($Q$). For more details, including application to swimming problems and error analysis, see references [@gallagher2018;@gallagher2019].

Despite its proven effectiveness, a barrier to wider adoption of the nearest neighbour discretisation is that the existing implementation underpinning most of the references above is an undocumented research code implemented in Matlab. That implementation primarily uses built-in linear algebra operations throughout the workflow, from stokeslet evaluation through matrix assembly, and avoids the use of for loops. While this approach has some advantages - notably brief code and very simple exploitation of parallelism [@gallagher2020], it tends to result in the storage of large stokeslet matrices, which can be highly inefficient as the quadrature discretisation is refined. CNearest addresses these limitations. 

In the wider setting of software packages and code for Stokes flow, we note several existing packages with related but distinct aims, which the present package is intended to complement. pyStokes [@Singh2020] implements an efficient method for solving coupled Stokes and Laplace problems associated with phoretic swimmers, utilising a spectral approximation to the flow field around potentially large numbers of spherical particles. It is therefore particularly suited to problems involving many bodies, but is less focused on detailed resolution of swimmer morphology. Along similar lines, "Stokesian dynamics in Python" [@townsend2024] addresses the need for efficient approximation of long range interactions of many particles; an older package implementing a similar method in Python is "StokeD" [@parker2016], also coded in Python. Implementations of the method of regularised stokeslets include an undocumented parallelized code repository of the classical method [@copos2020], and a more recent package which enables the use of analytic surface integrals [@ferranti2024;@ferranti2024package]. The latter method greatly improves the accuracy and efficiency of the method of regularised stokeslets, however it does require the generation of a true surface mesh rather than a point cloud discretisation. Taken together, we believe that the present package addresses a gap for an extensible, accessible and efficient methodology for Stokes flow problems, particularly those involving complex geometries.

To enable efficient and scalable use of computing resources, the package is implemented in C++, with the widely-used _eigen_ package handling linear algebra operations [@eigenweb]. The package also provides three specific algorithmic developments, which arose through the process of re-implementing the method in C++:

1. Tiebreaking: ensuring reproducibility of results when the nearest-neighbour mapping is non-unique.
2. Stokeslet accumulation: building the left hand side $3N\times 3N$ matrix by adding stokeslets one at a time, rather than forming the larger $3Q\times 3N$ stokeslet matrix and multiplying by a sparse projection matrix.
3. Rescaling by local stokeslet count: the degrees of freedom of the system are thereby the physical force associated with each discretisation point, thereby ensuring that the matrix and solution vector are convergent as the quadrature discretisation is refined.

# Usage

The workflow is that the user will first define the problem in terms of physical bodies such as spheres or spheroids (implementation of other shapes and flexible motions is an area for future development) from which the force and quadrature discretisations are created, then finally defining the boundary conditions. For the rigid body resistance problem (1-4) with no-slip, no-penetration boundary conditions, the boundary velocity can be calculated from supplied values of the translational and rotational velocity of the body, from which the surface velocity components at each point are calculated.

The library then constructs a linear algebra problem by detecting nearest neighbours, managing degrees of freedom, assembling matrices from the regularised stokeslets, and assembling vectors from the boundary conditions.
A linear solver (either the default solver based on LU-decomposition with pivoting or an alternative chosen by the user) is invoked to find the unknown force distribution over the nodes.
Finally the same tools which constructed the full size vectors are used to decompose them back to the total physical force and moment on the body. 

# Validation

The results of the code have been compared with the results of the (established) Matlab codebase. This comparison goes to make one of our suites of unit tests.
This code is also capable of reproducing the results tables published in @smith2018 for the force and moment on a translating and rotating sphere and prolate spheroid. This comparison is presented
as examples 1 and 2 respectively (see *examples/example-1-sphere* and *examples/example-2-prolate-spheroid*).



# Summary and areas for future development

Potential areas for future development include solution of model cells with flexible flagella, swimming problems (mobility due to a prescribed swimming stroke; [@gallagher2018]), multiple bodies and parallelisation [@gallagher2020], boundary image systems [@ainley2008;@cortez2015], stokeslet lines [@smith2009;@cortez2018], stokeslets with improved far field behaviour [@zhao2019], integration with elastohydrodynamics [@hall2019], stokeslet rings [@tyrrell2019], Richardson extrapolation [@gallagher2021], implementation of the double layer potential, which is highly important for problems involving surface slip [@smith2021], modelling  self-motile phoretic colloids [@montenegro2015] and fast multipole acceleration for very large problems [@rostami2016].

# Acknowledgements

This work was partially supported by the Engineering and Physical Sciences Research Council (EP/N021096/1).

We acknowledge Meurig Gallagher (Centre for Systems Modelling and Quantitative Biomedicine, University of Birmingham) for his continuing work on developing the nearest-neighbour method and associated Matlab implementations since 2018. We also acknowledge Research Software Engineer Simon Hartley (Advanced Research Computing, University of Birmingham), whose earlier engagement with this problem uncovered the risk of force points which were tied in for nearest neighbour.

# References
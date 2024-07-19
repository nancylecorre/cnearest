---
title: 'CNearest: A c++ toolkit to use the nearest neighbour algorithm to solve fluid problems'
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
date: 01 August 2024
bibliography: paper.bib

---

# Summary

This package implements a method for solving two fluid dynamics problems: calculating the drag force and moment on a given shaped rigid body immersed in a 'simple' (Newtonian) viscous fluid, in the limit where inertia is negligible relative to viscosity (zero Reynolds number / Stokes flow). This system is particularly relevant in microscale biological fluid dynamics systems and colloids; one specific application is the calculation of the diffusion tensor of a specified structure. The library also forms an extensible toolkit for problems involving calculations of flow fields due to flexible motions, such as cilia-driven transport, motility / swimming due to flagellar movement or surface deformations, and integration with models of actively bending elastic bodies. The underlying numerical algorithm is the nearest-neighbour implementation (Smith 2018; Gallagher et al. 2019) of regularised stokeslets (Cortez 2001; Cortez et al. 2005); this approach enables efficient and accurate calculations requiring only surface point discretisations rather than surface or volumetric meshes and completely avoids. The calculated flow field is also ensured to be regular throughout the interior, exterior and surface of the body. It is a feature-complete reimplementation and further development of the key test cases in [@Smith_2018].

# Statement of need

Microscale biological flow problems typically involve complex-shaped bodies undergoing large deformations, which renders analytical calculations or methods based on volumetric meshing (finite element, finite difference) challenging to implement. However, in the case where the fluid has Newtonian or approximately Newtonian properties, the flow can be modelled through the linear Stokes flow equations,
\[
-\bm{\nabla} p + \nabla \cdot u = 0, \quad \nabla \cdot u = 0.
\]
Linearity means that the problem can be solved through techniques such as the boundary element method and method of fundamental solutions, however these techniques involve singular solutions which present challenges for numerical implementation. Moreover, despite its high numerical efficiency, the boundary element method (Pozrikidis 1992; Smith 2009) requires the user to supply a true surface mesh with connectivity table and local basis system. The method of regularised stokeslets (Cortez 2001; Cortez et al. 2005) has achieved very significant success in biological fluid dynamics (see for example the Special Issue edited by Cortez & Olson, 2021) by removing both the mathematically singular behaviour, and avoiding the need to construct a true mesh.

In its simplest application, the method of regularised stokeslets enables the solution of the resistance problem, which amounts to determining the force \(F\) and moment \(M\) on a moving rigid body immersed in a viscous fluid. Due to the linearity and instantaneity of the Stokes flow equations, the solution for translational velocity \(U\) and angular velocity \(\Omega\) can be determined from a linear combination of the 'basic' modes corresponding to unit velocity along each cartesian axis and unit angular velocity about each cartesian axis; providing these calculations forms the basic functionality for this package. The \(6\times 6\) matrix of forces and moments corresponding to each basic rigid body model is referred to as the *grand resistance matrix* (Pozrikidis 2002). The inverse of this matrix can be used to calculate the diffusion tensor quantifying the translational and rotational Brownian motion of the body (ref).

Mathematically, the resistance problem is implemented by forming a \(3Q\times 3Q\) matrix of stokeslet evaluations between each combination of \(Q\) surface points (each stokeslet is a \(3\times 3\) rank 2 tensor), then inverting this matrix to determine stokeslet strengths (corresponding to surface forces) from the prescribed surface velocity. These stokeslet strengths can then be summed to calculate total force and moment on the body, and moreover used to evaluate the velocity at any point in the fluid. The mobility problem is a modest generalisation which involves instead prescribing the total force and moment and inverting a \(3(Q+2)\times 3(Q+2)\) matrix for the stokeslet strengths and rigid body motion \(U\), \(\Omega\). More advanced extensions such as the swimming problem involve extending the mobility problem by prescribing a swimming stroke relative to the local frame (Gallagher and Smith 2018), or integrating with an active elastohydrodynamic model (Hall-McNair et al. 2019).

The method of regularised stokeslets is subject to three primary sources of numerical error: (1) regularisation error, (2) discretisation error associated with the surface force distribution, (3) discretisation error associated with the approximation of the surface integral (Cortez 2005; Gallagher and Smith 2019). Sources (1) and (3) have an opposite dependence on the small regularisation \(\epsilon\), the effect of which is that attempting to reduce the regularisation parameter by taking \(\epsilon\rightarrow 0\) leads to a requirement for increasingly refined surface discretisations. Because the number of degrees of freedom is proportional to the number of discretisation points, the effect is that an increasingly large dense matrix must be inverted, resulting in high computational cost. One strategy to address this issue that has been developed is the use of two surface discretisations: a coarse force discretisation with \(N\) points - which determines the number of degrees of freedom, and a finer quadrature discretisation with \(Q\) points - which requires more kernel evaluations but does not affect the size of the matrix to be inverted. Projecting the force to the quadrature discretisation through nearest-neighbour interpolation leads to the 'NEAREST' method (Smith 2018) which achieves very significant improvements in accuracy and efficiency with fairly minimal additional complexity of implementation. The NEAREST method has been applied to model the diffusion tensor of a novel colloidal construct (Smith 2018), the motility of groups of swimming cells (Gallagher and Smith 2018; Gallagher and Smith 2020), revealing the propulsive role of the sperm flagellar endpiece (Neal et al. 2020) and calculating flow in a multiciliated organ (Gallagher and Smith 2020).

Despite the proven effectiveness of the method, a barrier to wider adoption of NEAREST is that the existing implementation underpinning most of the references above is an undocumented research code implemented in Matlab. That implementation primarily uses built-in linear algebra operations throughout the workfrom, from stokeslet evaluation through matrix assembly, and avoids the use of for loops. While this approach has some advantages - notably brief code and very simple exploitation of parallelism (Gallagher and Smith 2020), it tends to result in the storage of large stokeslet matrices, which can be highly inefficient as the quadrature discretisation is refined. Along with the lack of documentation and an extensible framework, the NEAREST code base has not yet been widely adopted, despite its demonstrated advantages on problems of widespread interest in microscale biological flow.

cNEAREST is therefore designed to address the need for an extensible, accessible and efficient methodology for Stokes flow problems. To enable efficient and scalable use of computing resources, the package is implemented in C++, with the widely-used _eigen_ package handling linear algebra operations. The package also implements three specific algorithmic developments, which arose through the process of re-implementing the method in C++:

1. Tiebreaking
2. Stokeslet accumulation
3. Rescaling by local stokeslet count

These changes are described in more detail in the reimplementation section.

# Usage - maybe overlaps too much with the documentation?

The workflow is that the user will first define the problem in terms of physical bodies such as spheres or spheroids (implementation of other shapes and flexible motions is an area for future development)m from which the force and quadrature discretisations are created, and finally defining the boundary conditions. For the rigid body resistance problem with no-slip, no-penetration boundary conditions, the latter follows from supplying the translational and rotational velocity of the body, from which the surface velocity components at each point are calculated.

The library then constructs a linear algebra problem by detecting nearest neighbours, 
managing degrees of freedom, assembling matrices from the regularised stokeslets, and assembling vectors from the boundary conditions.
A linear solver (either the default solver based on LU-decomposition with pivoting or alternative chosen by the user) is invoked to find the unknown force distribution over the nodes.
Finally the same tools which constructed the full size vectors are used to decompose them back to the total physical force and moment on the body. 

# Validation

The results of the code have been compared with the results of the (established) Matlab codebase. This comparison goes to make one of our suites of unit tests.
This code is also capable of reproducing the results tables published in [@Smith_2018] for the force and moment on a translating and rotating sphere and prolate spheroid. This comparison is presented
as example 1 in how to use our code.

# Reimplementation in C++

Reimplenting the functionality previously in Matlab[@gitlab_nearest] into C++ was not solely a language change but also afforded an opportunity to make the code more reusable and efficient.
The library is designed to be adaptable to different problems by making a general interface for discretisations, therefore even 
if our own library does not include a discretisation for the shape a consumer needs, one can be written outside of the 
library and injected in at runtime.
The library is also designed to be adaptable to future innovations by trying to predict potential areas for development 
(such as the linear solver, and the algorithm for splitting between tied neighbours), at these points in the code we 
have injected the dependency in the form of an interface, rather than hard code the options inside the library.

Reimplementation also suggested several material developments in the nearest neighbour algorithm.

1. Tiebreaking refers to the situation in which a quadrature point is equidistant between force points to within double precision floating point error. In this situation, different computer programs, and potentially even different binaries for the same program, may produce different mappings and hence slightly different solutions. cNEAREST provides the option of sharing a stokeslet contribution between two or more force points, and does so smoothly, to minimise this sensitivity.

2. Stokeslet accumulation refers to the method of assembling the left hand side matrix from stokeslet sums. Motivated by the aim of avoiding for-loops, the original NEAREST implementation created a \(3N\times 3Q\) stokeslet matrix then multiplied by a \(3Q\times 3N\) sparse matrix to produce the \(3N\times 3N\) left hand side. As \(3N\times 3Q\) double precision values starts to exceed cache size (**check this**) the calculation becomes inefficient. By contrast, the stokeslet accumulation option in cNEAREST takes the approach of adding the the \(3\times 3\) stokeslet values into the \(3N\times 3N\) matrix one-at-a-time through a for-loop.

3. The final adaptation relates to the fact that the original NEAREST implementation worked by summing stokeslets, so that each degree of freedom corresponded to the 'share' of the local force ascribed to each quadrature point. The effect of this choice was that as the number of quadrature points is increased, the magnitude of the left hand side matrix increases without bound, complicating convergence tests on the left hand side itself. cNEAREST provides the option to rescale by the number of quadrature points local to each force point, so that the left hand side matrix converges as \(Q\) is increased, and each degree of freedom corresponds simply to the force associated with each force point.


# Summary and areas for future development

Potential areas for future development include solution of model cells with flexible flagella, swimming problems (mobility due to a prescribed swimming stroke; Gallagher & Smith 2018), multiple bodies and parallelisation (Gallagher et al. 2020), boundary image systems (Ainley et al. 2008; Cortez & Varela 2015), stokeslet lines (Smith 2009; Cortez), stokeslets with improved far field behaviour (Zhou et al.), integration with elastohydrodynamics (Hall-McNair et al. 2020), stokeslet rings (Tyrrell et al. 2019), Richardson extrapolation (Gallagher et al. 2021), implementation of the double layer potential, which is highly important for problems involving surface slip (Smith et al. 2021), modelling  self-motile phoretic colloids (Montenegro-Johnson et al. 2015) and fast multipole acceleration for very large problems (Rostami and Olson 2016).

# Acknowledgements

This work was partially supported by the Engineering and Physical Sciences Research Council (EP/N021096/1).

We acknowledge contributions from Simon Hartley, whose earlier engagement with this problem uncovered the risk of force points which were tied in for nearest neighbour.

# References

Ainley, J., S. Durkin, R. Embid, P. Boindala, and R. Cortez (2008) The method of images for regularized Stokeslets. J. Comput. Phys. 227, 4600-4616

Cortez, R. (2001) The method of regularized Stokeslets. SIAM J. Sci. Comput. 23, 1204

Cortez, R., L. Fauci and A. Medovikov (2005) The method of regularized Stokeslets in three dimensions: Analysis, validation, and application to helical swimming. Phys. Fluids 17, 031504

Cortez, R. and D. Varela (2015) A general system of images for regularized Stokeslets and other elements near a plane wall. J. Comput. Phys. 285, 41-54

Cortez, R. (2018) Regularized stokeslet segments. J. Comput. Phys. 375, 783-796

Cortez, R. and S. Olson (2021) 20 Years of Regularized Stokeslets: Applications, Computation and Theory. https://www.mdpi.com/journal/fluids/special_issues/regularized_stokeslets

Gallagher, M.T. and D.J. Smith (2018) Meshfree and efficient modelling of swimming cells. Phys. Rev. Fluids 3, 053101

Gallagher, M.T., D. Choudhuri and D.J. Smith (2019) Sharp quadrature error bounds for the nearest-neighbor discretization of the regularized stokeslet boundary integral equation. SIAM J. Sci. Comput. 41(1), B139-B152

Gallagher, M.T. and D.J. Smith (2020) Passively parallel regularized stokeslets. Phil. Trans.
R. Soc. A378: 20190528

Gallagher, M.T. and D.J. Smith (2021) The art of coarse Stokes: Richardson extrapolation
improves the accuracy and efficiency of the method of regularized stokeslets. R. Soc. Open
Sci. 8:210108

Hall-McNair, A.L., M.T. Gallagher, T.D. Montenegro-Johnson, H. Gadelha and D.J. Smith (2019) Efficient implementation of elastohydrodynamics via integral operators. Phys. Rev. Fluids 4, 113101

Montenegro-Johnson, T.D., S. Michelin and E. Lauga (2015) A regularised singularity approach to phoretic problems. Eur. Phys. J. E 38, 1-7

Neal, C.V., A.L. Hall-McNair, J. Kirkman-Brown, D.J. Smith and M.T. Gallagher (2020) Doing more with less: the 
agellar end piece enhances the propulsive effectiveness of spermatozoa. Phys. Rev. Fluids. 5:073101

Rostami, M. and S. Olson (2016) Kernel-independent fast multipole method within the framework of regularized Stokeslets. J. Fluids Struct. 67, 60-84

Pozrikidis, C. (1992) Boundary integral and singularity methods for linearized viscous flow (Cambridge University, Cambridge)

Pozrikidis, C. (2002) A practical guid to boundary element methods with the software library BEMLIB. (CRC Press, Boca Raton, USA)

Smith, D.J. (2009) A boundary element regularised Stokeslet method applied to cilia and flagella-driven flow. Proc. R. Soc. Lond. A, 465, 3605-3626

Smith, D.J. (2018) A nearest-neighbour discretisation of the regularized stokeslet boundary
integral equation. J. Comput. Phys. 358, 88-102

Smith, D.J., M.T. Gallagher, R. Schuech and T.D. Montenegro-Johnson (2021) The role of
the double layer potential in regularized stokeslet models of self-propulsion. Fluids 6, 411

Tyrrell, J.M., D.J. Smith and R.J. Dyson (2019) Regularized Stokeslet rings - an efficient method for axisymmetric Stokes flow, with application to the growing pollen tube. Phys. Rev. Fluids 4:063102

Zhou, B., E. Lauga and L. Koens (2019) Method of regularized stokeslets: Flow analysis and improvement of convergence. Phys. Rev. Fluids 4(8), 084104

** need a ref for diffusion tensor - check Smith 2018 **

Wegener
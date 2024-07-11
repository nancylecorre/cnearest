---
title: 'CNearest: A c++ toolkit to use the nearest neighbour algorithm to solve fluid problems'
tags:
  - C++
  - fluid dynamics
  - regularised stokeslets
  - stokes flow
  - discretisation
authors:
  - name: David Smith
    equal-contrib: true 
    affiliation: 2
  - name: James Tyrrell
    equal-contrib: true 
    affiliation: 1
  - name: Neil A. Butcher
    equal-contrib: true
    affiliation: 1
affiliations:
 - name: ARC, University of Birmingham
   index: 1
 - name: School of Mathematics, University of Birmingham
   index: 2
date: 01 August 2024
bibliography: paper.bib

---

# Summary

# Statement of need

This library contains the tools and to define, and go on to solve, at least 2 types of fluid dynamics problems.

Solving the resistance problem, this is the problem of determining the forces and moment experienced by a body or 
collection of bodies as a result of their motion within the fluid.

Solving the mobility problem, this is the problem of calculating the motion due to an imposed force and moment on
the bodies.

# Usage - maybe overlaps too much with the documentation?

The workflow is that the user will first define the problem in terms of physical bodies, their discretisations, 
and their boundary conditions, such as speeds.
Next the tools offered in this library can compose this into a numerical problem, by detecting nearest neighbours, 
managing degrees of freedom, assembling matrices from the regularised stokeslets, and assembling vectors from the 
boundary conditions.
A linear solver (either the one suggested or some other) is invoked to solve the matrix-vector combinations.
Finally the same tools which constructed the full size vectors are used to decompose them back to physical forces
and moments on the original physical bodies 

# Validation

The results of the code have been compared with the results of the (established) matlab codebase. This comparison 
goes to make one of our suites of unit tests.
This code is also capable of reproducing the results tables published in [@Smith_2018]. This comparison is presented
as example 1 in how to use our code.

# Reimplementation in C++

Reimplenting the functionality previously in matlab[@gitlab_nearest] into c++ was not solely a language change but 
also afforded an opportunity to make the code more reusable.
We made our library adaptable to different problems by making a general interface for discretisations, therefore even 
if our own library does not include a discretisation for the shape a consumer needs, one can be written outside of the 
library and injected in at runtime.
We made the library adaptable to future innovations by trying to predict the parts which are likely to want revisiting 
(such as the linear solver, and the algorithm for splitting between tied neighbours), at these points in the code we 
have injected the dependency in the form of an interface, rather than hard code the options inside the library.

# Acknowledgements

We acknowledge contributions from Simon Hartley, whose earlier attempt to convert this to a c++ package discovered
the risk of nodes which were tied for nearest neighbour.

# References
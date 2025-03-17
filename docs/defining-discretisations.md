# Discretisation Definitions

In order to use this library we first need to define distributions of nodes in space.

We do this using discretisation definitions.

The discretisation is then applied one or more of the rigid bodies in the problem.


## Discretisation Classes

All discretisations are defined by a subclass of `iDiscretisation`, the principle functionality of this class is to return a collection of positions in 3D space.

## Defining the discretisation for your rigid body

There are options for how to define your discretisation

### Library of predefined discretisations

The library provides some standard options for discretising simple shapes. These include:

+ A single point
+ A line with equally spaced nodes
+ A tapered rod
+ A cube
+ A sphere generated from a projection of the cube
+ Fibonacci Sphere


### Empirical discretisation

If you know the exact points which you want to conclude your discretisation, you can make an empirical discretisation directly declaring these points.


### Custom analytical discretisation

You can create your own subclass of `iDiscretisation` outside of this library to use in your own simulation.
It must inherit from `iDiscretisation` and implement the `positions` method.

In this way you can add an analytical, parameterised discretisation on a case by case basis.

### Conglomerate discretisation

2 or more discretisations can be given as arguments to the constructor of a `Conglomerate discretisation` in order to build a new discretisation appending the positions of all the constituant discretisations.
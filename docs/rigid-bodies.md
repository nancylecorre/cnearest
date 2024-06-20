# Rigid bodies

The starting point for the simulation is made by defining a collection of rigid bodies.

Each of these will be discretised into quadrature nodes and force nodes (likely with different discretisation schemes)
See `Defining Discretisations`.

Each rigid body can also be given a translation and a rotation (in the form of a matrix), 
this allows the discretisation to be reused for a few matching rigid bodies at different locations.

Rigid bodies are also important which detecting nearest neighbours. 
While detecting neighbours, only nodes on the same rigid body will be considered.

Once the rigid bodies are defined and discretised, the nodes from all bodies are collected into the simulation.
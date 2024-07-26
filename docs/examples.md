# Example Cases

## Example 1 - sphere

This example recreates tables 2 and 3 in the paper.

This places a single sphere at the origin.

It first defines 2 discretisations, both uniform spheres of radius 1.0 but with different resolutions.

It then uses these 2 to discretise a single rigid body, placed at the origin.

### Force and moment

We then use the nearest neighbour algorithm to detect the nearest neighbours of each node
The 2 resolutions chosen happen to have no exact ties, thus we do not need a tiebreaker algorithm. 

We assign each node a global index with the appropriate class.

We use a matrix assembler and rigid body operations to assemble global matrices and vectors.

These are solved in the linear solver.

The results are mapped back onto the force nodes, then further back onto the rigid body.

This gives the force and moment on the rigid body.

This matches table 2 of the paper.

### Condition numbers

We also perform a second sweep of the different discretisations.
We assemble the mean stokeslet matrix as before.
We then calculate its condition number.

The calculation of the condition number takes a lot longer so it isn't generally performed.

This matches table 3 of the paper



## Example 2 - prolate spheroid

This example recreates Appendix D and table 10 in the paper.

This places a elongated sphere at the origin, it has principle radii 5,1,1, thus it would fit in a box 10x2x2.

It first defines 2 discretisations, both uniform spheres of this shape but with different resolutions.

It then uses these 2 to discretise a single rigid body, placed at the origin.

### Error in the grand resistance matrix

The 2 resolutions do have ties, we use a trivial tiebreaker algorithm, thus leading to differences with the matlab paper. 

We use the `ResistanceProblemSolver` class to encapsulate the steps carried out in example 1.

This class can directly return the resistance matrix.

We compare this result with an expected analytical matrix and report the error between the 2.

This matches table 10 of the paper.

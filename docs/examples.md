# Example Cases

## Example 1 - sphere

This places a single sphere at the origin.

It first defines 2 discretisations, both uniform spheres of radius 1.5 but with different resolutions.

It then uses these 2 to discretise a single rigid body, placed at the origin.

We then use the nearest neighbour algorithm to detect the nearest neighbours of each node
The 2 resolutions chosen happen to have no exact ties, thus we do not need a tiebreaker algorithm. 

We assign each node a global index with the appropriate class.
# Nearest Neighbours

We have a dedicated step to determine the nearest neighbours of the quadrature nodes.

In the source paper, each quadrature node is allocated nearest neighbour, the force node which is closest to it.

This implementation goes slightly beyond the source material, as there is sufficient future-proofing to allow a list of neighbouring force nodes, each with a weight assigned.
This extension is not (yet) used, see Nearest Neighbour Distributer.

Since the result of a nearest neighbour investigation is possibly subjective and not unique, 
we offer the option to write or use a custom `distributer` to decide how to weight different nodes. 


## Results

The nearest neighbour algorithm has a dedicated results class. This class maps each quadrature node to one NearestNeighbourResult.
The NearestNeighbourResult itself is a list of force nodes, each with a weight.
It is predicted that the weights will sum to 1.0.

## Algorithm

There is a dedicated class to produce the nearest neighbour mapping, this has a dedicated results object.
By default it will detect the single nearest neighbour and assign it a weight of 1.0.
There is an optional parameter to define a shell thickness, if 2 force nodes are tied-neighbours (to within this thickness), then an error is thrown.
To avoid the error you can also inject a `distributer` with customised method to decide how to weight the tied force nodes.


## Nearest Neighbour Distributer

Distributers are interchangeable classes with a single method to decide how to distribute weight to nodes which are considered as "tied" by the algorithm.


## Example cases

+ There is a shipped distributer which will assign a weight of 1.0 to the 1st node detected.
+ A distributer to consider would be one which assigns equal weights to all the tied nodes.

## Overruling the algorithm
+ With a very large shell thickness, you can make the algorithm declare all force nodes as tied, then the decision of weightings falls entirely to your distributer.
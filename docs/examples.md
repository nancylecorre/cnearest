# Example Cases

## Example 1 - sphere

This example recreates tables 2 and 3 in the paper [1].

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

This matches table 2 of the paper [1].

### Condition numbers

We also perform a second sweep of the different discretisations.
We assemble the mean stokeslet matrix as before.
We then calculate its condition number.

The calculation of the condition number takes a lot longer so it isn't generally performed.

This matches table 3 of the paper [1].



## Example 2 - prolate spheroid

This example recreates Appendix D and table 10 in the paper [1].

This places a elongated sphere at the origin, it has principle radii 5,1,1, thus it would fit in a box 10x2x2.

It first defines 2 discretisations, both uniform spheres of this shape but with different resolutions.

It then uses these 2 to discretise a single rigid body, placed at the origin.

### Error in the grand resistance matrix

The 2 resolutions do have ties, we use a trivial tiebreaker algorithm, thus leading to differences with the matlab paper. 

We use the `ResistanceProblemSolver` class to encapsulate the steps carried out in example 1.

This class can directly return the resistance matrix.

We compare this result with an expected analytical matrix and report the error between the 2.

This matches table 10 of the paper [1].



## References

1. Smith, D. J. (2018). A nearest-neighbour discretisation of the regularized stokeslet boundary integral equation. Journal of Computational Physics, 358, 88-102.

## Tables as in the reference

The tables from [1] are repeated here for convenience:

#### Table 2.
Relative error for the resistance problem of a unit sphere undergoing rigid body motion in Stokes flow in an infinite fluid; regularization parameter *ϵ* = 0.01. 

(a) Translation with velocity **U** = (1,0,0). 

(b) Rotation with angular velocity **Ω** = (1,0,0).

#### (a) Translation

|    N   |  DOF    | *h<sub>f</sub>*| Q:              | 864   | 3456  | 13824 | 55296 | 221184 |
|--------|---------|----------------|-----------------|-------|-------|-------|-------|--------|
| 54     | 162     | 0.5796         |                 | 0.0147| 0.0052| 0.0002| 0.0006| 0.0012 |
| 216    | 648     | 0.2942         |                 | 0.0166| 0.0079| 0.0038| 0.0022| 0.0020 |
| 864    | 2592    | 0.1611         |                 | 0.1262| 0.0083| 0.0043| 0.0027| 0.0025 |
| 3456   | 10368   | 0.0826         |                 | NaN   | 0.0277| 0.0043| 0.0028| 0.0025 |

#### (b) Rotation

|    N   |  DOF    | *h<sub>f</sub>*| Q:              | 864   | 3456  | 13824 | 55296 | 221184 |
|--------|---------|----------------|---------------- |-------|-------|-------|-------|--------|
| 54     | 162     | 0.5796         |                 | 0.0300| 0.0086| 0.0019| 0.0036| 0.0047 |
| 216    | 648     | 0.2942         |                 | 0.0378| 0.0182| 0.0095| 0.0063| 0.0058 |
| 864    | 2592    | 0.1611         |                 | 0.2193| 0.0194| 0.0109| 0.0078| 0.0074 |
| 3456   | 10368   | 0.0826         |                 | NaN   | 0.0495| 0.0110| 0.0080| 0.0075 |

#### Table 3. 
[Condition number](https://www.sciencedirect.com/topics/engineering/condition-number) for the Stokeslet matrix associated with the solution of the translation and rotation problems for a unit sphere in Stokes flow; regularization parameter *ϵ* = 0.01.

| N       |   DOF   |  Q:  |   864     |   3456    |   13824   |   55296   |   221184   |
|---------|---------|-----|-----------|-----------|-----------|-----------|------------|
| 54      | 162     |     | 132.807   | 903.874   | 415.837   | 292.569   | 260.382    |
| 216     | 648     |     | 100.744   | 242.112   | 904.001   | 3344.192  | 1721.377   |
| 864     | 2592    |     | 8.129     | 218.433   | 638.954   | 4823.278  | 15637.232  |
| 3456    | 10368   |     | Inf       | 38.309    | 617.308   | 3696.167  | 1870680.776|


#### Table 10. 
Test results for the grand resistance tensor of rigid body motion of a prolate spheroid with long semi-axis 5, short semi-axis 1; regularization parameter *ϵ* = 0.01.

|    N   |  DOF    | *h<sub>f</sub>*| Q :             | 864       | 3456      | 13824     | 55296     | 221184     |
|--------|---------|----------------|-----------------|-----------|-----------|-----------|-----------|------------|
| 54     | 162     | 1.0064         |                 | 0.0596    | 0.0221    | 0.0096    | 0.0045    | 0.0052     |
| 216    | 648     | 0.4305         |                 | 0.0459    | 0.0180    | 0.0079    | 0.0036    | 0.0021     |
| 864    | 2592    | 0.2171         |                 | 0.3675    | 0.0207    | 0.0097    | 0.0049    | 0.0034     |
| 3456   | 10368   | 0.1117         |                 | NaN       | 0.1171    | 0.0099    | 0.0052    | 0.0036     |



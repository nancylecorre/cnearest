# Example 3: Mobility problem
 
This example solves the mobility problem for a rigid body in Stokes flow. Instead of imposing the velocities (as in the resistance problem), we impose a total force F and a total moment M on the body and compute the translational velocity U and the rotational velocity Ω that result. The test case uses F = (6π, 0, 0) and M = (8π, 0, 0). The surface is discretised with the nearest-neighbour method, using two grids:
 
- N force points (coarse grid), which carry the unknowns, so the matrix to invert stays small.
- Q quadrature points (fine grid), used to integrate the total force and moment accurately.
## Method
 
The classical Nyström approach would need the full (3N+6) × (3N+6) matrix. I did not want to rebuild that by hand and duplicate code, so the example reuses what `cnearest` already provides: `MatrixAssemblerByBlockAccumulation` builds the 3N × 3N influence matrix, and `LinearSolverLUDirect` solves the system, as in the resistance problem.
 
To get U and Ω, the code builds the 6 × 6 global resistance matrix R:
 
1. The 6 unit motions (3 translations, 3 rotations) are imposed one after the other with `determine_nodal_speeds`.
2. For each one, the solver gives the nodal forces, and `sum_nodal_forces` adds them up to get the total force and moment. Each case fills one column of R.
3. Eigen then solves the small 6 × 6 system R [U, Ω]ᵀ = [F, M]ᵀ.
## Requirements
 
- The `cnearest` project (this repository) and its dependencies: Eigen3, BLAS, LAPACK, Threads
- CMake >= 3.5
## Build
 
From the root of the repository:
 
```
cmake -S . -B build
cmake --build build --target example3mobility
```
 
## Run
 
```
./build/examples/example3mobility
```
 
## Expected output
 
The program prints the translational velocity U and the rotational velocity Ω obtained for the imposed F and M.
 


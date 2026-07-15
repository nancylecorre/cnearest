# Older CPUs

## The problem

Attempting to run CNearest on workstations equipped with Broadwell 
CPUs (for example Xeon CPU E5-2640 v4) can result in an 'Illegal Instruction (Core Dumped)' error at the point that the linear solver is called.

## The cause

This error appears to be caused by OpenBLAS using AVX-512 instructions, which are not supported by Haswell/Broadwell era processors.

## The solution

The problem can be solved by:

1. Building OpenBLAS from source, with the correct CPU target
2. Forcing cmake to point to the new OpenBLAS library

## Detailed steps (for Linux)

### Building OpenBLAS from source

git clone https://github.com/OpenMath/OpenBLAS

cd OpenBLAS

make TARGET=HASWELL

sudo make PREFIX=/usr/local install

### Forcing cmake to point to the new OpenBLAS library

cmake -DBLAS_LIBRARIES=/usr/local/lib/libopenblas.so -DLAPACK_LIBRARIES=/usr/local/lib/libopenblas.so

make

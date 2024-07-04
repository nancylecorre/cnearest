#include "gtest/gtest.h"
#include "linear_solve/linear_solver_lu_direct.h"


 TEST(LinearSolveTest, test_linear_solver) {
    cnearest::Matrix a = cnearest::Matrix::Random(150, 150);
    cnearest::Vector b = cnearest::Vector::Random(150);

    cnearest::LinearSolverLUDirect solver;

    auto x = solver.solve(a, b);

    auto test = a * x;

    for (size_t i = 0; i < 150; i++) ASSERT_NEAR(test(i), b(i), 1.0e-12);

 }

 
 TEST(LinearSolveTest, test_linear_solver_multiple) {
     cnearest::Matrix a = cnearest::Matrix::Random(150, 150);
     cnearest::Vector b0 = cnearest::Vector::Random(150);
     cnearest::Vector b1 = cnearest::Vector::Random(150);

     cnearest::Collection<const cnearest::Vector*> bs = {&b0, &b1};

     cnearest::LinearSolverLUDirect solver;

     auto xs = solver.solve(a, bs);

     auto test0 = a * xs[0];
     auto test1 = a * xs[1];

     for (size_t i = 0; i < 150; i++) {
         ASSERT_NEAR(test0(i), b0(i), 1.0e-12);
         ASSERT_NEAR(test1(i), b1(i), 1.0e-12);
     }
 }
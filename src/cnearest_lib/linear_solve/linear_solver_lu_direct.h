#pragma once

#include "ilinear_solver.h"

namespace cnearest {

    class LinearSolverLUDirect : public iLinearSolver {
       public:
        LinearSolverLUDirect() = default;
        ~LinearSolverLUDirect() = default;

        /// <summary>
        /// Right hand side solve of Ax=B
        /// </summary>
        /// <param name="B"> the right hand side vector to solve</param>
        Vector solve(const Matrix& A, const Vector& B) const override;

        /// <summary>
        /// Right hand side solve of Ax=B
        /// For multiple values of B simultaneously
        /// This may work out faster if decompositions or intermediate steps can be cached
        /// </summary>
        /// <param name="B"> the right hand side vector to solve</param>
        Collection<Vector> solve(const Matrix& A, const Collection<const Vector*>& B) const override;
    };
}

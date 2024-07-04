#pragma once
#include "definitions.h"

namespace cnearest {

    /// <summary>
    /// An interface (interchangeable class), offering linear solve capabilities
    /// Right hand side solve of Ax=B
    /// We assume A is square
    /// </summary>
    class iLinearSolver {
       public:
        virtual ~iLinearSolver(){};

        /// <summary>
        /// Right hand side solve of Ax=B
        /// </summary>
        /// <param name="B"> the right hand side vector to solve</param>
        virtual Vector solve(const Matrix& A, const Vector& B) const = 0;

        /// <summary>
        /// Right hand side solve of Ax=B
        /// For multiple values of B simultaneously
        /// This may work out faster if decompositions or intermediate steps can be cached
        /// </summary>
        /// <param name="B"> the right hand side vector to solve</param>
        virtual Collection<Vector> solve(const Matrix& A, const Collection<const Vector*>& B) const = 0;
    };
}

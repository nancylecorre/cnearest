#include "linear_solver_lu_direct.h"
#include <algorithm>

namespace cnearest {
    Vector LinearSolverLUDirect::solve(const Matrix& A, const Vector& B) const {
        auto lu_decomposition = Eigen::PartialPivLU<cnearest::Matrix>(A);
        return lu_decomposition.solve(B);
    }

    Collection<Vector> LinearSolverLUDirect::solve(const Matrix& A,
                                                 const Collection<const Vector*>& B) const {
        auto lu_decomposition = Eigen::PartialPivLU<cnearest::Matrix>(A);
        Collection<Vector> result(B.size());
        std::transform(B.begin(), B.end(), result.begin(),
                       [&lu_decomposition](const Vector* b) { return lu_decomposition.solve(*b); });
        return result;
    }
}
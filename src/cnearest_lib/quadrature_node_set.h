#pragma once
#include "quadrature_node.h"

namespace cnearest {

    /// <summary>
    /// The result of a quadrature discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class QuadratureNodeSet {
       public:
        QuadratureNodeSet(const Collection<QuadratureNode>& nodes);
        ~QuadratureNodeSet();

        const Collection<QuadratureNode>& nodes() const;
        const size_t size() const;

       private:
        Collection<QuadratureNode> _nodes;
    };
}
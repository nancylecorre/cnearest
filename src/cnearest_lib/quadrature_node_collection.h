#pragma once
#include "quadrature_node.h"

namespace cnearest {

    /// <summary>
    /// The result of a quadrature discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class QuadratureNodeCollection {
       public:
        QuadratureNodeCollection(const Collection<QuadratureNode>& nodes);
        QuadratureNodeCollection(const Collection<Vector3>& positions);
        ~QuadratureNodeCollection();

        const Collection<QuadratureNode>& nodes() const;
        const size_t size() const;

        /// <summary>
        /// The maximum discretisation spacing of the quadrature nodes
        /// The max distance of a point to its nearest-neighbour
        /// hq
        /// Ref: j.jcp2017.12.008, eqn 28b.
        /// </summary>
        double quadrature_discretisation_spacing() const;

       private:
        Collection<QuadratureNode> _nodes;
    };
}
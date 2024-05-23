#pragma once
#include "definitions.h"

namespace cnearest {
    /// <summary>
    /// Nodes created by the quadrature discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class QuadratureNode {
       public:
        QuadratureNode(const Vector3& position);
        ~QuadratureNode();

        /// <summary>
        /// The position fo the quadrature node
        /// X[i],
        /// Ref: j.jcp2017.12.008, eqn 20.
        /// </summary>
        const Vector3& position() const;

       private:
        Vector3 _position;
    };
}
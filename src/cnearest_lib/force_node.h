#pragma once
#include "definitions.h"

namespace cnearest {
    /// <summary>
    /// Nodes created by the force discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class ForceNode {
       public:
        ForceNode(const Vector3& position);
        ~ForceNode();

        /// <summary>
        /// The position fo the force node
        /// x[i],
        /// Ref: j.jcp2017.12.008, eqn 20.
        /// </summary>
        const Vector3& position() const;

       private:
        Vector3 _position;
    };
}
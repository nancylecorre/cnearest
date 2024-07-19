#pragma once
#include "force_node.h"

namespace cnearest {
    /// <summary>
    /// The result of a force discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class ForceNodeCollection {
       public:
        ForceNodeCollection(const Collection<ForceNode>& nodes);
        ForceNodeCollection(const Collection<Vector3>& positions);
        ~ForceNodeCollection();

        const Collection<ForceNode>& nodes() const;
        const size_t size() const;

        /// <summary>
        /// The maximum discretisation spacing of the force nodes
        /// The max distance of a point to its nearest-neighbour
        /// hf
        /// Ref: j.jcp2017.12.008, eqn 28a.
        /// </summary>
        double force_discretisation_spacing() const;

       private:
        Collection<ForceNode> _nodes;
    };
}
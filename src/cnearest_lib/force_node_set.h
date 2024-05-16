#pragma once
#include "force_node.h"

namespace cnearest {
    /// <summary>
    /// The result of a force discretisation
    /// Ref: j.jcp2017.12.008, chapter 4.
    /// </summary>
    class ForceNodeSet {
       public:
        ForceNodeSet(const Collection<ForceNode>& nodes);
        ~ForceNodeSet();

        const Collection<ForceNode>& nodes() const;

       private:
        Collection<ForceNode> _nodes;
    };
}
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
        ~ForceNodeCollection();

        const Collection<ForceNode>& nodes() const;
        const size_t size() const;

       private:
        Collection<ForceNode> _nodes;
    };
}
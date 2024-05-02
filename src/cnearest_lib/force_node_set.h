#pragma once
#include "force_node.h"

namespace cnearest {
class ForceNodeSet {
   public:
    ForceNodeSet(const Collection<ForceNode>& nodes);
    ~ForceNodeSet();

    const Collection<ForceNode>& nodes() const;

   private:
    Collection<ForceNode> _nodes;
};
} 
#pragma once
#include "definitions.h"
#include "force_node_set.h"
#include "idiscretisation.h"
#include "quadrature_node_set.h"

namespace cnearest {

    class RigidBody {
       public:
        RigidBody(const iDiscretisation& force_discretisation,
                  const iDiscretisation& quadrature_discretisation);
        ~RigidBody() = default;

        const ForceNodeSet& force_nodes() const;
        const QuadratureNodeSet& quadrature_nodes() const;

       private:
        ForceNodeSet _force_nodes;
        QuadratureNodeSet _quadrature_nodes;
    };

}

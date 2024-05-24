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
        /// <summary>
        /// Rigid body constructed by displacing other discretisation,
        /// Optional rotation matrix also,
        /// Any rotation is applied BEFORE the displacement
        /// (this means the displacement entered is in the global frame)
        /// </summary>
        RigidBody(const iDiscretisation& force_discretisation,
                  const iDiscretisation& quadrature_discretisation,
                  const Vector3& displacement,
                  const Matrix3 rotation = Matrix3::Identity());
        ~RigidBody() = default;

        const ForceNodeSet& force_nodes() const;
        const QuadratureNodeSet& quadrature_nodes() const;

       private:
        ForceNodeSet _force_nodes;
        QuadratureNodeSet _quadrature_nodes;
    };

}

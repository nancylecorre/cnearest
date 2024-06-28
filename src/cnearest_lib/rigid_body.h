#pragma once
#include "definitions.h"
#include "force_node_collection.h"
#include "idiscretisation.h"
#include "quadrature_node_collection.h"

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
                  const iDiscretisation& quadrature_discretisation, const Vector3& displacement,
                  const Matrix3& rotation = Matrix3::Identity());
        ~RigidBody() = default;

        const ForceNodeCollection& force_nodes() const;
        const QuadratureNodeCollection& quadrature_nodes() const;

       private:
        ForceNodeCollection _force_nodes;
        QuadratureNodeCollection _quadrature_nodes;
    };

}

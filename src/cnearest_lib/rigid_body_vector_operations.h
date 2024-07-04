#pragma once
#include <map>

#include "rigid_body.h"

namespace cnearest {

    struct SixDOFVector {
        Vector3 linear_component;
        Vector3 angular_component;
    };

    class RigidBodyVectorOperations {
       public:
        RigidBodyVectorOperations(const RigidBody& rigid_body);

        const RigidBody& rigid_body() const;

        SixDOFVector sum_nodal_forces(const std::map<const ForceNode*, Vector3>& nodal_values) const;
        std::map<const ForceNode*, Vector3> determine_nodal_speeds
            (const SixDOFVector& rigid_body_speed) const;


       private:
        const RigidBody& _rigid_body;
    };

}

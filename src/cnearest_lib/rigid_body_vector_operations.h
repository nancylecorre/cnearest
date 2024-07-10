#pragma once
#include <map>

#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_results.h"

namespace cnearest {

    /// <summary>
    /// A six degree of freedom vector, with a linear component and an angular component
    /// Examples include a force+moment, or a velocity+angular velocity
    /// They will not typically have the same unit
    /// </summary>
    struct SixDOFVector {
        Vector3 linear_component;
        Vector3 angular_component;
    };

    /// <summary>
    /// A toolset of operations
    /// It can map 6dof velocities on a rigid body to nodal velocities
    /// It can accumulate nodal forces on a rigid body to a 6dof force and moment
    /// Note that these operations are difficult to reverse due to non-uniqueness
    /// </summary>
    class RigidBodyVectorOperations {
       public:
        RigidBodyVectorOperations(const RigidBody& rigid_body);

        const RigidBody& rigid_body() const;

        /// <summary>
        /// Sum a collection of nodal forces to give a force and a moment on the rigid body
        /// Use the force node position to determine the rigid arm length
        /// </summary>
        SixDOFVector sum_nodal_forces(const std::map<const ForceNode*, Vector3>& nodal_values) const;

        
        /// <summary>
        /// Sum a collection of nodal forces to give a force and a moment on the rigid body
        /// make use of the nearest neighbour results to determine the average quadrature node position to get rigid arm length
        /// </summary>
        SixDOFVector sum_nodal_forces(const std::map<const ForceNode*, Vector3>& nodal_values, const NearestNeighbourResults& nn) const;

        /// <summary>
        /// Determine nodal velocities from a given rigid body velocity (6dof)
        /// </summary>
        std::map<const ForceNode*, Vector3> determine_nodal_speeds(
            const SixDOFVector& rigid_body_speed) const;

       private:
        const RigidBody& _rigid_body;
    };

}

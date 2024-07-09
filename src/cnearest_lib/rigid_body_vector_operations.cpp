#include "rigid_body_vector_operations.h"

namespace cnearest {

    RigidBodyVectorOperations::RigidBodyVectorOperations(const RigidBody& rigid_body)
        : _rigid_body(rigid_body) {}

    const RigidBody& RigidBodyVectorOperations::rigid_body() const { return _rigid_body; }

    SixDOFVector RigidBodyVectorOperations::sum_nodal_forces(
        const std::map<const ForceNode*, Vector3>& nodal_values) const {
        SixDOFVector result = {{0, 0, 0}, {0, 0, 0}};
        Vector3 origin_point = _rigid_body.origin_point();

        for (auto& [node, contribution] : nodal_values) {
            Vector3 rigid_arm_vector = node->position() - origin_point;
            result.linear_component += contribution;
            result.angular_component += rigid_arm_vector.cross(contribution);
        }

        return result;
    }

    std::map<const ForceNode*, Vector3> RigidBodyVectorOperations::determine_nodal_speeds(
        const SixDOFVector& rigid_body_speed) const {
        std::map<const ForceNode*, Vector3> result = {};
        Vector3 origin_point = _rigid_body.origin_point();

        for (auto& node : _rigid_body.force_nodes().nodes()) {
            Vector3 rigid_arm_vector = node.position() - origin_point;
            Vector3 speed = rigid_body_speed.linear_component +
                            rigid_body_speed.angular_component.cross(rigid_arm_vector);
            result.insert({&node, speed});
        }

        return result;
    }
}

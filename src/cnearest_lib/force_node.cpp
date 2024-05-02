#include "force_node.h"

namespace cnearest {

ForceNode::ForceNode(const Vector3& position, const Vector3& velocity):
	_position(position), 
	_velocity(velocity){
}

ForceNode::~ForceNode() {}

const Vector3& ForceNode::position() const { return _position; }
const Vector3& ForceNode::velocity() const { return _velocity; }
}
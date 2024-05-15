#include "force_node.h"

namespace cnearest {

ForceNode::ForceNode(const Vector3& position):
	_position(position){
}

ForceNode::~ForceNode() {}

const Vector3& ForceNode::position() const { return _position; }
}
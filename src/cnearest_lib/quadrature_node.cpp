#include "quadrature_node.h"

namespace cnearest {

    QuadratureNode::QuadratureNode(const Vector3& position) : _position(position) {}

    QuadratureNode::~QuadratureNode() {}

    const Vector3& QuadratureNode::position() const { return _position; }
}
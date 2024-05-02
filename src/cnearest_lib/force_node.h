#pragma once
#include "definitions.h"

namespace cnearest {
    class ForceNode {
      public:
        ForceNode(const Vector3& position, const Vector3& velocity);
        ~ForceNode();
    
        const Vector3& position() const;
        const Vector3& velocity() const;

      private:
        Vector3 _position;
        Vector3 _velocity;

    };
}
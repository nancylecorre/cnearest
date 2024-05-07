#pragma once
#include "idiscretisation.h"

namespace cnearest {
/// <summary>
/// Discretisation scheme giving points all around a sphere
/// </summary>
class UniformSphericalDescretisation : public iDescretisation {
   public:
    UniformSphericalDescretisation(double radius, unsigned int msph)
        : _radius(radius), _msph(msph){};
    ~UniformSphericalDescretisation() = default;

    Collection<Vector3> positions() const override;

   private:
    double _radius;
    unsigned int _msph;
};
}  // namespace cnearest
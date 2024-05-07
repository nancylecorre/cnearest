#pragma once
#include "idiscretisation.h"


namespace cnearest {
/// <summary>
/// Discretisation scheme giving a single point with defined global coordinates
/// </summary>
class SinglePointDescretisation : public iDescretisation {
   public:
    SinglePointDescretisation() : _x(0), _y(0), _z(0){};
    SinglePointDescretisation(double x, double y, double z) : _x(x), _y(y), _z(z){};
    ~SinglePointDescretisation(){};

    Collection<Vector3> positions() const override {
        return { {_x, _y, _z} };
    }

   private:
    double _x, _y, _z;
};
}
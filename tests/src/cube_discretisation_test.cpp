#include "discretisations/single_point_discretisation.h"

/// <summary>
/// The 8 corners of a unit-cube
/// </summary>
class CubeCornersDiscretisation : public iDiscretisation {
   public:
    CubeCornersDiscretisation(){};
    ~CubeCornersDiscretisation() = default;

    Collection<Vector3> positions() const override {
        Collection<Vector3> result;

        result.push_back({1, 1, 1});
        result.push_back({1, 1, -1});
        result.push_back({1, -1, 1});
        result.push_back({1, -1, -1});
        result.push_back({-1, 1, 1});
        result.push_back({-1, 1, -1});
        result.push_back({-1, -1, 1});
        result.push_back({-1, -1, -1});

        return result;
    }
};
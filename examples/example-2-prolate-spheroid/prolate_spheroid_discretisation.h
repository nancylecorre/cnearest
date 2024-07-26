#include "discretisations/uniform_spherical_discretisation.h"

class ProlateSpheroidDiscretisation : public cnearest::iDiscretisation {
   public:
    ProlateSpheroidDiscretisation(cnearest::Vector3 radius, unsigned int msph)
        : _radius(radius), _msph(msph){};

    cnearest::Collection<cnearest::Vector3> positions() const override {
        cnearest::Collection<cnearest::Vector3> result;

        cnearest::UnitCubeDiscretisation cube(_msph);
        // rescale the unit cube to unit radius, then scale the 3 axis differently with a coefficient-wise-produce
        for (auto& position : cube.positions()) {
            result.emplace_back(position.normalized().cwiseProduct(_radius));
        }
        return result;
    }
   private:
    cnearest::Vector3 _radius;
    unsigned int _msph;
};

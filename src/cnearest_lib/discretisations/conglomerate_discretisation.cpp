#include "conglomerate_discretisation.h"

namespace cnearest {

    ConglomerateDiscretisation::ConglomerateDiscretisation(
        const iDiscretisation& d1, const iDiscretisation& d2)
        : _discretisations({&d1, &d2}) {

    }

    Collection<Vector3> ConglomerateDiscretisation::positions() const {
        Collection<Vector3> result;
        for (auto& d : _discretisations) {
            auto p = d->positions();
            result.insert(result.end(), p.begin(), p.end());
        }
        return result;
    }
}

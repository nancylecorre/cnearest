#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme appending 2 or more other schemes
    /// </summary>
    class ConglomerateDiscretisation : public iDiscretisation {
       public:
        ConglomerateDiscretisation(
            const Collection<const iDiscretisation*>& discretisations)
            : _discretisations(discretisations){};

        ConglomerateDiscretisation(const iDiscretisation& discretisation1,
                                   const iDiscretisation& discretisation2);

        Collection<Vector3> positions() const override;

       private:
        Collection<const iDiscretisation*> _discretisations;
    };
}

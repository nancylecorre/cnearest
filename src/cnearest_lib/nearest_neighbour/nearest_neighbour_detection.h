#pragma once
#include <map>
#include <memory>

#include "definitions.h"
#include "inearest_neighbour_distributer.h"
#include "nearest_neighbour_results.h"
#include "rigid_body.h"

namespace cnearest {

    /// <summary>
    /// Algorithm to detect the nearest neighbours of each quadrature node
    /// Nearest neighbour interpolation N(q)
    /// Ref: j.jcp2017.12.008, eqn 20
    /// </summary>
    class NearestNeighbourDetectionAlgorithm {
       public:
        /// <summary>
        /// Algorithm to detect the nearest neighbours of each quadrature node
        /// By default ties between nodes occur if the nodes are at exactly the same
        /// distance (overrule by defining a tie_shell_thickness) By default ties will
        /// result in an exception being throw (overrule by defining a distributer)
        /// </summary>
        /// <param name="distributer">This is an optional entry to decide how to
        /// distribute between tied neighbours, the choice comes in the form of an
        /// iNearestNeighbourDistributer</param> <param name="tie_shell_thickness">This
        /// values declares what constitutes a tie between neighbours, a non-zero value
        /// will mean that very close calls are delegated to the distributer</param>
        NearestNeighbourDetectionAlgorithm(
            std::unique_ptr<iNearestNeighbourDistributer> distributer = nullptr,
            double tie_shell_thickness = 0.0);
        ~NearestNeighbourDetectionAlgorithm() = default;

        /// <summary>
        /// For a given collection of rigid bodies, this generates a single mapping from
        /// each quadrature node to a force node Nearest neighbour interpolation N(q) Ref:
        /// j.jcp2017.12.008, eqn 20
        /// </summary>
        /// <returns>The results of the nearest neighbour investigation in the form of a
        /// NearestNeighbourResults</returns>
        NearestNeighbourResults nearest_neighbours(
            const Collection<RigidBody>& bodies) const;

       private:
        std::unique_ptr<iNearestNeighbourDistributer> _distributer;
        double _tie_shell_thickness;

        void populate_nearest_neighbours(
            const QuadratureNode& node, const RigidBody& body,
            std::map<const QuadratureNode*, NearestNeighbourResult>& results) const;
    };

}

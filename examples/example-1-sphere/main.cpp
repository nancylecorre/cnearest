#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"

int main(int argc, char *argv[]) {

    //define sphere parameters
    double radius = 1.5;
    int force_resolution = 2;
    int quadrature_resolution = 6;

    // Define regularisation
    double epsilon = 0.01;

    // Create discretisation
    cnearest::UniformSphericalDiscretisation force_discretisation(
        radius, force_resolution);
    cnearest::UniformSphericalDiscretisation quadrature_discretisation(
        radius, quadrature_resolution);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quadrature_discretisation)
    };

    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm;
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);

    // Construct the assembled matrix
    cnearest::MatrixAssemblerByNNMultiplication m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix resistance_matrix = m.generate_matrix();

}

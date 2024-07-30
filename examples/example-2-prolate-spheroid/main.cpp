#include "prolate_spheroid_discretisation.h"
#include "resistance_problem_solver.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_distributers.h"
#include "analytical_resistance_matrix.h"
#include <iostream>

int main(int argc, char *argv[]) {

    
    // define spheroid parameters
    double radius_x = 5.0;
    double radius_y_z = 1.0;

    // get the anaytic resistance matrix
    auto analytic_matrix = analytic_grand_resistance_matrix(radius_x, radius_y_z);

    for (int n : {3, 6, 12, 24}) {
        for (int q : {12, 24, 48, 96, 192}) {
            if (q < n) continue;



            int force_resolution = n;
            int quadrature_resolution = q;

            // Define regularisation
            double epsilon = 0.01;

            // Create discretisation
            ProlateSpheroidDiscretisation force_discretisation({radius_x, radius_y_z, radius_y_z},
                                                               force_resolution);
            ProlateSpheroidDiscretisation quadrature_discretisation({radius_x, radius_y_z, radius_y_z},
                                                                    quadrature_resolution);

            // Create a collection with one rigid body
            cnearest::Collection<cnearest::RigidBody> bodies{
                cnearest::RigidBody(force_discretisation, quadrature_discretisation)};

            // Create the default resistance problem

            auto distributer = std::make_unique<cnearest::NearestNeighbourDistributerFirst>();
            cnearest::ResistanceProblemSolver solver(bodies, epsilon, std::move(distributer));

            // Solve the resistance matrix 
            auto matrix = solver.grand_resistance_matrix(bodies[0]);

            //determine the error 
            double delta = (matrix - analytic_matrix).norm() / analytic_matrix.norm();

            //print results
            std::cout << "N: " << bodies[0].force_nodes().size() << "\t"
                      << "Q: " << bodies[0].quadrature_nodes().size() << std::endl
                      << std::endl;

            std::cout << "Matrix:" << std::endl << matrix << std::endl << std::endl;

            std::cout << "Relative Error in resistance matrix: " << delta << std::endl;
            
            std::cout << std::endl << std::endl << std::endl;

        }
    }
}

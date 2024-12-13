#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "nearest_neighbour_distributers.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "matrix_assembler_by_kokkos.h"
#include "rigid_body_vector_operations.h"
#include "linear_solve/linear_solver_lu_direct.h"
#include "table_printing.h"
#include <chrono>
#include<Kokkos_Core.hpp>

class MandelbrokSurfaceDiscretisation : public cnearest::iDiscretisation {
   public:
    MandelbrokSurfaceDiscretisation() : _resolution(0), _scale(1.0){};
    MandelbrokSurfaceDiscretisation(int resolution, double scale)
        : _resolution(resolution), _scale(scale){};
    ~MandelbrokSurfaceDiscretisation() = default;

    cnearest::Collection<cnearest::Vector3> positions() const override {
        cnearest::Collection<cnearest::Vector3> result;
        for (int i = 0; i < _resolution; i++) {
            for (int j = 0; j < _resolution; j++) {
                double x = i * 4.0 / _resolution-2.0;
                double y = j * 4.0 / _resolution-2.0;
                if (_is_point_on_surface(x, y,4.0/_resolution)) {
                    result.push_back({x*_scale, y*_scale,0});
                }
            }
        }
        return result;
    }

   private:
    int _resolution;
    double _scale;
    int _max_iteration = 100;

    
    bool _is_point_on_surface(double x0, double y0, double delta) const { 
        if (!_is_point_in_set(x0, y0)) {
            return false;
        }
        int yes_count = 0;
        if (_is_point_in_set(x0 - delta, y0 - delta)) yes_count++;
        if (_is_point_in_set(x0 - delta, y0        )) yes_count++;
        if (_is_point_in_set(x0 - delta, y0 + delta)) yes_count++;
        if (_is_point_in_set(x0        , y0 - delta)) yes_count++;
        if (_is_point_in_set(x0        , y0 + delta)) yes_count++;
        if (_is_point_in_set(x0 + delta, y0 - delta)) yes_count++;
        if (_is_point_in_set(x0 + delta, y0        )) yes_count++;
        if (_is_point_in_set(x0 + delta, y0 + delta)) yes_count++;

        return yes_count >= 3 && yes_count <=5;
    }

    bool _is_point_in_set(double x0, double y0) const {
        double x2 = 0;
        double y2 = 0;
        double w = 0;
        int iteration = 0;

        while (x2 + y2 <= 4 and iteration < _max_iteration) {
            double x = x2 - y2 + x0;
            double y = w - x2 - y2 + y0;
            x2 = x * x;
            y2 = y * y;
            w = (x + y) * (x + y);
            iteration = iteration + 1;
        }
        return iteration == _max_iteration;
    }

};



int main(int argc, char *argv[]) {
    Kokkos::initialize();

    print_table_header({{"N", 8}, {"Q", 8}, {"Force", 15}, {"Moment", 15}, {"Time (s)", 15}});

    for (int quadrature_resolution : {12, 24, 48, 96, 192 , 384}) {
        auto start = std::chrono::high_resolution_clock::now();
        // define sphere parameters
        double radius = 1.0;
        int force_resolution = 6;

        // Define regularisation
        double epsilon = 0.01;

        // Create discretisation
        cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
        cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);

        // Create a collection with one rigid body
        cnearest::Collection<cnearest::RigidBody> bodies{
            cnearest::RigidBody(force_discretisation, quadrature_discretisation)};

        // Detect the nearest neighbours
        cnearest::NearestNeighbourDetectionAlgorithm algorithm;
        auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

        // Assign global ids to the nodes (trivial with one rigid body)
        cnearest::GlobalIndexAssignment index_assignment(bodies);

        // Construct the assembled matrix
        cnearest::MatrixAssemblerByKokkos<false> m(index_assignment, nearest_neighbour_results, epsilon);

        cnearest::Matrix mean_stokeslet_matrix = m.generate_mean_stokeslet_matrix();

        assert(!mean_stokeslet_matrix.hasNaN());

        // give each node the same speed
        cnearest::RigidBodyVectorOperations o(bodies[0]);

        // case a motion in linear direction
        cnearest::Vector3 linear_speed = {1, 0, 0};
        cnearest::Vector3 angular_speed = {0, 0, 0};
        auto nodal_speeds_a = o.determine_nodal_speeds({linear_speed, angular_speed});

        // case b motion in angular direction
        linear_speed = {0, 0, 0};
        angular_speed = {1, 0, 0};
        auto nodal_speeds_b = o.determine_nodal_speeds({linear_speed, angular_speed});

        // build a unified vector of these speeds
        auto u_a = m.assemble_vector_from_nodal_values(nodal_speeds_a);
        auto u_b = m.assemble_vector_from_nodal_values(nodal_speeds_b);

        // perform an lu decomposition to facilitate the solve
        // solve to get a unified vector of all the forces
        cnearest::Collection<const cnearest::Vector *> us({&u_a, &u_b});
        auto f = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

        // decompose this to nodal forces
        auto nodal_forces_a = m.disassemble_vector_to_force_nodes(f[0]);
        auto nodal_forces_b = m.disassemble_vector_to_force_nodes(f[1]);

        // sum these to get the rigid body force
        auto summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_a, nearest_neighbour_results);

        // compare with the predicted stokes force
        double force = summed_rigid_body_force.linear_component.x();

        // sum these to get the rigid body moment
        summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_b, nearest_neighbour_results);

        // compare with the predicted stokes force
        double moment = summed_rigid_body_force.angular_component.x();


        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

        print_table_row({{(double)index_assignment.force_node_indicies().size(), 0, 8},
                         {(double)index_assignment.quadrature_node_indicies().size(), 0, 8},
                         {force, 8, 15},
                         {moment, 8, 15},
                         {duration, 8, 15}});
    }

    print_table_header({{"N", 8}, {"Q", 8}, {"Force", 15}, {"Moment", 15}, {"Time (s)", 15}});

    int force_resolution = 200;

    for (int quadrature_resolution = 10000; quadrature_resolution < 20000; quadrature_resolution += 400) {

        auto start = std::chrono::high_resolution_clock::now();
        // Define regularisation
        double epsilon = 0.01;

        // Create discretisation
        MandelbrokSurfaceDiscretisation force_discretisation(force_resolution, 1.0);
        MandelbrokSurfaceDiscretisation quadrature_discretisation(quadrature_resolution, 1.0);

        // Create a collection with one rigid body
        cnearest::Collection<cnearest::RigidBody> bodies{
            cnearest::RigidBody(force_discretisation, quadrature_discretisation)};

        // Detect the nearest neighbours
        cnearest::NearestNeighbourDetectionAlgorithm algorithm(
            std::make_unique<cnearest::NearestNeighbourDistributerFirst>());
        auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

        // Assign global ids to the nodes (trivial with one rigid body)
        cnearest::GlobalIndexAssignment index_assignment(bodies);

        // Construct the assembled matrix
        cnearest::MatrixAssemblerByKokkos<false> m(index_assignment, nearest_neighbour_results, epsilon);

        cnearest::Matrix mean_stokeslet_matrix = m.generate_mean_stokeslet_matrix();

        assert(!mean_stokeslet_matrix.hasNaN());

        // give each node the same speed
        cnearest::RigidBodyVectorOperations o(bodies[0]);

        // case a motion in linear direction
        cnearest::Vector3 linear_speed = {1, 1, 0};
        cnearest::Vector3 angular_speed = {0, 0, 0};
        auto nodal_speeds_a = o.determine_nodal_speeds({linear_speed, angular_speed});

        // build a unified vector of these speeds
        auto u_a = m.assemble_vector_from_nodal_values(nodal_speeds_a);

        // perform an lu decomposition to facilitate the solve
        // solve to get a unified vector of all the forces
        cnearest::Collection<const cnearest::Vector *> us({&u_a});
        auto f = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

        // decompose this to nodal forces
        auto nodal_forces_a = m.disassemble_vector_to_force_nodes(f[0]);

        // sum these to get the rigid body force
        auto summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_a, nearest_neighbour_results);

        // compare with the predicted stokes force
        double force = summed_rigid_body_force.linear_component.x();
        // compare with the predicted stokes force
        double moment = summed_rigid_body_force.angular_component.z();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

        print_table_row({{(double)index_assignment.force_node_indicies().size(), 0, 8},
                         {(double)index_assignment.quadrature_node_indicies().size(), 0, 8},
                         {force, 8, 15},
                         {moment, 8, 15},
                         {duration, 8, 15}});
    }
}
    
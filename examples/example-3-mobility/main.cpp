#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "rigid_body_vector_operations.h"
#include "linear_solve/linear_solver_lu_direct.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense> // Inclus pour résoudre la petite matrice 6x6 à la fin
int main(int argc, char *argv[]) {
// Paramètres de la sphère (idem au code Python)
double radius = 1.0;
int force_resolution = 6;
int quadrature_resolution = 12;
double epsilon = 0.01;
// Create discretisations
cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);
// Create a collection with one rigid body
cnearest::Collection<cnearest::RigidBody> bodies{
cnearest::RigidBody(force_discretisation, quadrature_discretisation)
};
// Detect the nearest neighbours
cnearest::NearestNeighbourDetectionAlgorithm algorithm;
auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);
// Assign global ids to the nodes (trivial with one rigid body)
cnearest::GlobalIndexAssignment index_assignment(bodies);
// Assemblage de la Matrice de Stokeslet 3Nx3N
cnearest::MatrixAssemblerByBlockAccumulation<false> m(index_assignment, nearest_neighbour_results, epsilon);
cnearest::Matrix mean_stokeslet_matrix = m.generate_mean_stokeslet_matrix();

// 3. Préparation des 6 mouvements fondamentaux (3 Translations, 3 Rotations)
cnearest::RigidBodyVectorOperations o(bodies[0]);
std::vector<cnearest::Vector> u_vectors;

std::vector<const cnearest::Vector*> u_pointers;

std::vector<cnearest::SixDOFVector> unit_motions = {
{{1, 0, 0}, {0, 0, 0}}, // Translation X
{{0, 1, 0}, {0, 0, 0}}, // Translation Y
{{0, 0, 1}, {0, 0, 0}}, // Translation Z
{{0, 0, 0}, {1, 0, 0}}, // Rotation X
{{0, 0, 0}, {0, 1, 0}}, // Rotation Y
{{0, 0, 0}, {0, 0, 1}} // Rotation Z
};

// Création des Right-Hand Sides (Vecteurs U)
for (const auto& motion : unit_motions) {
auto nodal_speeds = o.determine_nodal_speeds(motion);
u_vectors.push_back(m.assemble_vector_from_nodal_values(nodal_speeds));
}

// Stockage des pointeurs pour le solveur
for (const auto& u : u_vectors) {
u_pointers.push_back(&u);
}

// Résolution du fluide pour les 6 cas SIMULTANÉMENT
cnearest::Collection<const cnearest::Vector*> us(u_pointers);
auto f_results = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

// Construction de la Matrice de Résistance Globale 6x6
Eigen::Matrix<double, 6, 6> resistance_matrix;
for (int i = 0; i < 6; ++i) {
// Pour chaque mouvement 'i', on désassemble les forces et on fait la somme (Force + Moment)
auto nodal_forces = m.disassemble_vector_to_force_nodes(f_results[i]);
auto summed_force = o.sum_nodal_forces(nodal_forces, nearest_neighbour_results);
// Remplissage de la colonne 'i'
resistance_matrix(0, i) = summed_force.linear_component.x();
resistance_matrix(1, i) = summed_force.linear_component.y();
resistance_matrix(2, i) = summed_force.linear_component.z();
resistance_matrix(3, i) = summed_force.angular_component.x();
resistance_matrix(4, i) = summed_force.angular_component.y();
resistance_matrix(5, i) = summed_force.angular_component.z();
}

// ==========================================
// MOBILITY PROBLEM
// ==========================================
// (idem au code Python) : Force X = 6pi, Moment X = 8pi
Eigen::Matrix<double, 6, 1> applied_loads;
applied_loads << 6 * M_PI, 0, 0, 8 * M_PI, 0, 0;

// On résout le système 6x6 (resistance * vitesses = charges) pour trouver les vitesses
Eigen::Matrix<double, 6, 1> velocities = resistance_matrix.colPivHouseholderQr().solve(applied_loads);
std::cout << "Vitesse de Translation trouvee (U) : (" << velocities(0) << ", " << velocities(1) << ", " << velocities(2) << ")" << std::endl;
std::cout << "Vitesse de Rotation trouvee (W) : (" << velocities(3) << ", " << velocities(4) << ", " << velocities(5) << ")" << std::endl;
return 0;
}


#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"

int main(int argc, char *argv[]) {
    double radius = 1.5;
    int force_resolution = 2;
    int quadrature_resolution = 6;

    // Create discretisation
    cnearest::UniformSphericalDiscretisation force_discretisation(
        radius, force_resolution);
    cnearest::UniformSphericalDiscretisation quadrature_discretisation(
        radius, quadrature_resolution);

    // Create rigid body
    cnearest::RigidBody body(force_discretisation, quadrature_discretisation);
}

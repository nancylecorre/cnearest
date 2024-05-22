#pragma once
#include "definitions.h"
#include "force_node.h"
#include "quadrature_node.h"

namespace cnearest {

    /// <summary>
    /// Equations needed to calculate the regular stokeslet.
    /// Note that these calculations conform to the equations in the paper(s)
    /// Cortez et al, Phys.Fluids 17,031504 (2005)
    /// Ref: j.jcp2017.12.008, chapter 3.
    /// They have NOT yet been scaled by 1/8pi  !
    /// </summary>
    class RegularisedStokeslet {
       public:
        /// <summary>
        /// Green's function for the stokeslet flow
        /// S^{\epsilon}_{ij}
        /// Ref: eqn 10.b , Cortez et al, Phys.Fluids 17,031504 (2005)
        /// </summary>
        static Matrix3 stokeslet_tensor(const ForceNode& force_node,
                                        const QuadratureNode& quadrature_node,
                                        double epsilon);

        /// <summary>
        /// Pressure vector
        /// P^{\epsilon}_{j}
        /// Ref: eqn 10.a , Cortez et al, Phys.Fluids 17,031504 (2005)
        /// </summary>
        static Vector3 pressure(const ForceNode& force_node,
                                       const QuadratureNode& quadrature_node,
                                       double epsilon);

        /// <summary>
        /// Unnamed stress tensor precursor
        /// T^{\epsilon}_{ijk}
        /// Ref: eqn 10.c , Cortez et al, Phys.Fluids 17,031504 (2005)
        /// </summary>
        static std::array<Matrix3, 3> stress_tensor(const ForceNode& force_node,
                                                    const QuadratureNode& quadrature_node,
                                                    double epsilon);

       private:
        /// <summary>
        /// Green's function for the stokeslet flow
        /// S^{\epsilon}_{ij}
        /// Ref: eqn 10.b , Cortez et al, Phys.Fluids 17,031504 (2005)
        static Matrix3 stokeslet_tensor(const Vector3& offset, double epsilon);

        /// <summary>
        /// Pressure vector
        /// P^{\epsilon}_{j}
        /// Ref: eqn 10.a , Cortez et al, Phys.Fluids 17,031504 (2005)
        /// </summary>
        static Vector3 pressure(const Vector3& offset, double epsilon);

        /// <summary>
        /// Unnamed stress tensor precursor
        /// T^{\epsilon}_{ijk}
        /// Ref: eqn 10.c , Cortez et al, Phys.Fluids 17,031504 (2005)
        /// </summary>
        static std::array<Matrix3, 3> stress_tensor(const Vector3& offset,
                                                    double epsilon);
    };
}

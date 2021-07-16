#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H

#include "biorbdConfig.h"
#include "Muscles/StaticOptimizationIpopt.h"

namespace biorbd
{
namespace utils
{
class Matrix;
}

namespace muscles
{
///
/// \brief The actual implementation of the Static Optimization problem using a linearized approach
///
/// This algo should be much faster than tradition but less precise (Michaud, 2020)
///
///
class BIORBD_API StaticOptimizationIpoptLinearized : public
    biorbd::muscles::StaticOptimizationIpopt
{
public:
    ///
    /// \brief Construct an Ipopt static optimization problem
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target
    /// \param activationInit The initial activation
    /// \param useResidual If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param pNormFactor The p-norm to perform
    /// \param verbose Level of IPOPT verbose you want
    /// \param eps The precision to perform the finite diffentiation
    ///
    StaticOptimizationIpoptLinearized(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot,
        const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
        const biorbd::utils::Vector& activationInit,
        bool useResidual = true,
        unsigned int pNormFactor = 2,
        int verbose = 0,
        double eps = 1e-10);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~StaticOptimizationIpoptLinearized();

    ///
    /// \brief Method to return the constraint residuals
    /// \param n The number of variables
    /// \param x The values of the variables
    /// \param new_x If the variables were modified by IPOPT
    /// \param m The number of constraints
    /// \param g The actual contraints residual
    /// \return Return the presence of that function
    ///
    virtual bool eval_g(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Number* g);


    ///
    /// \brief Return the jacobian of contraints
    /// \param n The number of variables
    /// \param x The values of the variables
    /// \param new_x If the variables were modified by IPOPT
    /// \param m The number of constraints
    /// \param nele_jac Number of elements in the jacobian matrix
    /// \param iRow iterator on the rows of the jacobian
    /// \param jCol iterator on the columns of the jacobian
    /// \param values The actual constraint jacobian
    /// \return Return the presence of that function
    ///
    virtual bool eval_jac_g(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index nele_jac,
        Ipopt::Index* iRow,
        Ipopt::Index* jCol,
        Ipopt::Number* values);

protected:
    std::shared_ptr<biorbd::utils::Matrix> m_jacobian; ///< The constraints jacobian
    void prepareJacobian(); ///< Setup the constant constraints jacobian

};

}
}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H

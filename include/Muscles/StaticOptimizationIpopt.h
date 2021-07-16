#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_H

#include <memory>
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"

namespace biorbd
{
class Model;

namespace utils
{
class Vector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace muscles
{
class State;
///
/// \brief The actual implementation of the Static Optimization problem
///
class BIORBD_API StaticOptimizationIpopt : public Ipopt::TNLP
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
    StaticOptimizationIpopt(
        biorbd::Model &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedTorque &torqueTarget,
        const biorbd::utils::Vector &activationInit,
        bool  useResidual = true,
        unsigned int pNormFactor = 2,
        int verbose = 0,
        double eps = 1e-10);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~StaticOptimizationIpopt();

    ///
    /// \brief Get info about the NLP
    /// \param n Number of variables
    /// \param m Number of constraints
    /// \param nnz_jac_g Dimension of the constraint jacobian
    /// \param nnz_h_lag Dimension of the constraint hessian
    /// \param index_style Formatting of the matrix (C-Style)
    /// \return Return the presence of that function
    ///
    virtual bool get_nlp_info(
        Ipopt::Index& n,
        Ipopt::Index& m,
        Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag,
        IndexStyleEnum& index_style);

    ///
    /// \brief Return the bounds for my problem
    /// \param n Number of variables
    /// \param x_l Lower bounds of the variables
    /// \param x_u Upper bounds of the variables
    /// \param m Number of constraints
    /// \param g_l Lower bounds of the constraints
    /// \param g_u Upper bounds of the constraints
    /// \return Return the presence of that function
    ///
    virtual bool get_bounds_info(
        Ipopt::Index n,
        Ipopt::Number* x_l,
        Ipopt::Number* x_u,
        Ipopt::Index m,
        Ipopt::Number* g_l,
        Ipopt::Number* g_u);

    ///
    /// \brief Return the starting point for the algorithm
    /// \param n Number of variables
    /// \param init_x If variables are initialized. That variable must be true
    /// \param x The initial values for the variables (output)
    /// \param init_z If z parameters are initialized. That variable must be false
    /// \param z_L Lower bound of z (ignored output)
    /// \param z_U Upper bound of z (ignored output)
    /// \param m Number of constraints
    /// \param init_lambda If lambda parameters are initialized. That variable must be false
    /// \param lambda Initial values of lagrange multipliers (ignored output)
    /// \return Return the presence of that function
    ///
    virtual bool get_starting_point(
        Ipopt::Index n,
        bool init_x,
        Ipopt::Number* x,
        bool init_z,
        Ipopt::Number* z_L,
        Ipopt::Number* z_U,
        Ipopt::Index m,
        bool init_lambda,
        Ipopt::Number* lambda);

    // Method to return the objective value
    ///
    /// \brief Compute the objective value from a set of variables
    /// \param n The number of variables
    /// \param x The values of the variables
    /// \param new_x If the variables were modified by IPOPT
    /// \param obj_value The actual value of the objective function
    /// \return Return the presence of that function
    ///
    virtual bool eval_f(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Number& obj_value);

    ///
    /// \brief Return the gradient of the objective
    /// \param n The number of variables
    /// \param x The values of the variables
    /// \param new_x If the variables were modified by IPOPT
    /// \param grad_f The gradient vector of the objective function
    /// \return Return the presence of that function
    ///
    virtual bool eval_grad_f(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Number* grad_f);

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

    ///
    /// \brief Finalize the optimization
    /// \param status The status of the optimization
    /// \param n Number of variables
    /// \param x Optimal solution
    /// \param z_L Optimal lower bound of z (ignored result)
    /// \param z_U Optimal upper bound of z (ignored result)
    /// \param m number of constraints
    /// \param g Residual at solution
    /// \param lambda Lagrange multipliers at solution
    /// \param obj_value Objective function value at solution
    /// \param ip_data Not used results
    /// \param ip_cq Not used results
    ///
    virtual void finalize_solution(
        Ipopt::SolverReturn status,
        Ipopt::Index n,
        const Ipopt::Number* x,
        const Ipopt::Number* z_L,
        const Ipopt::Number* z_U,
        Ipopt::Index m,
        const Ipopt::Number* g,
        const Ipopt::Number* lambda,
        Ipopt::Number obj_value,
        const Ipopt::IpoptData* ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq);

    ///
    /// \brief Return the final solution
    /// \return The final solution
    ///
    biorbd::utils::Vector finalSolution() const;

    ///
    /// \brief Return the final residual
    /// \return The final residual
    ///
    biorbd::utils::Vector finalResidual() const;

protected:
    biorbd::Model& m_model; ///< The model
    std::shared_ptr<unsigned int> m_nbQ; ///< The number of generalized coordinates
    std::shared_ptr<unsigned int>
    m_nbQdot; ///< The number of generalized velocities
    std::shared_ptr<unsigned int> m_nbMus;///< The number of muscles
    std::shared_ptr<unsigned int> m_nbDof; ///< The number of degrees of freedom
    std::shared_ptr<unsigned int> m_nbTorque; ///< The number of torques to match
    std::shared_ptr<unsigned int>
    m_nbTorqueResidual; ///< The number of torque residual
    std::shared_ptr<double> m_eps; ///< Precision of the finite differentiate
    std::shared_ptr<biorbd::utils::Vector> m_activations; ///< The activations
    std::shared_ptr<biorbd::rigidbody::GeneralizedCoordinates>
    m_Q; ///< The generalized coordinates
    std::shared_ptr<biorbd::rigidbody::GeneralizedVelocity>
    m_Qdot; ///< The generalized velocities
    std::shared_ptr<biorbd::rigidbody::GeneralizedTorque>
    m_torqueTarget; ///< The torque to match
    std::shared_ptr<biorbd::utils::Vector>
    m_torqueResidual; ///< The torque residual
    std::shared_ptr<double> m_torquePonderation; ///< The torque ponderation
    std::shared_ptr<std::vector<std::shared_ptr<biorbd::muscles::State>>>
    m_states; ///< The muscle states
    std::shared_ptr<unsigned int> m_pNormFactor; ///< The p-norm factor
    std::shared_ptr<int> m_verbose; ///< Verbose level of IPOPT
    std::shared_ptr<biorbd::utils::Vector> m_finalSolution; ///< The final solution
    std::shared_ptr<biorbd::utils::Vector> m_finalResidual; ///< The final residual

    ///
    /// \brief To dispatch the variables into biorbd format
    /// \param x variables
    ///
    void dispatch(const Ipopt::Number* x);

};

}
}

#endif // BIORBD_MUSCLES_OPTIMIZATION_IPOPT_H

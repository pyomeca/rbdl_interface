#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimization.h"


s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mGenCoord& Qddot,
        const s2mVector& Activ,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_tauTarget(s2mTau(m)),
    m_Activ(Activ),
    m_p(p)
{
    m_tauTarget.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_tauTarget);
    std::cout << "m_Tau\n:" << m_tauTarget << std::endl;
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const s2mVector &Activ,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_tauTarget(tauTarget),
    m_Activ(Activ),
    m_p(p)
{

}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mGenCoord &Qddot,
        const std::vector<s2mMuscleStateActual> &state,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_tauTarget(s2mTau(m)),
    m_state(state),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_p(p)
{
    m_tauTarget.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_tauTarget);
    std::cout << "m_Tau\n:" << m_tauTarget << std::endl;
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_state[i].activation();
    }
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const std::vector<s2mMuscleStateActual> &state,
        const unsigned int p):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_tauTarget(tauTarget),
    m_state(state),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_p(p)
{
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_state[i].activation();
    }
}

int s2mStaticOptimization::optimize(
        bool LinearizedState
        )
{
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp;
    if (LinearizedState){
        std::cout << "*** Linearized optimization !" << std::endl;
        mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_Q, m_Qdot, m_tauTarget, m_Activ);
    }
    else {
        mynlp = new s2mStaticOptimizationIpopt(m_model, m_Q, m_Qdot, m_tauTarget, m_Activ, true, 2);
    }
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-6);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("derivative_test", "first-order");
    app->Options()->SetStringValue("check_derivatives_for_naninf", "yes");
    app->Options()->SetIntegerValue("max_iter", 10000);
    Ipopt::ApplicationReturnStatus status;
   status = app->Initialize();
   if( status != Ipopt::Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return status;
   }

   // Ask Ipopt to solve the problem
   status = app->OptimizeTNLP(mynlp);

   if( status == Ipopt::Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
   }
   else
   {
      std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
   }
   return status;

}

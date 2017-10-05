#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace boost::numeric::odeint;

double b = 0.01;
double dt = 0.5;
double t_max = 10;

struct ActuatorFunctor {
  
  Model* m_model;

  ActuatorFunctor(Model* m) : m_model(m) { };
  void operator() (const double t, const VectorNd& q,
		   const VectorNd& qd, VectorNd& tau) {
    for (int i=0;i<qd.size();i++)
      tau[i] = -1.*b*qd[i];
  };
};

template<typename TauFcn>
struct DynSystem {
  typedef std::vector<VectorNd> State;
  typedef State Deriv;
  typedef double Time;
  
  Model* m_model;
  ConstraintSet* m_cs;
  TauFcn& tauFcn;

  DynSystem( Model* m, ConstraintSet* cs, TauFcn& tFcn )
    : m_model(m), m_cs(cs), tauFcn(tFcn) {};

  void operator() ( const State& x, State& dxdt, const Time t ) {
    VectorNd tau = VectorNd::Zero(m_model->dof_count);
    dxdt[1] = tau; // for intialization
    tauFcn( t, x[0], x[1], tau );
    ForwardDynamicsContactsDirect( *m_model, x[0], x[1], tau, *m_cs, dxdt[1] );
    dxdt[0] = x[1];
  };
};

template<typename TauFcn>
struct DynSystem2 {
  typedef std::vector<double> State;
  typedef State Deriv;
  typedef double Time;
  
  Model* m_model;
  ConstraintSet* m_cs;
  TauFcn& tauFcn;

  DynSystem2( Model* m, ConstraintSet* cs, TauFcn& tFcn )
    : m_model(m), m_cs(cs), tauFcn(tFcn) {};

  void operator() ( const State& x, State& dxdt, const Time t ) {
    VectorNd zero = VectorNd::Zero(m_model->dof_count);
    VectorNd tau = zero, q = zero, qd = zero, qdd = zero;
    int i;
    for ( i=0; i<m_model->dof_count; i++  ) {
      q[i] = x[i];
      qd[i] = x[i+m_model->dof_count];
    }
    tauFcn( t, q, qd, tau );
    ForwardDynamicsContactsDirect( *m_model, q, qd, tau, *m_cs, qdd );
    for ( i=0; i<m_model->dof_count; i++  ) {
      dxdt[i] = qd[i];
      dxdt[i+m_model->dof_count] = qdd[i];
    }
  };
};

void createRobotArm( const double *L, const double *m, const Vector3d& g,
		     Model& robot, ConstraintSet& CS ) {
  
  Joint joint_1 = Joint(JointTypeRevoluteY),
    joint_2 = Joint(JointTypeRevoluteY);
  unsigned int body_1_id, body_2_id;
  Vector3d com1, com2;
  com1 << 0.5*L[0], 0.0, 0.0;
  com2 << 0.5*L[1], 0.0, 0.0;
  Matrix3d I1 = Matrix3dZero,I2 = Matrix3dZero;
  I1(0,0) = 0.0025; I1(1,1) = I1(2,2) = 1.015/12; I1 = m[0]*L[0]*L[0]*I1;
  I2(0,0) = 0.0025; I2(1,1) = I2(2,2) = 1.015/12; I2 = m[1]*L[1]*L[1]*I2;
  Body body_1 = Body( m[0], com1, I1 ), body_2 = Body( m[1], com2, I2 );  
  
  // ensamble eslabon 1
  body_1_id = robot.AddBody(         0, Xtrans(Vector3d(0.,0.,0.)), joint_1, body_1, "arm");
  // ensamble eslabon 1
  body_2_id = robot.AddBody( body_1_id, Xtrans(Vector3d(L[0],0.,0.)), joint_2, body_2, "forearm");
  // configurar gravedad
  robot.gravity = g;

  CS.AddConstraint ( body_2_id,	Vector3d (.3,0.,0.), Vector3d (0.,1.,0.) );
  CS.AddConstraint ( body_2_id, Vector3d (.3,0.,0.), Vector3d (0.,0.,1.) );

  CS.Bind( robot );
}

int main( int argc, char* argv[] ) {

  rbdl_check_api_version(RBDL_API_VERSION);

  double L[2] = {0.5, 0.3}, m[2] = {0.5, 0.3};
  Vector3d gravity = Vector3d(0.0, 0.0, -1.0*9.81);

  Model model;
  ConstraintSet CS;

  createRobotArm( L, m, gravity, model, CS );

  VectorNd q, qd;

  q = VectorNd::Zero ( model.q_size );
  qd = q;
  q[0] = 0.0*M_PI/4;
  q[1] = 0.0*M_PI/4;

  double t;

  //  typedef DynSystem<ActuatorFunctor> System;
  typedef DynSystem2<ActuatorFunctor> System;

  typedef System::State State;

  //  typedef runge_kutta4<State> Stepper;   // Stepper
  //  typedef runge_kutta_cash_karp54<State> Stepper; // Error Stepper
  //  typedef runge_kutta_dopri5<State> Stepper; // Error Stepper
  //  typedef runge_kutta_fehlberg78<State> Stepper;  // Error Stepper
  //  typedef controlled_runge_kutta< runge_kutta_cash_karp54< State > > Stepper;// Controlled Stepper
  //  typedef runge_kutta_cash_karp54<State> Error_Stepper; // Error Stepper
  //  typedef controlled_runge_kutta< runge_kutta_cash_karp54< State > > Stepper;
  typedef runge_kutta_cash_karp54<State> Error_Stepper; // Error Stepper
  typedef controlled_runge_kutta< runge_kutta_cash_karp54< State > > Controlled_Stepper;
  typedef dense_output_runge_kutta< Controlled_Stepper > Stepper;

  //  Stepper stepper;
  Error_Stepper e_stepper;
  //  Stepper stepper = make_controlled<Error_Stepper>( 1.0e-6, 1.0e-6, e_stepper );
  Stepper stepper = make_dense_output< runge_kutta_cash_karp54< State > >( 1.0e-6 , 1.0e-6 )

  std::vector<VectorNd> data;
  ActuatorFunctor actFcn(&model);
  System dynSys( &model, &CS, actFcn );

  //  State x = dynSys.initState(const VectorNd& q, const VectorNd& qd);
  // VectorNd d = getDataFromStep();
  /* 
    State x(2), x_err(2);  
    x[0] = q; x[1] = qd;
    x_err[0] = x_err[1] = 0.0*qd;*/
  State x(2*model.dof_count), x_err(2*model.dof_count);
  for ( int i=0; i<model.dof_count; i++ ) {
    x[i] = q[i];
    x[i+model.dof_count] = qd[i];
  }
  for ( t=0; t<=t_max ; t+=dt ) {
    //    VectorNd d( model.q_size+1+model.q_size );
    //    d << t, x[0], x_err[0];
    VectorNd d( model.q_size+1 );
    d[0] = t;
    for ( int i=0; i<model.dof_count; i++ )
      d[i+1] = x[i];
    std::cout << d.transpose() << std::endl;
    /* // csv-file
      std::cout << d[0] << ", "
      << d[1]*180/M_PI << ", " << d[2]*180/M_PI << " "
      << d[3]*180/M_PI << ", " << d[4]*180/M_PI << " "<< std::endl;*/
    data.push_back( d );
    //    stepper.do_step( dynSys, x, t, dt );
    //    stepper.do_step( dynSys, x, t, dt, x_err );
    stepper.try_step( dynSys, x, t, dt );
  }

  return 0;
}

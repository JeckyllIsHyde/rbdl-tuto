#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

void createSphere( const double R, const double m, const Vector3d& g,
		   Model& sphere ) {
  
  Joint joint = Joint( JointTypeFloatingBase );
  unsigned int body_id;
  Vector3d com(0.0,0.0,0.0);
  Matrix3d I = Matrix3dZero;
  I(0,0) = I(1,1) = I(2,2) = 2./5*m*R;
  Body body = Body( m, com, I );
  
  // ensamble eslabon 1
  body_id = robot.AddBody( 0, Xtrans(Vector3d::Zero), joint, body, "fb");
  // configurar gravedad
  robot.gravity = g;
}

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

int main( int argc, char* argv[] ) {

  rbdl_check_api_version(RBDL_API_VERSION);

  double R = 0.1, m = 1.;
  Vector3d gravity = Vector3d(0.0, 0.0, -1.0*9.81);

  Model model;

  createRobotArm( R, m, gravity, model );

  VectorNd q, qd;

  q = VectorNd::Zero ( model.q_size );
  qd = q;
  q[0] = 0.0*M_PI/4;
  q[1] = 0.0*M_PI/4;

  double t;

  typedef DynSystem<ActuatorFunctor> System;
  //  typedef DynSystem2<ActuatorFunctor> System;

  typedef System::State State;

  typedef runge_kutta_dopri5<State> Stepper; // Error Stepper

  Stepper stepper;

  std::vector<VectorNd> data;
  ActuatorFunctor actFcn(&model);
  System dynSys( &model, &CS, actFcn );

  //  State x = dynSys.initState(const VectorNd& q, const VectorNd& qd);
  // VectorNd d = getDataFromStep();
    State x(2), x_err(2);  
    x[0] = q; x[1] = qd;
    x_err[0] = x_err[1] = 0.0*qd;
  /* 
     State x(2*model.dof_count), x_err(2*model.dof_count);*/
  for ( int i=0; i<model.dof_count; i++ ) {
    x[i] = q[i];
    x[i+model.dof_count] = qd[i];
  }
  for ( t=0; t<=t_max ; t+=dt ) {
    VectorNd d( model.q_size+1+model.q_size );
    d << t, x[0], x_err[0];
    /*
    VectorNd d( model.q_size+1 );
    d[0] = t;
    for ( int i=0; i<model.dof_count; i++ )
      d[i+1] = x[i];
      std::cout << d.transpose() << std::endl;*/
    /* // csv-file
      std::cout << d[0] << ", "
      << d[1]*180/M_PI << ", " << d[2]*180/M_PI << " "
      << d[3]*180/M_PI << ", " << d[4]*180/M_PI << " "<< std::endl;*/
    data.push_back( d );
    //    stepper.do_step( dynSys, x, t, dt );
    stepper.do_step( dynSys, x, t, dt, x_err );
  }

  return 0;
}

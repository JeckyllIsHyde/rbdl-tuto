#include <iostream>
#include <vector>
//#include <boost/numeric/odeint.hpp>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double b = 0.01;
const double dt = 0.1;
const double t_max = 10;

void createSphere( const double R, const double m, const Vector3d& g,
		   Model& model ) {
  
Joint joint(
	    SpatialVector (0., 0., 0., 1., 0., 0.),
	    SpatialVector (0., 0., 0., 0., 1., 0.),
	    SpatialVector (0., 0., 0., 0., 0., 1.),
	    SpatialVector (0., 1., 0., 0., 0., 0.),
	    SpatialVector (0., 0., 1., 0., 0., 0.),
	    SpatialVector (1., 0., 0., 0., 0., 0.)
	    );
  unsigned int body_id;
  Vector3d com(0.0,0.0,0.0);
  Matrix3d I = Matrix3dZero;
  I(0,0) = I(1,1) = I(2,2) = 2./5*m*R;
  Body body = Body( m, com, I );
  
  // ensamble eslabon 1
  body_id = model.AddBody( 0, Xtrans(Vector3dZero), joint, body, "fb");
  //  body_id = model.SetFloatingBaseBody( body ); // SetFloatingBaseBody is not implemented!!
  // configurar gravedad
  model.gravity = g;
}

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
  //ConstraintSet* m_cs;
  TauFcn& tauFcn;

  DynSystem( Model* m,/* ConstraintSet* cs,*/ TauFcn& tFcn )
    : m_model(m), /*m_cs(cs),*/ tauFcn(tFcn) {};

  void operator() ( const State& x, State& dxdt, const Time t ) {
    VectorNd tau = VectorNd::Zero(m_model->dof_count);
    dxdt[1] = tau; // for intialization
    tauFcn( t, x[0], x[1], tau );
    //    ForwardDynamicsContactsDirect( *m_model, x[0], x[1], tau, *m_cs, dxdt[1] );
    ForwardDynamics( *m_model, x[0], x[1], tau, dxdt[1] );
    dxdt[0] = x[1];
  };
};

int main( int argc, char* argv[] ) {

  rbdl_check_api_version(RBDL_API_VERSION);

  double R = 0.5, m = 1.;
  Vector3d gravity = Vector3d(0.0, 0.0, -1.0*9.81);

  Model model;

  createSphere( R, m, gravity, model );

  VectorNd q, qd, qdd, tau;

  q = VectorNd::Zero ( model.q_size );
  qdd = qd = tau = q;
  q[0] = 0.0;  q[1] = 0.0;  q[2] = 1.0;
  q[3] = 0.0*M_PI/4;  q[4] = 1.0*M_PI/2;  q[5] = 0.0*M_PI/4;

  double t;
  VectorNd d( model.dof_count+1 );
  for ( t=0; t<=0.5/*t_max*/; t+=dt ) {

    if (q[2]-x)
    tau[2] = ;
    
    ForwardDynamics( model, q, qd, tau, qdd );

    q += dt*qd; 
    qd += dt*qdd;
    d << t,q;
    for (int j=0; j<d.size(); j++) std::cout << d[j] << ", "; std::cout << std::endl;
  }
  
  /*
  typedef DynSystem<ActuatorFunctor> System;
  typedef System::State State;
  typedef runge_kutta_dopri5<State> Stepper; // Error Stepper
  Stepper stepper;

  std::vector<VectorNd> data;
  ActuatorFunctor actFcn(&model);*/
  //  System dynSys( &model, /*&CS,*/ actFcn );

  /*
  State x(2), x_err(2);  
  x[0] = q; x[1] = qd;
  x_err[0] = x_err[1] = 0.0*qd;
  for ( t=0; t<=t_max ; t+=dt ) {
    VectorNd d( model.q_size+1+model.q_size );
    d << t, x[0], x_err[0];*/
    /* // csv-file
      std::cout << d[0] << ", "
      << d[1]*180/M_PI << ", " << d[2]*180/M_PI << " "
      << d[3]*180/M_PI << ", " << d[4]*180/M_PI << " "<< std::endl;*/
  /*
    data.push_back( d );
    //    stepper.do_step( dynSys, x, t, dt );
    stepper.do_step( dynSys, x, t, dt, x_err );
    }*/

  return 0;
}

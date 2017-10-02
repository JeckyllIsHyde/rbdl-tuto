/*
  How to do: 
  Introduce springs, constraints, impact and friction. 
  Use of simple DIY ODE step solvers: Euler and RK4.

  1. Introduce a spring as MuscleActuator
  2. Introduce an event for impact with a dense and 
  controlled stepper

 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <rbdl/rbdl.h>
#include <boost/numeric/odeint.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace boost::numeric::odeint;

double b = 0.01;
double t_max = 10.0;
double dt = 0.001;

struct MuscleActuator {
  double k, Ln;
  Model *m_model;
  Vector3d p1, p2;
  unsigned int body_1_id, body_2_id;
  MuscleActuator( Model* m, Vector3d& P1, Vector3d& P2,
		  unsigned int b_1_id, unsigned int b_2_id )
    : m_model(m), p1(P1), p2(P2), body_1_id(b_1_id), body_2_id(b_2_id)
  { k=50; Ln=0.1; };
  void operator() ( const double t, const VectorNd& q,
		    const VectorNd& qd, VectorNd& tau ) {
    UpdateKinematicsCustom(*m_model,&q,NULL,NULL);
    Vector3d p10 = m_model->X_base[1].E.transpose()*p1+m_model->X_base[1].r;
    Vector3d p20 = m_model->X_base[2].E.transpose()*p2+m_model->X_base[2].r;
    Vector3d d12 = p20-p10; double id12i = d12.norm();
    Vector3d f12 = -k*d12/id12i*(id12i-Ln); // force over 2 by 1
    
    // Vector3d f21i = -m_model->X_base[1].E*f12;
    // fext[1] << VectorCrossMatrix(p1)*f21i, f21i;
    Vector3d f12i = m_model->X_base[2].E*f12;
    // fext[2] << VectorCrossMatrix(p2)*f12i, f12i;
    // tau[0] += 0*fext[1][1];
    tau[0] = -1.*b*qd[0];
    tau[1] = -1.*b*qd[1]+1.*(VectorCrossMatrix(p2)*f12i)[1];// fext[2][1];
  };
};

void tauFcn( const double t, const VectorNd& q,
	     const VectorNd& qd, VectorNd& tau ) {
  for (int i=0;i<qd.size();i++)
    tau[i] = -1.*b*qd[i];
}

template<typename TauFcn>
void EulerCstep( Model& model, double t, double dt,
		 VectorNd& q, VectorNd& qd,
		 VectorNd& tau, ConstraintSet& CS, TauFcn& tauFcn ) {
  VectorNd qdd = VectorNd::Zero ( model.q_size );
  tauFcn( t, q, qd, tau );
  ForwardDynamicsContactsDirect( model, q, qd, tau, CS, qdd );
  q += qd*dt;
  qd += qdd*dt;
}

template<typename TauFcn>
void RK4Cstep( Model& model, double t, double dt,
	       VectorNd& q, VectorNd& qd,
	       VectorNd& tau, ConstraintSet& CS, TauFcn& tauFcn ) {
  VectorNd qdk1 = VectorNd::Zero ( model.q_size ),
    qdk2 = qdk1, qdk3 = qdk1, qdk4 = qdk1, qddk1 = qdk1,
    qddk2 = qdk1, qddk3 = qdk1, qddk4 = qdk1, qdtmp;

  // calculate k1
  tauFcn(t, q, qd, tau);
  qdk1 = qd;
  ForwardDynamicsContactsDirect ( model, q, qdk1, tau, CS, qddk1 );
  // calculate k2
  qdtmp = qd+0.5*qddk1*dt;
  tauFcn(t+0.5*dt, q/*+0.5*qdk1*dt*/, qdtmp, tau);
  qdk2 = qdtmp;
  ForwardDynamicsContactsDirect ( model, q+0.5*qdk1*dt, qdk2, tau, CS, qddk2 );
  // calculate k3
  qdtmp = qd+0.5*qddk2*dt;
  tauFcn(t+0.5*dt, q/*+0.5*qdk2*dt*/, qdtmp, tau);
  qdk3 = qdtmp;
  ForwardDynamicsContactsDirect ( model, q+0.5*qdk2*dt, qdk3, tau, CS, qddk3 );
  // calculate k4
  qdtmp = qd+qddk3*dt;
  tauFcn(t+dt, q/*+qdk3*dt*/, qdtmp, tau);
  qdk4 = qdtmp;
  ForwardDynamicsContactsDirect ( model, q+qdk3*dt, qdk4, tau, CS, qddk4 );
  // total update
  q += (qdk1+2*qdk2+2*qdk3+qdk4)*dt/6;
  qd += (qddk1+2*qddk2+2*qddk3+qddk4)*dt/6;
}

// controlled error-stepper
struct DynSystem {
  typedef std::vector<VectorNd> State;
  typedef State Deriv;
  typedef double Time;

  Model *m_model;
  ConstraintSet *m_CS;

  DynSystem( Model* m, ConstraintSet* cs )
    : m_model(m), m_CS(cs) {};
  
  void operator() (const State& x, State& dxdt, const Time t) {
    VectorNd tau = VectorNd::Zero(m_model->dof_count);
    dxdt[1] = tau;
    tauFcn(t, x[0], x[1], tau);
    dxdt[0] = x[1];
    ForwardDynamicsContactsDirect ( *m_model, x[0], x[1], tau, *m_CS, dxdt[1] );
  };
};

runge_kutta4<DynSystem::State> stepper;

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

  VectorNd q, qd, tau;

  createRobotArm( L, m, gravity, model, CS );

  Vector3d P1(0.25,0.,0.), P2(0.15,0.,0.);
  MuscleActuator muscle( &model, P1, P2,
			 model.GetBodyId("arm"), model.GetBodyId("forearm") );

  q = VectorNd::Zero ( model.q_size );
  qd = q; tau = q;
  q[0] = 0.0*M_PI/4;
  q[1] = 0.0*M_PI/4;

  double t;

  std::vector<VectorNd> data;

  DynSystem dynSys(&model,&CS);
  std::vector<VectorNd> x(2);
  for ( t=0; t<=t_max ; t+=dt ) {
    VectorNd d( model.q_size+1 );
    d << t,q;
    data.push_back( d );
    //    RK4Cstep( model, t, dt, q, qd, tau, CS, tauFcn ); // works
    //    RK4Cstep( model, t, dt, q, qd, tau, CS, muscle ); // compile
    //    EulerCstep( model, t, dt, q, qd, tau, CS, tauFcn ); // works
    //    EulerCstep( model, t, dt, q, qd, tau, CS, muscle ); // works
    // TODO: integrate stepper.do_step() from Odeint library
    x[0] = q; x[1] = qd;
    stepper.do_step( dynSys, x, t, dt );
    q = x[0]; qd = x[1];
  }

  char filename[] = "ArmWithMuscle.dat";
  std::fstream fs;
  fs.open(filename, std::fstream::out);
  
  fs <<"# data_size(): " << data.size()
     << " d size: " << data[0].size() << std::endl;
  for ( std::vector<VectorNd>::iterator d_it=data.begin();
	d_it != data.end(); d_it++ )
    fs << (*d_it).transpose() << std::endl;
  fs.close();

  char filename_csv[] = "ArmWithMuscle.csv";
  fs.open(filename_csv, std::fstream::out);
  
  for ( std::vector<VectorNd>::iterator d_it=data.begin();
	d_it != data.end(); d_it++ )
    fs << (*d_it)[0] << ", "
       << (*d_it)[1]*180/M_PI << ", "
       << (*d_it)[2]*180/M_PI << std::endl;
  fs.close();

  return 0;
}

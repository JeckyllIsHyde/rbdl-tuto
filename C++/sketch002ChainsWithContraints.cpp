#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

double b = 0.8;
double t_max = 10.0;
double dt = 0.05;

void tauFcn(const double t, const VectorNd& q,
	    const VectorNd& qd, VectorNd& tau) {
  for (int i=0;i<qd.size();i++)
    tau[i] = -b*qd[i];
}

void EulerCstep( Model& model, double t, double dt,
		 VectorNd& q, VectorNd& qd,
		 VectorNd& tau, ConstraintSet& CS ) {
  VectorNd qdd = VectorNd::Zero ( model.q_size );
  tauFcn(t, q, qd, tau);
  ForwardDynamicsContactsDirect( model, q, qd, tau, CS, qdd );
  q += qd*dt;
  qd += qdd*dt;
}

void RK4Cstep( Model& model, double t, double dt,
	      VectorNd& q, VectorNd& qd,
	      VectorNd& tau, ConstraintSet& CS ) {
  VectorNd qdk1 = VectorNd::Zero ( model.q_size ),
    qdk2 = qdk1, qdk3 = qdk1, qdk4 = qdk1, qddk1 = qdk1,
    qddk2 = qdk1, qddk3 = qdk1, qddk4 = qdk1;

  // calculate k1
  tauFcn(t, q, qd, tau);
  qdk1 = qd;
  ForwardDynamicsContactsDirect ( model, q, qdk1, tau, CS, qddk1 );
  // calculate k2
  tauFcn(t+0.5*dt, q/*+0.5*qdk1*dt*/, qd+0.5*qddk1*dt, tau);
  qdk2 = qd+qddk1*dt/2;
  ForwardDynamicsContactsDirect ( model, q+qdk1*dt/2, qdk2, tau, CS, qddk2 );
  // calculate k3
  tauFcn(t+0.5*dt, q/*+0.5*qdk2*dt*/, qd+0.5*qddk2*dt, tau);
  qdk3 = qd+qddk2*dt/2;
  ForwardDynamicsContactsDirect ( model, q+qdk2*dt/2, qdk3, tau, CS, qddk3 );
  // calculate k4
  tauFcn(t+dt, q/*+qdk3*dt*/, qd+qddk3*dt, tau);
  qdk4 = qd+qddk3*dt;
  ForwardDynamicsContactsDirect ( model, q+qdk3*dt, qdk4, tau, CS, qddk4 );
  // total update
  q += (qdk1+2*qdk2+2*qdk3+qdk4)*dt/6;
  qd += (qddk1+2*qddk2+2*qddk3+qddk4)*dt/6;
}

void make_four_bar( Model& model, ConstraintSet& CS ) {
  Joint joint_rot_y ( SpatialVector (0.,1.,0.,0.,0.,0.) );
  Body body ( 1.0, Vector3d (0.5,0.,0.),
	      Matrix3d ( 0.1,0.,0.,
			 0.,0.1,0.,
			 0.,0.,0.1 ) );
  unsigned int body_1_id = model.AddBody ( 0, Xtrans (Vector3d (0.,0.,0.)), joint_rot_y, body );
  unsigned int body_2_id = model.AddBody ( body_1_id, Xtrans (Vector3d (1.,0.,0.)), joint_rot_y, body );
  unsigned int body_3_id = model.AddBody ( body_2_id, Xtrans (Vector3d (1.,0.,0.)), joint_rot_y, body );
  model.gravity = Vector3d ( 0.,0.,-9.81 );

  CS.AddConstraint ( body_3_id,	Vector3d (1.,0.,0.), Vector3d (1.,0.,0.) );
  CS.AddConstraint ( body_3_id, Vector3d (1.,0.,0.), Vector3d (0.,1.,0.) );
  CS.AddConstraint ( body_3_id, Vector3d (1.,0.,0.), Vector3d (0.,0.,1.) );

  CS.Bind( model );
}

void make_crank_and_slider( Model& model, ConstraintSet& CS ) {
  Joint joint_rot_y ( SpatialVector (0.,1.,0.,0.,0.,0.) );
  Body body ( 1.0, Vector3d (0.5,0.,0.),
	      Matrix3d ( 0.1,0.,0.,
			 0.,0.1,0.,
			 0.,0.,0.1 ) );
  unsigned int body_1_id = model.AddBody ( 0, Xtrans (Vector3d (0.,0.,0.)), joint_rot_y, body );
  unsigned int body_2_id = model.AddBody ( body_1_id, Xtrans (Vector3d (1.,0.,0.)), joint_rot_y, body );
  model.gravity = Vector3d ( 0.,0.,-9.81 );

  CS.AddConstraint ( body_2_id,	Vector3d (1.,0.,0.), Vector3d (0.,1.,0.) );
  CS.AddConstraint ( body_2_id, Vector3d (1.,0.,0.), Vector3d (0.,0.,1.) );

  CS.Bind( model );
}

enum available_models_types { FOURBAR, CANDS } e_model_type;

int main( int argc, char* argv[] ) {

  char* filename;

  if ( argc > 2 ) {
    filename = argv[1];
    if ( strcmp(argv[2], "4bars") == 0 )
      e_model_type = FOURBAR;
    else if ( strcmp(argv[2], "cands") == 0 )
      e_model_type = CANDS;
    else {
      std::cerr << "Usage: " << argv[0] << " filename [4bars|cands]" << std::endl;
      exit(1);
    }
  }
  else {
    std::cerr << "Usage: " << argv[0] << " filename [4bars|cands]" << std::endl;
    exit(1);
  }
  
  Model model;
  ConstraintSet CS;

  VectorNd q, qd, tau;

  if (e_model_type==FOURBAR) {
    // Four Bar example
    make_four_bar( model, CS );
    q = VectorNd::Zero ( model.q_size );
    qd = q; tau = q;
    q[0] =-1.0*M_PI/2;
    q[1] = 1.0*M_PI/2;
    q[2] = 0.0*M_PI;
  } else if (e_model_type == CANDS) {
    // Crank and Slider example
    make_crank_and_slider( model, CS );
    q = VectorNd::Zero ( model.q_size );
    qd = q; tau = q;
    q[0] =-1.0*M_PI/4;
    q[1] = 2.0*M_PI/4;
  }

  double t;
    
  std::vector<VectorNd> data;
  for ( t=0; t<=t_max ; t+=dt ) {
    VectorNd d( model.q_size+1 );
    d << t,q;
    data.push_back( d );
    //    EulerCstep( model, t, dt, q, qd, tau, CS );
    RK4Cstep( model, t, dt, q, qd, tau, CS );
  }

  std::fstream fs;
  fs.open(filename, std::fstream::out);
  
  fs <<"# data_size(): " << data.size()
     << " d size: " << data[0].size() << std::endl;
  for ( std::vector<VectorNd>::iterator d_it=data.begin();
	d_it != data.end(); d_it++ )
    fs << (*d_it).transpose() << std::endl;
  fs.close();

  return 0;
}

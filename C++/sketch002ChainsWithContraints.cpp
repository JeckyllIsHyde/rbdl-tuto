#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

double b = 0.8;
double t_max = 10.0;
double dt = 0.001;

void tauFcn(const double t, const VectorNd& q,
	    const VectorNd& qd, VectorNd& tau) {
  tau << -b*qd[0],-b*qd[1],-b*qd[2];
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

int main( int argc, char* argv[] ) {

  char* filename; 

  if ( argc > 1 )
    filename = argv[1];
  else {
    std::cerr << "Usage: " << argv[0] << " filename" << std::endl;
    exit(1);
  }
  
  Model model;
  Joint joint_rot_y ( SpatialVector (0.,1.,0.,0.,0.,0.) );
  Body body ( 1.0, Vector3d (0.5,0.,0.),
	      Matrix3d ( 0.1,0.,0.,
			 0.,0.1,0.,
			 0.,0.,0.1 ) );
  unsigned int body_1_id = model.AddBody ( 0, Xtrans (Vector3d (0.,0.,0.)), joint_rot_y, body );
  unsigned int body_2_id = model.AddBody ( body_1_id, Xtrans (Vector3d (1.,0.,0.)), joint_rot_y, body );
  unsigned int body_3_id = model.AddBody ( body_2_id, Xtrans (Vector3d (1.,0.,0.)), joint_rot_y, body );
  model.gravity = Vector3d ( 0.,0.,-9.81 );

  ConstraintSet CS;

  CS.AddConstraint ( body_3_id,	Vector3d (1.,0.,0.), Vector3d (1.,0.,0.) );
  CS.AddConstraint ( body_3_id, Vector3d (1.,0.,0.), Vector3d (0.,1.,0.) );
  CS.AddConstraint ( body_3_id, Vector3d (1.,0.,0.), Vector3d (0.,0.,1.) );

  CS.Bind( model );

  VectorNd q = VectorNd::Zero ( model.q_size ),
    qd = q, tau = q;
  q[0] =-1.0*M_PI/2;
  q[1] = 1.0*M_PI/2;
  q[2] = 0.0*M_PI;

  double t;
    
  std::vector<VectorNd> data;
  for ( t=0; t<t_max ; t+=dt ) {
    VectorNd d(4);
    d << t,q;
    data.push_back( d );
    EulerCstep( model, t, dt, q, qd, tau, CS );    
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

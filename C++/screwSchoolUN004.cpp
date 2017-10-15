 /*
  How to do: 
  Implement Custom Joints: Euler angles family, spherical, planar and
  floating base.

  From API functions: 
  * jcalc( model, joint_id, q, qd ) updates: 
  model.X_J and model.v_J. For multi-dof joints use model.multdof3S
  * XJ[joint_id] = jcalc_XJ( model, joint_id, q )

 */
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double b = 0.5;
const double dt = 0.01, tmax = 5.0;

void dynSystem( Model& model, VectorNd& q, VectorNd& qd, VectorNd& qdd  ) {
  VectorNd zero = VectorNd::Zero(model.q_size);
  VectorNd tau = zero;
  // std::vector<SpatialVector> f_ext(4,SpatialVectorZero);
  // SpatialVector v13 = ( model.X_base[0+1].inverse().toMatrix()*model.S[0+1]*qd[0] +
  //			model.X_base[1+1].inverse().toMatrix()*model.S[1+1]*qd[1] +
  //			model.X_base[2+1].inverse().toMatrix()*model.S[2+1]*qd[2] );
  // f_ext[3] = -b*v13;
  // calculate forces
  Matrix3d S; S << ( (model.X_lambda[2+1]*model.X_lambda[1+1]).apply(model.S[0+1]) ).segment<3>(0),
		( model.X_lambda[2+1].apply(model.S[1+1]) ).segment<3>(0),
		( model.S[2+1] ).segment<3>(0);
  tau = -b*S.transpose()*S*qd;
  ForwardDynamics( model, q, qd, tau, qdd/*, &f_ext*/ );
}

void stepEuler( double dt, Model& model, VectorNd& q, VectorNd& qd ) {
  VectorNd zero = VectorNd::Zero(model.q_size);
  VectorNd qdd = zero;
  // make step
  dynSystem( model, q, qd, qdd );
  q += dt*qd;
  qd += dt*qdd;
}

void stepRK4( double dt, Model& model, VectorNd& q, VectorNd& qd ) {
  VectorNd zero = VectorNd::Zero(model.q_size);
  VectorNd qdd = zero;
  // make step
  dynSystem( model, q, qd, qdd );
  q += dt*qd;
  qd += dt*qdd;
}

const double Zeta = +0.1786178958448091;
const double Lambda = -0.2123418310626054;
const double Xi = -0.06626458266981849;
void stepOmelyanPEFRL( double dt, Model& model, VectorNd& q, VectorNd& qd ) {
  VectorNd zero = VectorNd::Zero(model.q_size);
  VectorNd qdd = zero;
  q+=qd*(Zeta*dt); // Mueva_r(dt,Zeta);
  dynSystem( model, q, qd, qdd ); // CalculeTodasLasFuerzas();
  qd+=qdd*((1-2*Lambda)/2*dt); // Mueva_v(dt,(1-2*Lambda)/2);
  q+=qd*(Xi*dt); // Mueva_r(dt,Xi);
  dynSystem( model, q, qd, qdd ); // CalculeTodasLasFuerzas();
  qd+=qdd*(Lambda*dt); // Mueva_v(dt,Lambda);
  q+=qd*((1-2*(Xi+Zeta))*dt); // Mueva_r(dt,1-2*(Xi+Zeta));
  dynSystem( model, q, qd, qdd ); // CalculeTodasLasFuerzas();
  qd+=qdd*(Lambda*dt); // Mueva_v(dt,Lambda);
  q+=qd*(Xi*dt); // Mueva_r(dt,Xi);
  dynSystem( model, q, qd, qdd ); // CalculeTodasLasFuerzas();
  qd+=qdd*((1-2*Lambda)/2*dt); // Mueva_v(dt,(1-2*Lambda)/2);
  q+=qd*(Zeta*dt); // Mueva_r(dt,Zeta);
}

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  // Body link 1
  double m = 1.0, R = 0.1;
  Vector3d com = Vector3d(0.5,0.0,0.0);
  Matrix3d I = Matrix3dIdentity; I = 2./5.*m*R*R*I;
  Body null_body = Body( ); null_body.mIsVirtual = true;
  Body link = Body( m, com, I );
  // Joint
  Joint joint_rx = Joint( JointTypeRevoluteX );
  Joint joint_ry = Joint( JointTypeRevoluteY );
  Joint joint_rz = Joint( JointTypeRevoluteZ );
  // Model
  Vector3d g(0.0,0.0,-1.0*10.0);
  Model model0;
  model0.AddBody( 0, Xtrans(Vector3d(0.,0.,0.)), joint_rz, null_body );
  model0.AppendBody( Xtrans(Vector3d(0.,0.,0.)), joint_ry, null_body );
  model0.AppendBody( Xtrans(Vector3d(0.,0.,0.)), joint_rx, link );
  model0.gravity = g;

  VectorNd zero = VectorNd::Zero(model0.q_size);
  VectorNd q0=zero, qd0=zero; 
  q0 << 45.0*M_PI/180, 0.0*M_PI/180, 90.0*M_PI/180;

  VectorNd d(1+model0.q_size+model0.q_size+model0.q_size);
  double t;
  for ( t=0; t<=tmax; t+=dt ) {
    // MODEL 0: Composite revolution ZYX joints for spherical joint
    //    stepEuler( dt, model0, q0, qd0 );
    stepOmelyanPEFRL( dt, model0, q0, qd0 );
    d << t, q0, q0, q0;
    std::cout << d[0] << ", ";    
    for (int j=1;j<d.size();j++)
      std::cout << 180.0/M_PI*d[j] << " ";
    std::cout << std::endl;
  }
  
  return 0;
}

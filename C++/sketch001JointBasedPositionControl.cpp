/*
  How to do: 
  A position and tracking control for a robot2R planar

  1. PD controller
  2. PD + gravity compensation controller
  3. PD + inverse dinamics controller

 */

#include <iostream>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/numeric/odeint.hpp>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace boost::numeric::odeint;

typedef std::vector<double> Estado_type;

const double h_dirac = 1e-2;
inline double dircFcn(double t, double a) {
  return ( std::max(0,(t-a>=0)?1:-1)
	   -std::max(0,(t-(a+h_dirac)>=0)?1:-1) )/h_dirac;
};
inline double stepFcn(double t, double a) { return std::max(0,(t-a>=0)?1:-1); };
inline double rampFcn(double t, double a) { return std::max(0.0,t-a); };
inline double parbFcn(double t, double a) { return rampFcn(t,a)*rampFcn(t,a); };

void qDesiredForRegulationFcn (const double t, VectorNd& qD,
			       VectorNd& qdD, VectorNd& qddD ) {
  double pi = M_PI;

  qD[0] = pi/4*stepFcn(t,0)-pi/4*stepFcn(t,2.5)+pi/4*stepFcn(t,5)-pi/4*stepFcn(t,7.5);
  qD[1] = pi/4*stepFcn(t,0)-pi/4*stepFcn(t,2.5)+pi/4*stepFcn(t,5)+pi/4*stepFcn(t,7.5);
};

void qDesiredForTrackingFcn ( const double t, VectorNd& qD,
			     VectorNd& qdD, VectorNd& qddD ) {
  VectorNd zero = VectorNd::Zero ( 2/*m_model->dof_count*/ );
  qD = qdD = qddD = zero;
  double pi = M_PI;

  qD[0] = pi/4/2.5*rampFcn(t,0)-pi/4/1.25*rampFcn(t,2.5)
    +pi/4/1.25*rampFcn(t,5)-pi/4/1.25*rampFcn(t,7.5);
  qD[1] = pi/4/2.5*rampFcn(t,0)-pi/4/1.25*rampFcn(t,2.5)
    +pi/4/1.25*rampFcn(t,5)-pi/4/1.25*rampFcn(t,7.5);

  qdD[0] = pi/4/2.5*stepFcn(t,0)-pi/4/1.25*stepFcn(t,2.5)
    +pi/4/1.25*stepFcn(t,5)-pi/4/1.25*stepFcn(t,7.5);
  qdD[1] = pi/4/2.5*stepFcn(t,0)-pi/4/1.25*stepFcn(t,2.5)
    +pi/4/1.25*stepFcn(t,5)-pi/4/1.25*stepFcn(t,7.5);

  qddD[0] = pi/4/2.5*dircFcn(t,0)-pi/4/1.25*dircFcn(t,2.5)
    +pi/4/1.25*dircFcn(t,5)-pi/4/1.25*dircFcn(t,7.5);
  qddD[1] = pi/4/2.5*dircFcn(t,0)-pi/4/1.25*dircFcn(t,2.5)
    +pi/4/1.25*dircFcn(t,5)-pi/4/1.25*dircFcn(t,7.5);
};

void xDesiredForRegulationFcn( const double t, VectorNd& xD,
				   VectorNd& xpD, VectorNd& xppD ) {
  Vector3d p1(0.5,0.0,0.3), p2(0.5,0.0,-0.3);
  VectorNd zero = VectorNd::Zero ( 3 );
  xD = xpD = xppD = zero;
  
  double flag = 1.0;
  
  xD = p1*stepFcn(t,0)-flag*p1*stepFcn(t,2.5)
    +flag*p2*stepFcn(t,2.5)-flag*p2*stepFcn(t,5)
    +flag*p1*stepFcn(t,5)-flag*p1*stepFcn(t,7.5)
    +flag*p2*stepFcn(t,7.5);

}

void xDesiredForTrackingFcn( const double t, VectorNd& xD,
			     VectorNd& xpD, VectorNd& xppD ) {
  VectorNd zero = VectorNd::Zero ( 3 );
  xD = xpD = xppD = zero;
  Vector3d pc(0.5,0.0,0.3);
  double r = 0.2, w = 0.25;
  
  xD << r*sin(w*t)+pc[0], pc[1], r*cos(w*t)+pc[2];
  xpD << w*r*cos(w*t), 0, -w*r*sin(w*t);
  xppD << -w*w*r*sin(w*t), 0, -w*w*r*cos(w*t) ;
}

void PDControllerFcn ( Model& model,
		       const VectorNd& q, const VectorNd& qd,
		       const VectorNd& qD, const VectorNd& qdD,const VectorNd& qddD,
		       VectorNd& tau,
		       std::vector<Math::SpatialVector>& f_ext ) {
  double kp = 10.0, kd = 1.0;
  tau[0] = kp*(qD[0]-q[0]) - kd*qd[0];
  tau[1] = kp*(qD[1]-q[1]) - kd*qd[1];
}

void PDWithGCControllerFcn ( Model& model,
			     const VectorNd& q, const VectorNd& qd,
			     const VectorNd& qD, const VectorNd& qdD,const VectorNd& qddD,
			     VectorNd& tau,
			     std::vector<Math::SpatialVector>& f_ext ) {
  double kp = 10.0, kd = 1.0;
  InverseDynamics( model, q, 0*q, 0*q, tau );
  tau = kp*(qD-q) + kd*(-qd) + tau;
}

void PDWithIDControllerFcn ( Model& model,
			     const VectorNd& q, const VectorNd& qd,
			     const VectorNd& qD, const VectorNd& qdD,const VectorNd& qddD,
			     VectorNd& tau,
			     std::vector<Math::SpatialVector>& f_ext ) {
  // double kp = 10.0, kd = 1.0;
  double kp = 39.47, kd = 12.57;
  MatrixNd H = MatrixNd::Zero ( model.dof_count,
				model.dof_count );
  InverseDynamics( model, q, qd, 0*q, tau, &f_ext );
  CompositeRigidBodyAlgorithm( model, q, H, true );
  tau = H*(kp*(qD-q) + kd*(qdD-qd) + qddD) + tau;
}

void TransposeJacobianPControllerFcn ( Model& model,
				       const VectorNd& q, const VectorNd& qd,
				       const VectorNd& xD, const VectorNd& ,const VectorNd& ,
				       VectorNd& tau,
				       std::vector<Math::SpatialVector>& f_ext ) {

  MatrixNd J = MatrixNd::Zero ( 6, model.dof_count ),
    Jp = J, invJp = J;

  Vector3d ph( 0.3,0.0,0.0 );
  VectorNd zero = VectorNd::Zero (model.dof_count);
  double kp = 10.0;

  unsigned int body_2_id = model.GetBodyId("body_2");
  J.setZero();
  CalcBodySpatialJacobian( model, q, body_2_id, J, true );
  J = model.X_base[2].inverse().toMatrix()*J;
  Vector3d p = CalcBodyToBaseCoordinates( model, q, body_2_id, ph, false ),
    fu = kp*( xD-p );
  SpatialVector f;
  f << VectorCrossMatrix(p)*fu, fu;
  tau = J.transpose()*f;
}

void InverseJacobianPDControllerFcn ( Model& model,
				      const VectorNd& q, const VectorNd& qd,
				      const VectorNd& xD, const VectorNd& ,const VectorNd& ,
				      VectorNd& tau,
				      std::vector<Math::SpatialVector>& f_ext ) {

  MatrixNd J = MatrixNd::Zero ( 6, model.dof_count ),
    Jp = J, invJp = J;

  Vector3d ph( 0.3,0.0,0.0 );
  VectorNd zero = VectorNd::Zero (model.dof_count);
  double kp = 10.0, kd = 0.8;
  //  double T = 1, kp = 4*M_PI*M_PI/T, kd = 2*sqrt(kp);

  unsigned int body_2_id = model.GetBodyId("body_2");
  J.setZero();
  CalcBodySpatialJacobian( model, q, body_2_id, J, true );
  J = model.X_base[2].toMatrix().inverse()*J;
  Vector3d p = CalcBodyToBaseCoordinates( model, q, body_2_id, ph, false ),
    x_err = ( xD-p ),
    xd_err = ( Vector3d(0.0,0.0,0.0)-J.block(3,0,3,2)*qd ),
    fu = kp*x_err + kd*xd_err;
  Jp = Xtrans(p).toMatrix()*J;
  invJp = (Jp.transpose()*Jp).inverse()*Jp.transpose();
  SpatialVector f;
  InverseDynamics ( model, q, zero, zero, tau ); 
  // f << VectorCrossMatrix(p)*fu, fu;
  // tau += J.transpose()*f;
  tau += kp*invJp.block(0,3,2,3)*x_err;
}

struct ControllerFunctor {
  Model* m_model;
  void (*desiredTrayFcn)( const double, VectorNd&, VectorNd&, VectorNd& );
  void (*controllerFcn)( Model&,
			 const VectorNd&, const VectorNd&,
			 const VectorNd&, const VectorNd&,const VectorNd&,
			 VectorNd&,
			 std::vector<Math::SpatialVector>& );  
  ControllerFunctor( Model* model,
		     void (*ctrlerFcn)( Model& model, const VectorNd&, const VectorNd&,
				      const VectorNd&, const VectorNd&, const VectorNd&,
				      VectorNd&, std::vector<Math::SpatialVector>& ),
		     void (*dTrayFcn)( const double, VectorNd&, VectorNd&, VectorNd& ) )
    : m_model( model ), controllerFcn( ctrlerFcn ), desiredTrayFcn( dTrayFcn ) { }
  void operator() ( const double t, const VectorNd& q, const VectorNd& qd,
		    std::vector<Math::SpatialVector>& f_ext,
		    VectorNd& tau ) {
    // qDesired, qdDesired, qddDesired, tau, f_ext
    //    VectorNd zero = VectorNd::Zero (m_model->dof_count),
    //  qD = zero, qdD = zero, qddD = zero;
    VectorNd D, dD, ddD;
    // q(t) desired function
    desiredTrayFcn ( t, D, dD, ddD );
    
    // controller function
    controllerFcn( *m_model, q, qd, D, dD, ddD, tau, f_ext );
  };
};

struct DynRobotFunctor {
  Model* m_model;
  VectorNd q, qd, qdd, tau;
  ControllerFunctor controllerFcntr;
  DynRobotFunctor( Model* model,
		   void (*ctrlerFcn)( Model& model, const VectorNd&, const VectorNd&,
				      const VectorNd&, const VectorNd&, const VectorNd&,
				      VectorNd&, std::vector<Math::SpatialVector>&),
		   void (*dTrayFcn)( const double, VectorNd&, VectorNd&, VectorNd& ) )
    : m_model( model ), controllerFcntr( model, ctrlerFcn, dTrayFcn ) { 
    q = VectorNd::Zero (model->dof_count);
    qd = VectorNd::Zero (model->dof_count);
    qdd = VectorNd::Zero (model->dof_count);
    tau = VectorNd::Zero (m_model->dof_count);
  }
  void operator() (const Estado_type &x, Estado_type &dxdt, double t ) {
    q = VectorNd::Map(&x[0], m_model->dof_count);
    qd = VectorNd::Map(&x[m_model->dof_count], m_model->dof_count);

    // f_ext(t) desired function
    std::vector<Math::SpatialVector> f_ext(3);
    double b = 0.2;
    f_ext[0] = f_ext[1] = f_ext[2] = SpatialVectorZero;
    f_ext[1][1] = b*(qd[1]-qd[0]);
    f_ext[2][1] = -b*qd[1];

    // controller function
    controllerFcntr( t, q, qd, f_ext, tau );
    
    ForwardDynamics (*m_model, q, qd, tau, qdd, &f_ext );
    dxdt[0] = qd[0]; dxdt[1] = qd[1];
    dxdt[2] = qdd[0]; dxdt[3] = qdd[1];
  }
};

struct ObserverFunctor {
  DynRobotFunctor* dyn_model;
  ObserverFunctor( DynRobotFunctor* dm ) : dyn_model( dm ) { };
  void operator() ( const Estado_type &x, double t ) {
    VectorNd D, dD, ddD;
    dyn_model->controllerFcntr.desiredTrayFcn( t, D, dD, ddD );
    std::cout << t << " "
	      << x[0] << " "
	      << x[1] << " "
	      << x[2] << " "
	      << x[3] << " "
	      << D.transpose() << std::endl;
  };
};

void createRobotArm(const double *L, const double *m, const Vector3d& g,
		    Model& robot) {
  
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
  body_1_id = robot.AddBody(         0, Xtrans(Vector3d(0.,0.,0.)), joint_1, body_1, "body_1");
  // ensamble eslabon 1
  body_2_id = robot.AddBody( body_1_id, Xtrans(Vector3d(L[0],0.,0.)), joint_2, body_2, "body_2");
  // configurar gravedad
  robot.gravity = g;
};

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  double L[2] = {0.5, 0.3}, m[2] = {0.5, 0.3};
  Vector3d gravity = Vector3d(0.0, 0.0, -1.0*9.81);
  
  Model robot2R;
  createRobotArm( L, m, gravity, robot2R);
  

  // Choose: qDesiredForRegulationFcn,  qDesiredForTrackingFcn, xDesiredForRegulationFcn,
  //         xDesiredForTrackingFcn
  // Choose: PDControllerFcn, PDWithGCControllerFcn, PDWithIDControllerFcn,
  //         TransposeJacobianPControllerFcn, InverseJacobianPDControllerFcn
  // DynRobotFunctor dynRobotFnc( &robot2R, PDWithIDControllerFcn, qDesiredForTrackingFcn );
  DynRobotFunctor dynRobotFnc( &robot2R, TransposeJacobianPControllerFcn, xDesiredForRegulationFcn );
  double t = 0.0, t_init = 0.0, t_end = 10.0, dt = 0.01;
  Estado_type x(2*robot2R.dof_count);

  x[0] = x[1] = x[2] = x[3] =0.0;
  x[0] = 0*M_PI/4;
  x[1] = 0*M_PI/2;

  integrate( dynRobotFnc, x, t_init, t_end, dt, ObserverFunctor( &dynRobotFnc ) );

  return 0;
}

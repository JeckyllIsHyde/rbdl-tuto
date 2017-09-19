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

void qDesiredForTrakingFcn (const double t, VectorNd& qD,
			    VectorNd& qdD, VectorNd& qddD ) {
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

struct ControllerFunctor {
  Model* m_model;
  void (*qDesiredFcn)( const double, VectorNd&, VectorNd&, VectorNd& );
  void (*controllerFcn)( Model&,
			 const VectorNd&, const VectorNd&,
			 const VectorNd&, const VectorNd&,const VectorNd&,
			 VectorNd&,
			 std::vector<Math::SpatialVector>& );  
  ControllerFunctor( Model* model,
		     void (*ctrlerFcn)( Model& model, const VectorNd&, const VectorNd&,
				      const VectorNd&, const VectorNd&, const VectorNd&,
				      VectorNd&, std::vector<Math::SpatialVector>& ),
		     void (*qDFcn)( const double, VectorNd&, VectorNd&, VectorNd& ) )
    : m_model( model ), controllerFcn( ctrlerFcn ), qDesiredFcn( qDFcn ) { }
  void operator() ( const double t, const VectorNd& q, const VectorNd& qd,
		    std::vector<Math::SpatialVector>& f_ext,
		    VectorNd& tau ) {
    // qDesired, qdDesired, qddDesired, tau, f_ext
    VectorNd zero = VectorNd::Zero (m_model->dof_count),
      qD = zero, qdD = zero, qddD = zero;
    // q(t) desired function
    qDesiredFcn ( t, qD, qdD, qddD );
    
    // controller function
    controllerFcn( *m_model, q, qd, qD, qdD, qddD, tau, f_ext );
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
		   void (*qDFcn)( const double, VectorNd&, VectorNd&, VectorNd& ) )
    : m_model( model ), controllerFcntr( model, ctrlerFcn, qDFcn ) { 
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
    double b = 0.1;
    f_ext[0] = f_ext[1] = f_ext[2] = SpatialVectorZero;
    f_ext[1][1] = b*(qd[1]-qd[0]);
    f_ext[2][1] = -b*qd[1];

    // controller function
    //    controllerFcn( *m_model, q, qd, qDesired, qdDesired, qddDesired, tau, f_ext );
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
    VectorNd zero = VectorNd::Zero (2),
      qDesired = zero, qdDesired = zero, qddDesired = zero;
    // qDesiredForRegulationFcn( t, qDesired, qdDesired, qddDesired );
    // qDesiredForTrakingFcn( t, qDesired, qdDesired, qddDesired );
    // dyn_model->qDesiredFcn( t, qDesired, qdDesired, qddDesired );
    dyn_model->controllerFcntr.qDesiredFcn( t, qDesired, qdDesired, qddDesired );
    std::cout << t << " "
	      << x[0] << " "
	      << x[1] << " "
	      << x[2] << " "
	      << x[3] << " "
	      << qDesired[0] << " "
	      << qDesired[1] << std::endl;
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
  body_1_id = robot.AddBody(         0, Xtrans(Vector3d(0.,0.,0.)), joint_1, body_1);
  // ensamble eslabon 1
  body_2_id = robot.AddBody( body_1_id, Xtrans(Vector3d(L[0],0.,0.)), joint_2, body_2);
  // configurar gravedad
  robot.gravity = g;
};

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  double L[2] = {0.5, 0.3}, m[2] = {0.5, 0.3};
  Vector3d gravity = Vector3d(0.0, 0.0, -10.0*1.0);

  Model robot2R;
  createRobotArm( L, m, gravity, robot2R);
  

  // Choose: qDesiredForRegulationFcn,  qDesiredForTrakingFcn
  // Choose: PDControllerFcn, PDWithGCControllerFcn, PDWithIDControllerFcn
  DynRobotFunctor dynRobotFnc( &robot2R, PDWithIDControllerFcn, qDesiredForTrakingFcn );
  double t = 0.0, t_init = 0.0, t_end = 10.0, dt = 0.01;
  Estado_type x(2*robot2R.dof_count);

  x[0] = x[1] = x[2] = x[3] =0.0;
  integrate( dynRobotFnc, x, t_init, t_end, dt, ObserverFunctor( &dynRobotFnc ) );

  return 0;
}

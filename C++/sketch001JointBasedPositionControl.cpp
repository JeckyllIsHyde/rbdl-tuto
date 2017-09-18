/*
  How to do: 
  A position control for a robot2R planar

  1. PD controller
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

double stepFcn(double t, double a) { return std::max(0,(t-a>=0)?1:-1); };
double rampFcn(double t, double a) { return std::max(0.0,t-a); };

struct ObserverFunctor {
  void operator() ( const Estado_type &x, double t ) {
    std::cout << t << " "
	      << x[0] << " "
	      << x[1] << " "
	      << x[2] << " "
	      << x[3] << std::endl;
  };
};

void qDesiredForRegulationFcn (const double t, VectorNd& qD ) {
  double pi = M_PI;

  qD[0] = pi/4*stepFcn(t,0)-pi/4*stepFcn(t,2.5)+pi/4*stepFcn(t,5)-pi/4*stepFcn(t,7.5);
  qD[1] = pi/4*stepFcn(t,0)-pi/4*stepFcn(t,2.5)+pi/4*stepFcn(t,5)+pi/4*stepFcn(t,7.5);
};

void qDesiredForTrakingFcn (const double t, VectorNd& qD ) {
  double pi = M_PI;

  qD[0] = pi/4/2.5*rampFcn(t,0)-pi/4/1.25*rampFcn(t,2.5)
    +pi/4/1.25*rampFcn(t,5)-pi/4/1.25*rampFcn(t,7.5);
  qD[1] = pi/4/2.5*rampFcn(t,0)-pi/4/1.25*rampFcn(t,2.5)
    +pi/4/1.25*rampFcn(t,5)-pi/4/1.25*rampFcn(t,7.5);
};

void PDControllerFcn( Model& model,
		      const VectorNd& q, const VectorNd& qd,
		      const VectorNd& qD, VectorNd& tau ) {
  double kp = 10.0, kd = 1.0;
  tau[0] = kp*(qD[0]-q[0]) - kd*qd[0];
  tau[1] = kp*(qD[1]-q[1]) - kd*qd[1];
}

void PDWithGCControllerFcn( Model& model,
			    const VectorNd& q, const VectorNd& qd,
			    const VectorNd& qD, VectorNd& tau ) {
  double kp = 10.0, kd = 1.0;
  InverseDynamics( model, q, 0*q, 0*q, tau );
  tau = kp*(qD-q) + kd*(-qd) + tau;
}

struct DynRobotFunctor {
  Model* m_model;
  VectorNd q, qd, qdd, tau;
  DynRobotFunctor(Model* model) : m_model(model) {
    q = VectorNd::Zero (model->dof_count);
    qd = VectorNd::Zero (model->dof_count);
    qdd = VectorNd::Zero (model->dof_count);
    tau = VectorNd::Zero (m_model->dof_count);
  }
  void operator() (const Estado_type &x, Estado_type &dxdt, double t ) {
    q = VectorNd::Map(&x[0], m_model->dof_count);
    qd = VectorNd::Map(&x[m_model->dof_count], m_model->dof_count);
    VectorNd qDesired(m_model->dof_count);

    double pi = M_PI;

    //
    qDesiredForRegulationFcn ( t, qDesired );
    // qDesiredForTrakingFcn ( t, qDesired );
      
    // controller function
    // PDControllerFcn( *m_model, q, qd, qDesired, tau );
    PDWithGCControllerFcn( *m_model, q, qd, qDesired, tau );
    
    std::vector<Math::SpatialVector> f_ext(3);
    double b = 0.1;
    f_ext[0] = f_ext[1] = f_ext[2] = SpatialVectorZero;
    f_ext[1][1] = b*(qd[1]-qd[0]);
    f_ext[2][1] = -b*qd[1];

    ForwardDynamics (*m_model, q, qd, tau, qdd, &f_ext );
    dxdt[0] = qd[0]; dxdt[1] = qd[1];
    dxdt[2] = qdd[0]; dxdt[3] = qdd[1];
  }
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
  

  DynRobotFunctor dynRobotFnc(&robot2R);
  double t = 0.0, t_init = 0.0, t_end = 10.0, dt = 0.01;
  Estado_type x(2*robot2R.dof_count);

  x[0] = x[1] = x[2] = x[3] =0.0;
  integrate( dynRobotFnc, x, t_init, t_end, dt, ObserverFunctor() );

  return 0;
}

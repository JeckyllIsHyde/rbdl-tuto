/*
  How to do: 
  A position control for a robot2R planar

  
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
  Vector3d gravity = Vector3d(0.0, 0.0, -10.0*0.0);

  Model robot2R;
  createRobotArm( L, m, gravity, robot2R);
  
  return 0;
}

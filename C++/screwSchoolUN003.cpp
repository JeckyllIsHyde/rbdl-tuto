/*
  How to do: 
  6D basic oprations in a Robot2R for its statics.

  Construction from Model API is more compact than URDF file.
  Use InverseDynamics() to compute torques and spatial forces on each link.
  
 */
#include <iostream>
#include <cmath>
#include <vector>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  double L[2] = {1.0, 1.0},
    m[2] = {1.0, 1.0};
  
  Model robot2R;
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
  body_1_id = robot2R.AddBody(         0, Xtrans(Vector3d(0.,0.,0.)), joint_1, body_1);
  // ensamble eslabon 1
  body_2_id = robot2R.AddBody( body_1_id, Xtrans(Vector3d(L[0],0.,0.)), joint_2, body_2);
  // configurar gravedad
  robot2R.gravity = Vector3d(0.0, 0.0, -10.0);

  std::cout << "Overview robot2R:" << std::endl;
  std::cout << GetModelHierarchy(robot2R) << std::endl;
  
  VectorNd q = VectorNd::Zero(robot2R.dof_count);
  VectorNd qd = VectorNd::Zero(robot2R.dof_count);
  VectorNd qdd = VectorNd::Zero(robot2R.dof_count);
  VectorNd tau = VectorNd::Zero(robot2R.dof_count);

  q << 0.0, 0.0;
  // Inverse dynamics for statics
  InverseDynamics( robot2R, q, qd, qdd, tau );
  std::cout << "torques:" << std::endl;
  std::cout << tau.transpose() << std::endl << std::endl;
  std::cout << "internal joint forces:" << std::endl;
  std::cout << "q1:\n" << robot2R.f[1].transpose() << std::endl
	    << "q2:\n" << robot2R.f[2].transpose() << std::endl << std::endl;

  return 0;
}

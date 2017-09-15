/*
  How to do: 
  3D basic oprations
 */
#include <iostream>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main ( int argc, char* arg[] ) {
  rbdl_check_api_version( RBDL_API_VERSION );

  // define a base for 3D motion
  Vector3d i = Vector3d(1.0,0.0,0.0), j = Vector3d(1.0,0.0,0.0), k = Vector3d(1.0,0.0,0.0);
  Matrix3d ix = VectorCrossMatrix(i), jx = VectorCrossMatrix(j), kx = VectorCrossMatrix(k);

  std::cout << "i':\n" << i.transpose() << std::endl;
  std::cout << "j':\n" << j.transpose() << std::endl;
  std::cout << "k':\n" << k.transpose() << std::endl;

  return 0;
}

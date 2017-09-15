/*
  How to do: 
  3D basic oprations <+,->, cross product, cross matrix, rotation matrices, quaternions.
 */
#include <iostream>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main ( int argc, char* arg[] ) {
  rbdl_check_api_version( RBDL_API_VERSION );

  // define a base for 3D motion
  Vector3d i = Vector3d(1.0,0.0,0.0), j = Vector3d(0.0,1.0,0.0), k = Vector3d(0.0,0.0,1.0);
  Matrix3d ix = VectorCrossMatrix(i), jx = VectorCrossMatrix(j), kx = VectorCrossMatrix(k);

  std::cout << "=== 3D Vector Operations, Cross Matrix." << std::endl;
  std::cout << "i':\n" << i.transpose() << std::endl;
  std::cout << "ix:\n" << ix << std::endl;
  std::cout << "j':\n" << j.transpose() << std::endl;
  std::cout << "jx:\n" << jx << std::endl;
  std::cout << "k':\n" << k.transpose() << std::endl;
  std::cout << "kx:\n" << kx << std::endl << std::endl;

  Vector3d v = 3*i+2*j+4*k;
  Matrix3d vx = VectorCrossMatrix(v);

  std::cout << "=== Cross Matrix and cross product" << std::endl;
  std::cout << "v':\n" << v.transpose() << std::endl;
  std::cout << "vx:\n" << vx << std::endl;
  std::cout << "vxi:\n" << vx*i << std::endl << std::endl;

  double th = M_PI/6;
  std::cout << "=== Rotation matrices. th = " << th*180/M_PI  << std::endl;
  std::cout << "rotX(th):\n" << rotx(th) << std::endl;
  std::cout << "rotY(th):\n" << roty(th) << std::endl;
  std::cout << "rotZ(th):\n" << rotz(th) << std::endl << std::endl;

  std::cout << "=== Rotation matrices from arbitrary vector" << std::endl;
  std::cout << "Vector norm. sqrt(v.dot(v)) = " << std::sqrt(v.dot(v)) << std::endl;
  double angle = v.norm();
  std::cout << "Vector norm. v.norm() = " << angle << std::endl;
  v.normalize();
  std::cout << "Normalized vector. v.normalize()' = \n" << v.transpose() << std::endl;
  std::cout << "Normalized angle: "
	    << remainder( angle, M_PI ) << " in [0-pi]" << std::endl << std::endl;

  std::cout << "=== Quaternions from v and angle \n" << v << std::endl;
  Quaternion q = Quaternion::fromAxisAngle(v,angle);
  std::cout << "q':\n" << q.transpose() << std::endl;
  std::cout << "matrix from q:\n" << q.toMatrix() << std::endl;
  std::cout << "q' from matrix:\n"
	    << Quaternion::fromMatrix(q.toMatrix()).transpose()
	    << std::endl << std::endl;

  return 0;
}

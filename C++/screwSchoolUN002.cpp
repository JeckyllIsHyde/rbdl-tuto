/*
  How to do: 
  6D basic oprations in a planar robot with 2dofs. 
  Inverse and direct kinematics in position and velocity in different frames.
 */
#include <iostream>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct RobotModel {
  RobotModel(int dofs, double* q, double* L, Vector3d* r)
    : dof(dofs) {
    for ( int i=0; i<dofs; i++ ) {
      qi[i] = q[i]; li[i] = L[i];
      XJi[i] = Xrotz(qi[i]); XTi[i] = Xtrans(r[i]);
      Xi[i] = XJi[i]*XTi[i];
    }
  };
  int dof;
  double qi[2];
  double li[2];
  SpatialTransform XJi[2];
  SpatialTransform XTi[2];
  SpatialTransform Xi[2];
};

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  int dofs = 2;
  double q[dofs] = {M_PI/6,M_PI/4};
  double L[dofs] = {0.5,0.3};
  Vector3d r[dofs] = {Vector3d(L[0],0.,0.), Vector3d(L[1],0.,0.)};
  
  RobotModel robot( dofs, q, L, r );

  std::cout << "==========================================================" << std::endl;
  std::cout << "        Using Spatial Math Utils "
	    << std::endl << std::endl;


  return 0;
}

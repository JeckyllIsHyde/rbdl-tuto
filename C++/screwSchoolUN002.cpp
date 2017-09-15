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
  RobotModel(int dofs, double* q, double* L, Vector3d* r);
  ~RobotModel();
  void print(void)  const;
  Vector3d position_from_base(const double b_id, const Vector3d& p) const;

  int dof;
  double* qi;
  double* li;
  SpatialTransform* XJi;
  SpatialTransform* XTi;
  SpatialTransform* Xi;
};

RobotModel::RobotModel(int dofs, double* q, double* L, Vector3d* r)
  : dof(dofs) {
  qi = new double[dofs];
  li = new double[dofs];
  XJi = new SpatialTransform[dofs];
  XTi = new SpatialTransform[dofs];
  Xi = new SpatialTransform[dofs];
  for ( int i=0; i<dofs; i++ ) {
    qi[i] = q[i]; li[i] = L[i];
    XJi[i] = Xrotz(qi[i]); XTi[i] = Xtrans(r[i]);
    Xi[i] = XJi[i]*XTi[i];
  }
};

RobotModel::~RobotModel() {
  delete[] qi;
  delete[] li;
  delete[] XJi;
  delete[] XTi;
  delete[] Xi;
};

void RobotModel::print(void)  const {
  int i;
  std::cout << "q: ";
  for ( i=0;i<dof;i++ )
    std::cout << qi[i] << " ";
  std::cout << std::endl;
  std::cout << "li: ";
  for ( i=0;i<dof;i++ )
    std::cout << li[i] << " ";
  std::cout << std::endl;
  std::cout << "==> Joint transformations:\n";
  for ( i=0;i<dof;i++ )
    std::cout << "=> Joint " << i << ":\n" << XJi[i] << "\n";
  std::cout << std::endl;
  std::cout << "==> Link transformations:\n";
  for ( i=0;i<dof;i++ )
    std::cout << "=> Link " << i << ":\n" << XTi[i] << "\n";
  std::cout << std::endl;
  std::cout << "==> Link-to-link transformations:\n";
  for ( i=0;i<dof;i++ )
    std::cout << "=> lambda(" << i << ")-to-" << i << ":\n" << Xi[i] << "\n";
  std::cout << std::endl;
};

Vector3d RobotModel::position_from_base(const double b_id, const Vector3d& p) const {
  SpatialTransform X0i;
  // b_id must be less than dof
  for ( int i=0;i<b_id;i++ )
    X0i = Xi[i]*X0i;
  X0i = X0i.inverse();
  return X0i.E*(p-X0i.r);
}

int main (int argc, char* arg[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  int dofs = 2;
  double q[dofs] = {M_PI/6,M_PI/4};
  double L[dofs] = {0.5,0.3};
  Vector3d r[dofs] = {Vector3d(0.,0.,0.), Vector3d(L[0],0.,0.)};
  
  RobotModel robot( dofs, q, L, r );

  std::cout << "==========================================================" << std::endl;
  std::cout << "        Using Spatial Math Utils "
	    << std::endl << std::endl;

  robot.print();

  Vector3d ph(L[1],0.0,0.0), ph0;
  // 0->First link, 1->Second link
  ph0 = robot.position_from_base( 1, ph );

  std::cout << "ph':\n" << ph.transpose() << std::endl;
  std::cout << "ph0':\n" << ph0.transpose() << std::endl;

  return 0;
}

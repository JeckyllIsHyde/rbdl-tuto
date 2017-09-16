/*
  How to do: 
  6D basic oprations in a planar robot with 2dofs. 
  Inverse and direct kinematics in position and velocity in different frames.

  Two approaches:
  1. Using Math utils form rbdl library (Must understand Featherstone's method).
  2. Using Model, Body, Joint and Kinematic modules from rbdl library.
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
  // 1->First link, 2->Second link
  ph0 = robot.position_from_base( dofs, ph );

  std::cout << "Solving direct kinematic: position" << std::endl;
  std::cout << "ph':\n" << ph.transpose() << std::endl;
  std::cout << "ph0':\n" << ph0.transpose() << std::endl;

  SpatialVector s1(.0,.0,1.,.0,.0,.0), s2=s1;

  MatrixNd Ji(6,2), J0i(6,2);
  Ji << robot.Xi[1].apply(s1), s2;
  J0i << robot.Xi[0].inverse().apply(s1), (robot.Xi[1]*robot.Xi[0]).inverse().apply(s2);  

  std::cout << "Spacial Jacobian of body i:" << std::endl;
  std::cout << "Ji':\n" << Ji.transpose() << std::endl;
  std::cout << "J0i':\n" << J0i.transpose() << std::endl;

  MatrixNd Jphi(6,2), Jph0i(6,2);
  Jphi = Xtrans(ph).toMatrix()*Ji;
  Jph0i = Xtrans(ph0).toMatrix()*J0i;

  std::cout << "Point ph Jacobian of body i:" << std::endl;
  std::cout << "Jphi':\n" << Jphi.transpose() << std::endl;
  std::cout << "Jph0i':\n" << Jph0i.transpose() << std::endl;

  VectorNd qp(2); qp << 0,1;
  SpatialVector v0 = Jph0i*qp;
  std::cout << "Spacial velocity vh of body i at point ph:" << std::endl;
  std::cout << "v0':\n" << v0.transpose() << std::endl;
  
  std::cout << "==========================================================" << std::endl;
  std::cout << "        Using Model Class and Kinematic Functions"
	    << std::endl << std::endl;
  
  // create a robot with Model class in rbdl library.
  Model robot2R;
  // define joints and links
  Joint joint_1 = Joint(JointTypeRevoluteZ), joint_2 = Joint(JointTypeRevoluteZ);
  unsigned int link_1_id, link_2_id;
  Body link_1 = Body(), link_2 = Body();
  // asssemble
  link_1_id = robot2R.AddBody(         0, Xtrans(Vector3d(0.,0.,0.)), joint_1, link_1);
  link_2_id = robot2R.AddBody( link_1_id, Xtrans(Vector3d(L[0],0.,0.)), joint_2, link_2);

  VectorNd Q = VectorNd::Zero( robot2R.dof_count );
  Q << q[0], q[1]; 
  UpdateKinematicsCustom ( robot2R, &Q, NULL, NULL );
  std::cout << "Final efector position ph in global coords:" << std::endl;
  ph0 = CalcBodyToBaseCoordinates( robot2R, Q, link_2_id, ph, false );
  std::cout << "ph0':\n" << ph0.transpose() << std::endl << std::endl;

  Ji.setZero();
  std::cout << "Spatial Jacobian in local coords:" << std::endl;
  CalcBodySpatialJacobian( robot2R, Q, link_2_id, Ji, false );
  std::cout << "Ji':\n" << Ji.transpose() << std::endl;
  std::cout << "Spatial Jacobian in global coords:" << std::endl;
  std::cout << "J0i':\n"
	    << ((robot2R.X_lambda[2]*robot2R.X_lambda[1]*robot2R.X_lambda[0]).
		inverse().toMatrix()*Ji).transpose()
	    << std::endl << std::endl;

  Jph0i.setZero();
  ph = CalcBaseToBodyCoordinates( robot2R, Q, link_2_id, ph0, false );
  std::cout << "Jacobiano 6D del punto ph, Jph0i (global coords):" << std::endl;
  CalcPointJacobian6D( robot2R, Q, link_2_id, ph, Jph0i, false );
  std::cout << "Jph0i':\n" << Jph0i.transpose() << std::endl;

  Jphi.setZero();
  std::cout << "Linear velocity Jacobian at point ph, Jphi(4:6,:):" << std::endl;
  CalcPointJacobian( robot2R, Q, link_2_id, ph, Jphi, false );
  std::cout << "Jphi':\n" << Jphi.block(0,0,3,2).transpose() << std::endl;

  VectorNd QDot = VectorNd::Zero( robot2R.dof_count );
  QDot << 0.0, 1.0;
  UpdateKinematicsCustom ( robot2R, &Q, &QDot, NULL );
  Vector3d v0_3d = CalcPointVelocity( robot2R, Q, QDot, link_2_id, ph, false );
  std::cout << "Linear velocity at point ph (global coords)" << std::endl;
  std::cout << "v0': \n" << v0_3d.transpose() << std::endl;


  return 0;
}

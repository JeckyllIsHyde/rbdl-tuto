#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

const double dt = 0.001;
const double tmax = 5.0;

struct MechTreeSystem {

  typedef std::vector<SpatialVector> ForceContainer;

  Model model;
  VectorNd q, qd, qdd, tau;
  ForceContainer f_ext;

  void initGeneralizedVariables();
  void forwardDynamics();
  void step( double dt );
};

struct PhysicsEngine {
  MechTreeSystem mechSys;
  
  void printData( double t ) {
    std::cout << "q: " << mechSys.q.transpose() << std::endl;
    std::cout << "qd: " << mechSys.qd.transpose() << std::endl;
    std::cout << "qdd: " << mechSys.qdd.transpose() << std::endl;
  }
  void update( double dt ) {
    mechSys.forwardDynamics();
    mechSys.step( dt );
  }
};

void init_engine_with_humanoid( PhysicsEngine& engine ) {

  Model* human = &(engine.mechSys.model);

  // read lua file
  LuaModelReadFromFile( "ModelHuman.lua", human, true );

  // add virtual bodies for any mass==0.0
  for (int id=0; id<human->mBodies.size(); id++)
    if (human->mBodies[id].mMass==0.0)
      human->mBodies[id].mIsVirtual = true;

  // initialize q,qd,qdd,tau and Q
  engine.mechSys.initGeneralizedVariables();
}

int main() {

  PhysicsEngine engine;

  // init engine
  init_engine_with_humanoid( engine );

  // simulate
  double t;
  for ( t=0; t<=tmax; t+=dt ) {
    engine.printData( t );
    engine.update( dt );
  }
  // final state
  engine.printData( t );
  
  return 0;
}

void MechTreeSystem::initGeneralizedVariables() {
  q = VectorNd::Zero (model.q_size);
  qd = VectorNd::Zero (model.qdot_size);
  qdd = VectorNd::Zero (model.qdot_size);
  tau = VectorNd::Zero (model.qdot_size);

  /*
  model.SetQuaternion( model.GetBodyId("pelvis"),
		       Quaternion::fromZYXAngles( Vector3dZero ),
		       q );
  */

  for ( int i; i<model.mBodies.size(); i++ )
    if ( model.mJoints[i].mJointType==JointTypeSpherical )
      model.SetQuaternion( i,
			   Quaternion::fromZYXAngles( Vector3dZero ),
			   q );

  f_ext = ForceContainer( model.mBodies.size(), SpatialVectorZero );
}

void MechTreeSystem::forwardDynamics() {
  ForwardDynamics( model, q, qd, tau, qdd, &f_ext );
}

void MechTreeSystem::step( double dt ) {
  if (model.q_size != model.qdot_size) {
    for (unsigned int i=1;i<model.mJoints.size();i++) {
      unsigned int q_index = model.mJoints[i].q_index;
      if (model.mJoints[i].mJointType == JointTypeSpherical) {
	Matrix3d S = model.multdof3_S[i].block<3,3>(0,0);
	Vector3d w = S*Vector3d(qd[q_index+0],
				qd[q_index+1],
				qd[q_index+2]);
	if ( w.squaredNorm()>0.0001 ) {
	  Quaternion Q = model.GetQuaternion( i, q );
	  Quaternion Qw = Quaternion(w[0],w[1],w[2],0.0);
	  // Quaternion Qw = Quaternion::fromAxisAngle(w.normalized(), dt*w.norm());
	  Q += Quaternion(0.5*dt*Qw)*Q;
	  Q.normalize();
	  model.SetQuaternion(i,Q,q);
	}
      } else if (model.mJoints[i].mJointType == JointTypeTranslationXYZ) {
	q[q_index+0] += dt*qd[q_index+0];
	q[q_index+1] += dt*qd[q_index+1];
	q[q_index+2] += dt*qd[q_index+2];
      } else {
	q[q_index] += dt*qd[q_index];
      }
    }
    qd += dt*qdd;
  } else {
    q += dt*qd;
    qd += dt*qdd;
  }
}

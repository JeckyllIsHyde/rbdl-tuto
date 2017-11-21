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
};

struct PhysicsEngine {
  MechTreeSystem mechSys;
  
  void printData( double t ) {}
  void update( double dt ) {}
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

  std::cout << "q: " << engine.mechSys.q.transpose() << std::endl;
  std::cout << "qd: " << engine.mechSys.qd.transpose() << std::endl;
  std::cout << "qdd: " << engine.mechSys.qdd.transpose() << std::endl;
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

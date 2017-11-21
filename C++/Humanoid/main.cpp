#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

const double dt = 0.001;
const double tmax = 5.0;

struct PhysicsEngine {
  void printData( double t ) {}
  void update( double dt ) {}
};

void init_engine_with_humanoid( PhysicsEngine& engine ) {
  std::cout << "Simulator!!!" << std::endl;
  Model human;

  LuaModelReadFromFile("ModelHuman.lua", &human, true);

  for (int id=0; id<human.mBodies.size(); id++)
    if (human.mBodies[id].mMass==0.0)
      human.mBodies[id].mIsVirtual = true;
  
  std::cout << "human q size: " << human.q_size <<  std::endl;
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

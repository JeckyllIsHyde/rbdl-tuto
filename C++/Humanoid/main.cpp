#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

int main() {

  std::cout << "Simulator!!!" << std::endl;
  Model human;

  LuaModelReadFromFile("ModelHuman.lua", &human, true);

  for (int id=0; id<human.mBodies.size(); id++)
    if (human.mBodies[id].mMass==0.0)
      human.mBodies[id].mIsVirtual = true;
  
  std::cout << "human q size: " << human.q_size <<  std::endl;
  
  return 0;
}

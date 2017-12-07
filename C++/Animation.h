#ifndef ANIMATION_H
#define ANIMATION_H

#include <string> 
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct PhysicsEngine;

enum JointTypeForFile {
  FREEFLAYER = 0,
  ROTYXZ,
  ROTYZ,
  ROTY,
  FIXED
};

std::string addSphereToCsvFile( std::string BODYName );
std::string addSphereToLuaFile( std::string BODYName, Vector3d pos );

std::string addBodyToCsvFile( std::string BODYName, JointTypeForFile jtype );

void makeAnimationFilesForOnlySpheres( PhysicsEngine& engine );  

#endif // ANIMATION_H

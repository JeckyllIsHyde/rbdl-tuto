#ifndef ANIMATION_H
#define ANIMATION_H

#include <string> 
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct PhysicsEngine;

std::string addSphereToCsvFile( std::string BODYName );
std::string addSphereToLuaFile( std::string BODYName, Vector3d pos );
void makeAnimationFilesForOnlySpheres( PhysicsEngine& engine );  

#endif // ANIMATION_H

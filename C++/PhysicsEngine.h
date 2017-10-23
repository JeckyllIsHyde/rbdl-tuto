#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include <rbdl/rbdl.h>
#include "PhysicsEngine.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct Sphere;
struct MechTreeSystem;

struct Sphere {
  Vector3d pos, f, tau;
  double R;
  MechTreeSystem* sys_pt;
  unsigned int b_id;
  Sphere( Vector3d p, double r )
   : pos(p), f(Vector3dZero), tau(Vector3dZero),
    R(r), sys_pt(NULL), b_id(0) {};
  void bind(MechTreeSystem* mts, unsigned int id) {
    sys_pt = mts; b_id = id;
  }
  Vector3d globalPosition();
  Vector3d globalVelocityOnPoint( const Vector3d& p );
  void applyLoad();
  void clearLoad();
};

struct MechTreeSystem {
  Model model;
  ConstraintSet constraintSet;
  VectorNd q, qd, qdd, tau;

  void initGeneralizedCoordinates(); // init GenCoords = {q,qd,qdd and tau}
  void applyGeneralizedCoordinates(); //
  void forwardDynamics();            // calculate FD
  void step( double dt );            // take a step ahead over GenCoords
};

struct PhysicsEngine {
  typedef std::vector<MechTreeSystem*> SystemContainer;
  typedef SystemContainer::iterator SysIterator;
  typedef std::vector<Sphere> SphereContainer;
  typedef SphereContainer::iterator SphereIterator;
  SystemContainer systems;
  SphereContainer spheres;
  SphereContainer walls;

  typedef std::vector< std::vector<Vector3d> > CollisionState;
  typedef std::vector< std::vector<int> > CollisionBoolState;
  CollisionState collisionState;
  CollisionBoolState isCollision;
  
  PhysicsEngine();
  void update( double dt );
  void drawData( double t );
  void loadByJOnI(Sphere& s1, Sphere& s2,
		  Vector3d& L, int& isCol, double dt);
  void initCollisionState();
};

#endif // PHYSICSENGINE_H

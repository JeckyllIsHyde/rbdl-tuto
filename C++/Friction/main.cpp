/*
  Collision and friction for rigid multi-body dynamics

  - Screw-based formulation

  - Global frame coords (simplify for 2D with X-Z)

  Z^
  |
  Yx---> X
*/
#include <iostream>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics::Math;

const double dt = 0.01;
const double tmax = 5.0;

struct OneBody {
  double mMass;
  Matrix3d mInertia;
  SpatialVector v, a, f;
  SpatialTransform X;

  void forwardDynamics( void );
  void step( double dt );
};

struct PhysicsEngine {
  OneBody body;

  void update( double t );
  void printData( double dt );
};

void init_engine_with_rod( PhysicsEngine& engine );

int main() {

  PhysicsEngine engine;

  // init engine
  init_engine_with_rod( engine );

  // simulate
  double t;
  for ( t=0; t<=tmax+dt; t+=dt ) {
    engine.printData( t );
    engine.update( dt );
  }

  return 0;
};

void init_engine_with_rod( PhysicsEngine& engine ) {
  engine.body.mMass = 1.0;
  engine.body.mInertia = Matrix3d(0.0,0.0,0.0,
				  0.0,0.002,0.0,
				  0.0,0.0,0.0);
  engine.body.v=engine.body.a=engine.body.f  = SpatialVectorZero;
  engine.body.X.E = Matrix3dIdentity;
  engine.body.X.r = Vector3dZero;
}

void PhysicsEngine::update( double dt ) {
  body.forwardDynamics();
  body.step( dt );
}

void PhysicsEngine::printData( double t ) {}

void OneBody::forwardDynamics( void ) {
  //  a = FD(q,v,f)
}

void OneBody::step( double dt ) {
  //  q += dt*v;
  v += dt*a;
}

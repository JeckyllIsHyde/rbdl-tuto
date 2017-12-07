/*
  Collision and friction for rigid multi-body dynamics

   - Screw-based formulation
*/
#include <iostream>

const double dt = 0.01;
const double tmax = 5.0;

struct PhysicsEngine {
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

void init_engine_with_rod( PhysicsEngine& engine ) {}

void PhysicsEngine::update( double t ) {}
void PhysicsEngine::printData( double dt ) {}

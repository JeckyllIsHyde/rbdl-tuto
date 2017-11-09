#include <iostream>

#include "Animation.h"
#include "PhysicsEngine.h"

const double dt = 0.0001, tmax = 10.0;

double m = 1.0, R = 0.1;
Vector3d com(0.0,0.0,0.0);
Matrix3d I = (2./5*R*R*m)*Matrix3dIdentity;
Vector3d g(0.0,0.0,-1.0*10.0);
Body body(m,com,I);
Joint joint( JointTypeFloatingBase );
SpatialTransform X;

void add_sphere_to_engine(  PhysicsEngine& engine,
			    double x, double y, double z ) {
  // push one system
  engine.systems.push_back ( new MechTreeSystem );
  engine.systems.back()->model.AddBody( 0, X, joint, body );
  engine.systems.back()->model.gravity = g;
  engine.systems.back()->initGeneralizedCoordinates();
  engine.systems.back()->q[0] = x;
  engine.systems.back()->q[1] = y;
  engine.systems.back()->q[2] = z;
  engine.systems.back()->qd[4] = 1;
  engine.systems.back()->model.
    SetQuaternion( 2,
		   Quaternion::fromZYXAngles(Vector3dZero),
		   engine.systems.back()->q );
  engine.spheres.push_back( Sphere(Vector3d(-0.,0.,0.),R) );
  engine.spheres.back().bind(engine.systems.back(),2);

  engine.systems.back()->applyGeneralizedCoordinates();
}

const int Nx = 5,Ny = 5,Nz = 5;
void init_engine_for_collision_spheres( PhysicsEngine& engine ) {

  double dx = 1.0/Nx, dy = 1.0/Ny, dz = 1.0/Nz;
  for (int nx=0.; nx<Nx; nx++ )
    for (int ny=0.; ny<Ny; ny++ )
      for ( int nz=0.; nz<Nz; nz++ )
	add_sphere_to_engine( engine,
			      (double)(nx*dx+dx/2),
			      (double)(ny*dy+dy/2),
			      (double)(nz*dz+dz/2) );
};

int main() {

  PhysicsEngine engine;

  init_engine_for_collision_spheres( engine );

  engine.initCollisionState();
    
  double t;
  for ( t=0; t<=tmax+dt; t+=dt ) {
    engine.drawData( t );
    engine.update( dt );
  }
  
  makeAnimationFilesForOnlySpheres( engine );

  for ( PhysicsEngine::SysIterator sys_it=engine.systems.begin();
	sys_it<engine.systems.end(); sys_it++ )
    delete *sys_it;
    
  return 0;
}

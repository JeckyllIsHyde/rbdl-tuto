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

const double dt = 0.0025;
const double tmax = 0.5;

struct OneBody {
  double mMass;
  Matrix3d mInertia;
  SpatialVector v, a, f;
  SpatialTransform X;

  double getTheta( void );
  void setTheta( double qy );
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
  engine.body.v = engine.body.a = engine.body.f  = SpatialVectorZero;
  engine.body.X.E = Matrix3dIdentity;
  engine.body.X.r = Vector3dZero;

  // intial states
  double qy = 30.*M_PI/180; // intial angular position
  engine.body.setTheta( qy );
  engine.body.X.r[2] = 1.0; // initial height on z-axis
  engine.body.v[1] = 4.0; // initial angular velocity w0y
  engine.body.v[3] = 0.0; // initial linear velocity v0x
  engine.body.v[5] = 0.0; // initial linear velocity v0z
}

void PhysicsEngine::update( double dt ) {
  engine.body.f  = SpatialVectorZero;
  // fq1= y+l/2*sin(th)
  // fq2= y-l/2*sin(th)
  // if ( fq1>=r )
  //   f=0
  // else
  //   n1 = [ 0, 1, l/2*cos(t)]
  //   d1 = [ 1, 0, l/2*cos(t)]
  //   d2 = [-1, 0, l/2*cos(t)]
  // if ( fq2>=r )
  //   f=0
  // else
  //   n2 = [ 0, 1,-l/2*cos(t)]
  //   d3 = [ 1, 0,-l/2*cos(t)]
  //   d4 = [-1, 0,-l/2*cos(t)]
  body.forwardDynamics();
  body.step( dt );
}

void PhysicsEngine::printData( double t ) {
  const char* separator = ", ";
  std::cout << t << separator
	    << body.getTheta()*180./M_PI << separator
	    << body.X.r[0] << separator
	    << body.X.r[2]// << separator
    //<< body.v[1] << separator
    //<< body.v[3] << separator
    //<< body.v[5]
	    << std::endl;
}

inline double OneBody::getTheta( void ) {
  return atan2( X.E(0,2), X.E(0,0) );
}

inline void OneBody::setTheta( double qy ) {
  X.E(0,0) = X.E(2,2) = cos(qy);
  X.E(2,0) = -sin(qy);
  X.E(0,2) = sin(qy);
}

void OneBody::forwardDynamics( void ) {
  //  a = FD(q,v,f) CoM coincides with origin XYZ
  a[1] = f[1]/mInertia(1,1);
  a[3] = f[3]/mMass;
  a[5] = -10.0 + f[5]/mMass;
}

void OneBody::step( double dt ) {
  //  q += dt*v;
  double qy = getTheta();
  qy += dt*v[1]; // angular y
  setTheta( qy );
  X.r[0] += dt*v[3]; // position x
  X.r[2] += dt*v[5]; // position z
  //  v += dt*a;
  v[1] += dt*a[1]; // angular velocity wy
  v[3] += dt*a[3]; // linear velocity vx
  v[5] += dt*a[5]; // linear velocity vz
}

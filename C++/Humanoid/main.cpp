#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

const double dt = 0.01;
const double tmax = 0.2;

inline Vector3d fromQuaternionToZYXangles( const Quaternion& Q );

struct Sphere;
struct MechTreeSystem;

struct Sphere {
  Vector3d pos, f, tau;
  double R;
  MechTreeSystem* sys_pt;
  unsigned int b_id;

  Sphere( Vector3d p, double r );
  void bind( MechTreeSystem* mts, unsigned int id );
};

struct MechTreeSystem {

  typedef std::vector<SpatialVector> ForceContainer;

  Model model;
  VectorNd q, qd, qdd, tau;
  ForceContainer f_ext;

  void initGeneralizedVariables();
  void applyGeneralizedCoordinates();
  void forwardDynamics();
  void step( double dt );
};

struct PhysicsEngine {

  typedef std::vector<Sphere> SphereContainer;
  typedef SphereContainer::iterator SphereIterator;

  MechTreeSystem mechSys;
  SphereContainer spheres;
  SphereContainer walls;

  PhysicsEngine();
  void printData( double t );
  void update( double dt ) {
    mechSys.forwardDynamics();
    mechSys.step( dt );
  }
};

void init_engine_with_humanoid( PhysicsEngine& engine ) {

  Model* human = &(engine.mechSys.model);

  // read lua file
  LuaModelReadFromFile( "ModelHuman.lua", human/*, true*/ );

  // add virtual bodies for any mass==0.0
  for (int id=0; id<human->mBodies.size(); id++)
    if (human->mBodies[id].mMass==0.0)
      human->mBodies[id].mIsVirtual = true;

  // initialize q,qd,qdd,tau and Q
  engine.mechSys.initGeneralizedVariables();

  // manual dof initialization
  //  engine.mechSys.q[0] = 1.0; // x-axis height
  //  engine.mechSys.q[1] = 1.0; // y-axis height
  engine.mechSys.q[2] = 1.0; // z-axis height

  Vector3d ori( 0.0, 1.0*M_PI/3, 0.0); // body orientation
  if ( engine.mechSys.model.
       mJoints[engine.mechSys.model.
	       GetBodyId( "pelvis" )].mJointType ==
       JointTypeSpherical )
    engine.mechSys.
      model.SetQuaternion( engine.mechSys.model.GetBodyId( "pelvis" ),
			   Quaternion::fromZYXAngles( ori ),
			   engine.mechSys.q );
  else
    engine.mechSys.q.segment<3>(3) = ori;

  engine.mechSys.qd[0] = 1.0; // x-axis linear velocity
  //  engine.mechSys.qd[1] = 1.0; // y-axis linear velocity
  engine.mechSys.qd[2] = 1.0; // z-axis linear velocity
  //  engine.mechSys.qd[3] = 1.0; // z-axis angular velocity
  //  engine.mechSys.qd[4] = 1.0; // y-axis angular velocity
  engine.mechSys.qd[5] = 1.0; // x-axis angular velocity

  // apply initial contitions to model struct
  engine.mechSys.applyGeneralizedCoordinates();
}

int main() {

  PhysicsEngine engine;

  // init engine
  init_engine_with_humanoid( engine );

  // simulate
  double t;
  for ( t=0; t<=tmax+dt; t+=dt ) {
    engine.printData( t );
    engine.update( dt );
  }
  
  return 0;
}

Sphere::Sphere( Vector3d p, double r )
  : pos(p), f(Vector3dZero), tau(Vector3dZero),
    R(r), sys_pt(NULL), b_id(0) { }

void Sphere::bind( MechTreeSystem* mts, unsigned int id ) {
  sys_pt=mts; b_id=id;
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

  for ( int i=0; i<model.mJoints.size(); i++ )
    if ( model.mJoints[i].mJointType==JointTypeSpherical )
      model.SetQuaternion( i,
			   Quaternion::fromZYXAngles( Vector3dZero ),
			   q );

  f_ext = ForceContainer( model.mBodies.size(), SpatialVectorZero );
}

void MechTreeSystem::forwardDynamics() {
  ForwardDynamics( model, q, qd, tau, qdd, &f_ext );
}

void MechTreeSystem::step( double dt ) {
  if (model.q_size != model.qdot_size) {
    for (unsigned int i=1;i<model.mJoints.size();i++) {
      unsigned int q_index = model.mJoints[i].q_index;
      if (model.mJoints[i].mJointType == JointTypeSpherical) {
	Matrix3d S = model.multdof3_S[i].block<3,3>(0,0);
	Vector3d w = S*Vector3d(qd[q_index+0],
				    qd[q_index+1],
				    qd[q_index+2]);
	if ( w.squaredNorm()>0.0001 ) {
	  Quaternion Q = model.GetQuaternion( i, q );
	  Quaternion Qw = Quaternion(w[0],w[1],w[2],0.0);
	  Q += Quaternion(0.5*dt*Qw)*Q;
	  // Quaternion Qw = Quaternion::fromAxisAngle(w.normalized(), dt*w.norm());
	  // Q = Q*Qw;
	  Q.normalize();
	  model.SetQuaternion(i,Q,q);
	}
      } else if ( model.mJoints[i].mDoFCount > 1 ) {
	for ( int idof=0; idof<model.mJoints[i].mDoFCount; idof++ )
	  q[q_index+idof] += dt*qd[q_index+idof];
      } else {
	q[q_index] += dt*qd[q_index];
      }
    }
    qd += dt*qdd;
  } else {
    q += dt*qd;
    qd += dt*qdd;
  }
}

void MechTreeSystem::applyGeneralizedCoordinates() {
  UpdateKinematicsCustom( model,&q,&qd,NULL );
}

inline Vector3d fromQuaternionToZYXangles( const Quaternion& Q ) {
  Matrix3d E = Q.toMatrix();
  double q1 = atan2( -E(0,2), sqrt(E(0,1)*E(0,1)+E(0,0)*E(0,0)) );
  return ( cos(q1)<0.0 )?
    Vector3d( atan2(-E(0,1),-E(0,0)),
	      q1,
	      atan2(-E(1,2),-E(2,2)) ):
    Vector3d( atan2(E(0,1),E(0,0)),
	      q1,
	      atan2(E(1,2),E(2,2)) );
}

PhysicsEngine::PhysicsEngine() {
  // walls
  double L = 2.0, bigR = 1e6;
  double bottom_plane = 0.0, left_plane = -L/2, back_plane = -L/2;
  Sphere bottom(Vector3d(0.,0.,bottom_plane-bigR), bigR); 
  walls.push_back( bottom ); // bottom plane
  Sphere left(Vector3d(left_plane-bigR,0.,0.), bigR); 
  walls.push_back( left ); // left plane
  Sphere back(Vector3d(0.,back_plane-bigR,0.), bigR);
  walls.push_back( back ); // back plan
  Sphere up(Vector3d(0.,0.,bottom_plane+L+bigR), bigR); 
  walls.push_back( up ); // up plane
  Sphere right(Vector3d(L+left_plane+bigR,0.,0.), bigR); 
  walls.push_back( right ); // right plane
  Sphere front(Vector3d(0.,L+back_plane+bigR,0.), bigR);
  walls.push_back( front ); // front plane
}

void PhysicsEngine::printData( double t ) {
  std::string separator = ", ";
  
  VectorNd data( mechSys.model.qdot_size );
  for ( int i=0; i<mechSys.model.mJoints.size(); i++ ) {
    if ( mechSys.model.mJoints[i].mJointType==JointTypeSpherical ) {
      Quaternion Q = mechSys.model.GetQuaternion( i, mechSys.q );
      Vector3d e_zyx( fromQuaternionToZYXangles(Q) );
      data.segment( mechSys.model.mJoints[i].q_index,
		    mechSys.model.mJoints[i].mDoFCount ) = e_zyx;
    } else if ( mechSys.model.mJoints[i].mDoFCount>1 )
      data.segment( mechSys.model.mJoints[i].q_index,
		    mechSys.model.mJoints[i].mDoFCount ) =
	mechSys.q.segment( mechSys.model.mJoints[i].q_index,
			   mechSys.model.mJoints[i].mDoFCount );
    else
      data[mechSys.model.mJoints[i].q_index] =
	mechSys.q[mechSys.model.mJoints[i].q_index];
  }

  std::cout << t << separator;
  for (int i=0; i<data.size(); i++)
    if (i<3)
      std::cout << data[i] << separator;
    else
      std::cout << 180.0/M_PI*data[i] << separator;
  std::cout << std::endl;
}

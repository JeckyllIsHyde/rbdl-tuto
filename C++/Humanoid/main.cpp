#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

const double dt = 0.0001;
const double tmax = 5.0;

const double K = 1.e4;
const double Gamma = 500;

const double Kcundall = 10, MU = 0.4;
const double ERFF = 1.e-8;

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
  void clearLoad();
  Vector3d globalPosition();
  Vector3d globalVelocityOnPoint( const Vector3d& p );
  void applyLoad( Vector3d p );
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
  inline void set_zero( ForceContainer &spatial_values );
};

struct PhysicsEngine {

  typedef std::vector<Sphere> SphereContainer;
  typedef SphereContainer::iterator SphereIterator;

  MechTreeSystem mechSys;
  SphereContainer spheres;
  SphereContainer walls;

  typedef std::vector< std::vector<Vector3d> > CollisionState;
  typedef std::vector< std::vector<int> > CollisionBoolState;
  CollisionState collisionState;
  CollisionBoolState isCollision;

  PhysicsEngine();
  void printData( double t );
  void update( double dt );
  void loadByJOnI( Sphere& s1, Sphere& s2,
		   Vector3d& L, int& isCol, double dt );
  void initCollisionState();
};

void init_engine_with_humanoid( PhysicsEngine& engine ) {

  Model* human = &(engine.mechSys.model);

  // read lua file
  LuaModelReadFromFile( "ModelHuman.lua", human/*, true*/ );

  // add virtual bodies for any mass==0.0
  for (int id=0; id<human->mBodies.size(); id++)
    if (human->mBodies[id].mMass==0.0)
      human->mBodies[id].mIsVirtual = true;
  /*    else // debugging inertias
      std::cout << human->GetBodyName(id) << ":\n"
		<< human->mBodies[id].mInertia
		<< std::endl;
  */
  
  // initialize q,qd,qdd,tau and Q
  engine.mechSys.initGeneralizedVariables();

  // ids bodies
  unsigned int pelvis_id = human->GetBodyId( "pelvis" );
  unsigned int thigh_r_id = human->GetBodyId( "thigh_r" );
  unsigned int shank_r_id = human->GetBodyId( "shank_r" );
  unsigned int foot_r_id = human->GetBodyId( "foot_r" );
  unsigned int thigh_l_id = human->GetBodyId( "thigh_l" );
  unsigned int shank_l_id = human->GetBodyId( "shank_l" );
  unsigned int foot_l_id = human->GetBodyId( "foot_l" );
  unsigned int middle_trunk_id = human->GetBodyId( "middle_trunk" );
  unsigned int upper_trunk_id = human->GetBodyId( "upper_trunk" );
  unsigned int head_id = human->GetBodyId( "head" );
  unsigned int upper_arm_r_id = human->GetBodyId( "upper_arm_r" );
  unsigned int lower_arm_r_id = human->GetBodyId( "lower_arm_r" );
  unsigned int upper_arm_l_id = human->GetBodyId( "upper_arm_l" );
  unsigned int lower_arm_l_id = human->GetBodyId( "lower_arm_l" );

  // manual dof initialization
  //  engine.mechSys.q[0] = 1.0; // x-axis height
  //  engine.mechSys.q[1] = 1.0; // y-axis height
  engine.mechSys.q[2] = 1.0; // z-axis height

  Vector3d ori( 0.0, 1.0*M_PI/3, 0.0); // body orientation
  if ( human->mJoints[pelvis_id].mJointType ==
       JointTypeSpherical )
    engine.mechSys.
      model.SetQuaternion( pelvis_id,
			   Quaternion::fromZYXAngles( ori ),
			   engine.mechSys.q );
  else
    engine.mechSys.q.segment<3>(3) = ori;

  //engine.mechSys.qd[0] = 0.1; // x-axis linear velocity
  //engine.mechSys.qd[1] = 0.05; // y-axis linear velocity
  //engine.mechSys.qd[2] = 0.1; // z-axis linear velocity
  //engine.mechSys.qd[3] = 1.0; // z-axis angular velocity
  //engine.mechSys.qd[4] = 1.0; // y-axis angular velocity
  //engine.mechSys.qd[5] = 1.0; // x-axis angular velocity

  // create spheres for system collision
  Vector3d com;
  // pelvis
  com = human->mBodies[pelvis_id].mCenterOfMass;
  engine.spheres.push_back( Sphere( com, 0.07) ); // pelvis com
  engine.spheres.back().bind( &(engine.mechSys), pelvis_id );

  // RIGHT LEG
  engine.spheres.push_back( Sphere( Vector3d( 0.0,-0.0872, 0.0 ),
				    0.05) ); // hip_r
  engine.spheres.back().bind( &(engine.mechSys), pelvis_id );
  // thigh_r
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0,-0.4222 ),
				    0.05) ); // knee
  engine.spheres.back().bind( &(engine.mechSys), thigh_r_id );
  // shank_r
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0,-0.4403 ),
				    0.05) ); // ankle
  engine.spheres.back().bind( &(engine.mechSys), shank_r_id );
  // foot_r
  engine.spheres.push_back( Sphere( Vector3d( -0.01, 0.0,-0.06195 ),
				    0.04185) ); // heel
  engine.spheres.back().bind( &(engine.mechSys), foot_r_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.1870,-0.05,-0.0787 ),
				    0.025) ); // meta5
  engine.spheres.back().bind( &(engine.mechSys), foot_r_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.1870, 0.05,-0.0787 ),
				    0.025) ); // hallux
  engine.spheres.back().bind( &(engine.mechSys), foot_r_id );

  // LEFT LEG
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0872, 0.0 ),
				    0.05) ); // hip_l
  engine.spheres.back().bind( &(engine.mechSys), pelvis_id );
  // thigh_l
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0,-0.4222 ),
				    0.05) ); // knee
  engine.spheres.back().bind( &(engine.mechSys), thigh_l_id );
  // shank_l
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0,-0.4403 ),
				    0.05) ); // ankle
  engine.spheres.back().bind( &(engine.mechSys), shank_l_id );
  // foot_l
  engine.spheres.push_back( Sphere( Vector3d( -0.01, 0.0,-0.06195 ),
				    0.04185) ); // heel 
  engine.spheres.back().bind( &(engine.mechSys), foot_l_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.1870, 0.05,-0.0787 ),
				    0.025) ); // meta5
  engine.spheres.back().bind( &(engine.mechSys), foot_l_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.1870,-0.05,-0.0787 ),
				    0.025) ); // hallux
  engine.spheres.back().bind( &(engine.mechSys), foot_l_id );
  // TRUNK
  // middle trunk
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0, 0.1457 ),
				    0.08) ); // spine joint
  engine.spheres.back().bind( &(engine.mechSys), middle_trunk_id );
  // upper trunk
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.0, 0.2155 ),
				    0.1 ) ); //
  engine.spheres.back().bind( &(engine.mechSys), upper_trunk_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.0,-0.1900, 0.2421 ),
				    0.04 ) ); // right shoulder
  engine.spheres.back().bind( &(engine.mechSys), upper_trunk_id );
  engine.spheres.push_back( Sphere( Vector3d( 0.0, 0.1900, 0.2421 ),
				    0.04 ) ); // right shoulder
  engine.spheres.back().bind( &(engine.mechSys), upper_trunk_id );
  // head
  com = human->mBodies[head_id].mCenterOfMass;
  engine.spheres.push_back( Sphere( com, 0.2429/2*0.6 ) ); // head com
  engine.spheres.back().bind( &(engine.mechSys), head_id );

  // RIGHT ARM
  // upper arm
  engine.spheres.push_back( Sphere( Vector3d( 0.0,0.0,-0.2817 ),
				    0.04) ); // elbow_arm_r
  engine.spheres.back().bind( &(engine.mechSys), upper_arm_r_id );
  // lower arm
  engine.spheres.push_back( Sphere( Vector3d( 0.0,0.0,-0.2817 ),
				    0.04) ); // twist_arm_r
  engine.spheres.back().bind( &(engine.mechSys), lower_arm_r_id );

  // LEFT ARM
  // upper arm
  engine.spheres.push_back( Sphere( Vector3d( 0.0,0.0,-0.2817 ),
				    0.04) ); // elbow_arm_r
  engine.spheres.back().bind( &(engine.mechSys), upper_arm_l_id );
  // lower arm
  engine.spheres.push_back( Sphere( Vector3d( 0.0,0.0,-0.2817 ),
				    0.04) ); // twist_arm_r
  engine.spheres.back().bind( &(engine.mechSys), lower_arm_l_id );

  // apply initial contitions to model struct
  engine.mechSys.applyGeneralizedCoordinates();
}

int main() {

  PhysicsEngine engine;

  // init engine
  init_engine_with_humanoid( engine );
  engine.initCollisionState();

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

void Sphere::clearLoad() {
  f=Vector3dZero;
  tau=Vector3dZero;
}

Vector3d Sphere::globalPosition() {
  if (sys_pt==NULL)
    return pos;
  return Vector3d( sys_pt->model.X_base[b_id].E.transpose()*pos
		   +sys_pt->model.X_base[b_id].r );
}

Vector3d Sphere::globalVelocityOnPoint( const Vector3d& p ) {
  if (sys_pt==NULL)
    return Vector3dZero;
  Vector3d lp = sys_pt->model.X_base[b_id].E
    *(p-sys_pt->model.X_base[b_id].r);
  return CalcPointVelocity( sys_pt->model,
			    sys_pt->q, sys_pt->qd,
			    b_id, lp, false );
}

void Sphere::applyLoad( Vector3d p ){
  if (sys_pt==NULL)
    return;
  // apply on f_ext
  SpatialVector f_tmp;
  f_tmp << (VectorCrossMatrix( p )*f+tau),f;
  sys_pt->f_ext[b_id] += f_tmp; 
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

inline void MechTreeSystem::set_zero( ForceContainer &spatial_values ) {
  for (unsigned int i = 0; i < spatial_values.size(); i++)
    spatial_values[i].setZero();
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

void PhysicsEngine::update( double dt ) {
  // 1. reset loads: forces and torques
  mechSys.tau=VectorNd::Zero( mechSys.tau.size() );
  mechSys.set_zero( mechSys.f_ext) ;
  for ( SphereIterator e_it=spheres.begin();
	e_it<spheres.end(); e_it++ )
    e_it->clearLoad();
  for ( SphereIterator e_it=walls.begin();
	e_it<walls.end(); e_it++ )
    e_it->clearLoad();
  // 2. apply loads and in between bodies
  int i,j;
  for( i=0;i<spheres.size();i++ ) {
    for ( j=i+1;j<spheres.size();j++ )
      loadByJOnI( spheres[i],spheres[j],
		  collisionState[i][j],
		  isCollision[i][j], dt );
    for ( j=0;j<walls.size();j++ )
      loadByJOnI( spheres[i],walls[j],
		  collisionState[i][j+spheres.size()],
		  isCollision[i][j+spheres.size()], dt );
  }

  mechSys.forwardDynamics();
  mechSys.step( dt );
}

void PhysicsEngine::initCollisionState() {
  collisionState =
    CollisionState( spheres.size()+walls.size(),
		    std::vector<Vector3d>( spheres.size()+walls.size(),
					   Vector3dZero ) );
  isCollision =
    CollisionBoolState( spheres.size()+walls.size(),
			std::vector<int>( spheres.size()+walls.size(),
					  0 ) );
}

void PhysicsEngine::loadByJOnI( Sphere& s1, Sphere& s2,
				Vector3d& L, int& isCol, double dt ) {
  Vector3d F2, Fn, Ft, r21, n, t, vc, vcn, vct,
    gP2 = s2.globalPosition(),
    gP1 = s1.globalPosition(),
    gPc;
  double d21, s, Fn_n, vcn_n, vct_t, Ft_tmax,
    m12= 0.5*1/*m1*m2/(m1+m2)*/;
  r21= gP2-gP1;
  d21 = r21.norm();
  s = (s1.R+s2.R)-d21;
  if (s>0) {
    /* Debugging contacts
    if ( s1.sys_pt!=NULL || s2.sys_pt!=NULL )
      std::cout << "body collision!!! between: "
		<< ((s1.sys_pt==NULL)?
		    "Walls"
		    :s1.sys_pt->model.GetBodyName(s1.b_id))
		<< " and "
		<< ((s2.sys_pt==NULL)?
		    "Walls"
		    :s2.sys_pt->model.GetBodyName(s2.b_id))
		<< std::endl;
    */    

    // geometria y dinamica del contacto
    n = r21/d21;
    // calcular velocidad de contacto y el vector tangente
    gPc = gP1+r21*(s1.R/d21);
    vc = s2.globalVelocityOnPoint( gPc )
      - s1.globalVelocityOnPoint( gPc );
    vcn_n = vc.dot(n);    vcn = n*vcn_n;
    vct = vc-vcn; vct_t = vct.norm();
    if (vct_t < ERFF)
      t = Vector3dZero;
    else
      t = vct/vct_t;
      
    // fuerzas normales
    // Fn: fuerza de Hertz
    Fn_n = K*pow(s,1.5);
    // disipacion plastica
    Fn_n-= m12*sqrt(s)*Gamma*vcn_n;
    if (Fn_n<0)
      Fn_n = 0.0;
    Fn = n*Fn_n;

    // fuerzas tangenciales
    // fuerza estatica
    L += vct*dt;
    Ft = -Kcundall*L;
    // fuerza cinetica
    Ft_tmax = MU*Fn_n;
    if (Ft.norm()>Ft_tmax)
      Ft = L*(-Ft_tmax/L.norm());

    // construir fuerza total
    F2 = Fn+Ft;
    s2.f = F2;
    s1.f =(-F2);
    s1.applyLoad( gPc );
    s2.applyLoad( gPc );

    isCol=1;
  }
  else if (isCol == 1) {
    L=Vector3dZero; isCol=0;
  }
}

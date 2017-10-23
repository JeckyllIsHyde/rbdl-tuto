#include <iostream>
#include <string>
#include "PhysicsEngine.h"

const double K = 1.e4;
const double Gamma = 50, Kcundall = 10, MU = 0.4;
const double ERFF = 1.e-8;

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
void Sphere::applyLoad() {
  if (sys_pt==NULL)
    return;
  sys_pt->tau[0]+=f[0];
  sys_pt->tau[1]+=f[1];
  sys_pt->tau[2]+=f[2];
  sys_pt->tau[3]+=tau[2];
  sys_pt->tau[4]+=tau[1];
  sys_pt->tau[5]+=tau[0];
}
void Sphere::clearLoad() {
  f=Vector3dZero;
  tau=Vector3dZero;
}

void MechTreeSystem::initGeneralizedCoordinates() {
  q = VectorNd::Zero (model.q_size);
  qd = VectorNd::Zero (model.qdot_size);
  qdd = VectorNd::Zero (model.qdot_size);
  tau = VectorNd::Zero (model.qdot_size);
}
void MechTreeSystem::applyGeneralizedCoordinates() {
  UpdateKinematicsCustom( model,&q,&qd,NULL );
}
void MechTreeSystem::forwardDynamics() {
  if (constraintSet.size() > 0)
    ForwardDynamicsContactsDirect( model, q, qd, tau,constraintSet, qdd );
  else
    ForwardDynamics( model, q, qd, tau, qdd );
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
	  // Quaternion Qw = Quaternion::fromAxisAngle(w.normalized(), dt*w.norm());
	  Q += Quaternion(0.5*dt*Qw)*Q;
	  Q.normalize();
	  model.SetQuaternion(i,Q,q);
	}
      } else if (model.mJoints[i].mJointType == JointTypeTranslationXYZ) {
        q[q_index+0] += dt*qd[q_index+0];
	q[q_index+1] += dt*qd[q_index+1];
	q[q_index+2] += dt*qd[q_index+2];
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

PhysicsEngine::PhysicsEngine() {
  // Paredes
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
void PhysicsEngine::update( double dt ) {
  // 1. Borre Torque: que representan fuerzas
  for ( SysIterator sys_it=systems.begin(); sys_it<systems.end(); sys_it++ ) {
    (*sys_it)->tau = VectorNd::Zero( (*sys_it)->model.qdot_size );
    UpdateKinematicsCustom((*sys_it)->model,&((*sys_it)->q),&((*sys_it)->qd),NULL);
  }
  for ( SphereIterator e_it=spheres.begin(); e_it<spheres.end(); e_it++ )
    e_it->clearLoad();
  for ( SphereIterator e_it=walls.begin(); e_it<walls.end(); e_it++ )
    e_it->clearLoad();
  // 2. Agregue Cargas Entre (cuerpos[i],cuerpos[j]);
  int i,j;
  for( i=0;i<spheres.size();i++ ) {
    for ( j=i+1;j<spheres.size();j++ )
      loadByJOnI( spheres[i],spheres[j],
		  collisionState[i][j],
		  isCollision[i][j], dt );
    for ( j=0;j<walls.size();j++ )
      loadByJOnI( spheres[i],walls[j],
		  collisionState[i][j+spheres.size()-1],
		  isCollision[i][j+spheres.size()-1], dt );
  }
  // calculate forces
  for ( SysIterator sys_it=systems.begin(); sys_it<systems.end(); sys_it++ ) {
    (*sys_it)->forwardDynamics();
    (*sys_it)->step( dt );
  }
}

inline Vector3d fromQuaternionToZYXangles( const Quaternion& Q ) {
  Matrix3d E = Q.toMatrix();
  return Vector3d(std::atan2(E(0,1),E(0,0)),
		  std::asin(-E(0,2)),
		  std::atan2(E(1,2),E(2,2)));
}
void PhysicsEngine::drawData( double t ) {
  std::string separator = ", ";
  std::cout << t;
  for (SysIterator sys_it=systems.begin(); sys_it<systems.end(); sys_it++) {
    Vector3d pos((*sys_it)->q.segment<3>(0)),
      e_zyx( fromQuaternionToZYXangles( (*sys_it)->model.GetQuaternion(2,(*sys_it)->q) ) );
    std::cout << separator << pos[0] << separator << pos[1] << separator << pos[2]
	      << separator << 180.0/M_PI*e_zyx[0]
	      << separator << 180.0/M_PI*e_zyx[1]
	      << separator << 180.0/M_PI*e_zyx[2];
  }
  std::cout << std::endl;
}
void PhysicsEngine::initCollisionState() {
  collisionState = CollisionState( spheres.size()+walls.size(),
				   std::vector<Vector3d>( spheres.size()+walls.size(),
							  Vector3dZero ) );
  isCollision = CollisionBoolState( spheres.size()+walls.size(),
				    std::vector<int>( spheres.size()+walls.size(),
						      0 ) );
}
void PhysicsEngine::loadByJOnI(Sphere& s1, Sphere& s2,
			       Vector3d& L, int& isCol, double dt) {
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
    // geometria y dinamica del contacto
    n = r21/d21;
    // calcular velocidad de contacto y el vector tangente
    gPc = gP1+r21*(s1.R/d21);
    vc = s2.globalVelocityOnPoint( gPc )
      - s1.globalVelocityOnPoint( gPc );
    vcn_n = vc.dot(n);    vcn = n*vcn_n;
    vct = vc-vcn;    vct_t = vct.norm();
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
    if ( Ft.norm()>Ft_tmax )
      Ft = L*(-Ft_tmax/L.norm());

    // construir fuerza total
    F2 = Fn+1*Ft;
    s2.f += F2; s2.tau += VectorCrossMatrix( -n*s2.R )*Ft; 
    s1.f +=(-F2); s1.tau += VectorCrossMatrix( n*s1.R )*(-Ft); 
    s1.applyLoad();
    s2.applyLoad();
    
    isCol = 1;
  }
  else if (isCol == 1) {
    L=Vector3dZero; isCol=0;
  }
}

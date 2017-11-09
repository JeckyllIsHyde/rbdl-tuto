import numpy as np
import rbdl

# Create a new model
m, R  = 1.0, 0.0001
com = np.array([0.5,0.0,0.0])
I = 2./5.*m*R*R*np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
# Create body and virtual bodies
null_body = rbdl.Body()
null_body.mIsVirtual = True
body = rbdl.Body.fromMassComInertia( m, com, I)
xtrans = rbdl.SpatialTransform()
# Create Needed Joints
joint_rz = rbdl.Joint.fromJointType("JointTypeRevoluteZ")
joint_ry = rbdl.Joint.fromJointType("JointTypeRevoluteY")
joint_rx = rbdl.Joint.fromJointType("JointTypeRevoluteX")
joint_e = rbdl.Joint.fromJointType("JointTypeEulerZYX")
joint_s = rbdl.Joint.fromJointType("JointTypeSpherical")
# Create Spherical Joint by individual Revolute Joints q = [thz,thy,thx]
model1 = rbdl.Model()
model1.AddBody(0,xtrans, joint_rz, null_body)
model1.AppendBody(xtrans, joint_ry, null_body)
model1.AppendBody(xtrans, joint_rx, body)
model1.gravity = np.array([0.0,0.0,-1.0*10.0])
# Creaate Spherical Joints by Euler ZYX rotation E0s = Rz*Ry*Rx
model2 = rbdl.Model()
model2.AddBody(0,xtrans, joint_e, body)
model2.gravity = np.array([0.0,0.0,-1.0*10.0])
# Creaate Spherical Joints by Spherical with Quaternions q = [q0,q1,q2,q3]
model3 = rbdl.Model()
model3.AddBody(0,xtrans, joint_s, body)
model3.gravity = np.array([0.0,0.0,-1.0*10.0])

def QfromAxisAngle(v,th):
    d = np.sqrt(np.dot(v,v))
    s2 = np.sin(th*0.5)/d
    return np.hstack((v*s2,np.cos(th*0.5)))
    
def Qmultiply(a,b) :
    return np.array([ 
        b[3] * a[0] + b[0] * a[3] + b[1] * a[2] - b[2] * a[1],
        b[3] * a[1] + b[1] * a[3] + b[2] * a[0] - b[0] * a[2],
        b[3] * a[2] + b[2] * a[3] + b[0] * a[1] - b[1] * a[0],
        b[3] * a[3] - b[0] * a[0] - b[1] * a[1] - b[2] * a[2] ])

def QtoEulerZYX(Q):
    x = Q[0];
    y = Q[1];
    z = Q[2];
    w = Q[3];
    return np.array( [ np.arctan2(2*x*y + 2*w*z,1 - 2*y*y - 2*z*z), # thz
                       np.arcsin(-(2*x*z - 2*w*y)), # thy
                       np.arctan2(2*y*z + 2*w*x,1 - 2*x*x - 2*y*y) ] ) # thx 

import numpy as np
import rbdl

# Create a new model
m, R  = 1.0, 0.001
com = np.array([0.0,0.0,0.0])
I = 2./5.*m*R*R*np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])

model = rbdl.Model()
joint_e = rbdl.Joint.fromJointType("JointTypeEulerZYX")
body = rbdl.Body.fromMassComInertia(m,com,I)
xtrans = rbdl.SpatialTransform()
model.AddBody(0,xtrans, joint_e, body)

print (model)

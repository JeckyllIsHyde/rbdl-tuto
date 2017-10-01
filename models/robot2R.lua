meshes = {
  RotJoint = {
    name = "RotJoint",
    dimensions = { 0.1, 0.1, 0.1},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0},
    geometry = {
      cylinder = {radius=1.,length=1},
    },
  },
  LinkBody1 = {
    name = "LinkBody1",
    dimensions = { 0.05, 0.05, 0.5},
    color = { 0.8, 0.2, 0.8},
    mesh_center = { 0, 0, 0.25},
    geometry = {
      cylinder = {radius=1.,length=1},
    },
  },
  LinkBody2 = {
    name = "LinkBody2",
    dimensions = { 0.05, 0.05, 0.3},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0.15},
    geometry = {
      cylinder = {radius=1.,length=1},
    },   
  },
  EndJoint = {
    name = "EndJoint",
    dimensions = { 0.1, 0.1, 0.1},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, 0},
    geometry = {
      sphere = {radius=1.},
    },   
  },
}

model = {
  configuration = {
    axis_front = { 1, 0, 0 },
    axis_up    = { 0, 0, 1 },
    axis_right = { 0, -1, 0 },
    rotation_order = { 1, 2, 0},
  },

  frames = {
    {
      name = "LINK1",
      parent = "ROOT",
      joint_frame = {
        r = { 0.0, 0.0, 1.0 },
	E = {
          {0., 0., -1.},
          {0., 1., 0.},
          {1., 0., 0.}
        }
      },
      visuals = {
        meshes.LinkBody1,
      },
    },
    {
      name = "JOINTBALL1",
      parent = "LINK1",
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
	E = {
          {1., 0., 0.},
          {0., 0., 1.},
	  {0., -1., 0.}
	}
      },
      visuals = {
        meshes.RotJoint,
      },
    },
    {
      name = "LINK2",
      parent = "LINK1",
      joint_frame = {
        r = { 0.0, 0.0, 0.5 },
	E = {
          {1., 0., 0.},
          {0., 1., 0.},
          {0., 0., 1.}
        },
      },
      visuals = {
        meshes.LinkBody2,
      },
    },
    {
      name = "JOINTBALL2",
      parent = "LINK2",
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
	E = {
	  {1., 0., 0.},
          {0., 0., 1.},
          {0., -1., 0.}
	},
      },
      visuals = {
        meshes.RotJoint,
      },
    },
    {
      name = "ENDJOINT",
      parent = "LINK2",
      joint_frame = {
        r = { 0.0, 0.0, 0.3 },
      },
      visuals = {
        meshes.EndJoint,
      },
    },
  }
}

return model

meshes = {
  SphereJoint = {
    name = "SphereJoint",
    dimensions = { 0.1, 0.1, 0.1},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkBody = {
    name = "LinkBody",
    dimensions = { 0.05, 0.05, 1.0},
    color = { 0.8, 0.2, 0.8},
    mesh_center = { 0, 0, 0.5},
    geometry = {
      cylinder = {radius=1.,length=1},
    },
  },
}

model = {
  configuration = {
    axis_front = { 1, 0, 0 },
    axis_up    = { 0, 0, 1 },
    axis_right = { 0, -1, 0 },
    -- rotation_order = { 0, 1, 2},
  },

  frames = {
    {
      name = "LINK11",
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
        meshes.LinkBody,
      },
    },
    {
      name = "JOINTBALL11",
      parent = "LINK11",
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
    {
      name = "COMLINK11",
      parent = "LINK11",
      joint_frame = {
        r = { 0.0, 0.0, 0.5 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
    {
      name = "LINK12",
      parent = "ROOT",
      joint_frame = {
        r = { 0.0, 1.0, 1.0 },
	E = {
          {0., 0., -1.},
          {0., 1., 0.},
          {1., 0., 0.}
        }
      },
      visuals = {
        meshes.LinkBody,
      },
    },
    {
      name = "JOINTBALL12",
      parent = "LINK12",
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
    {
      name = "COMLINK12",
      parent = "LINK12",
      joint_frame = {
        r = { 0.0, 0.0, 0.5 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
    {
      name = "LINK13",
      parent = "ROOT",
      joint_frame = {
        r = { 0.0, 2.0, 1.0 },
	E = {
          {0., 0., -1.},
          {0., 1., 0.},
          {1., 0., 0.}
        }
      },
      visuals = {
        meshes.LinkBody,
      },
    },
    {
      name = "JOINTBALL13",
      parent = "LINK13",
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
    {
      name = "COMLINK13",
      parent = "LINK13",
      joint_frame = {
        r = { 0.0, 0.0, 0.5 },
      },
      visuals = {
        meshes.SphereJoint,
      },
    },
  }
}

return model

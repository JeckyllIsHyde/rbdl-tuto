meshes = {
  Sphere = {
    name = "Sphere",
    dimensions = { 0.5, 0.5, 0.5},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0},
    geometry = {
      sphere = {radius=1.0},
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
      name = "BODY",
      parent = "ROOT",
      visuals = {
        meshes.Sphere,
      },
    },
  },
}

return model

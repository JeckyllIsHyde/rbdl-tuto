joints = {
  j_spherical = { "JointTypeSpherical" },
  --j_spherical = { "JointTypeEulerZYX" },
}

model = {
  configuration = {
    axis_front = { 1, 0, 0 },
    axis_up    = { 0, 0, 1 },
    axis_right = { 0, -1, 0 },
    rotation_order = { 1, 2, 0},
  },
  gravity = {0.0,0.0,-10.0},
  frames = {
  
    {
      name = "virtual_pelvis",
      parent = "ROOT",
      body = {
        mass = 0.0
      },
      joint = {"JointTypeTranslationXYZ"},
    },
    
    {
      name = "pelvis",
      parent = "virtual_pelvis",
      body = {
        mass = 1.0,
	com = {0.0,0.0,0.0},
	iniertia = {
	  {0.1,0.0,0.0},
	  {0.0,0.1,0.0},
	  {0.0,0.0,0.1},
	},
      },
      joint = joints.j_spherical,
      joint_frame = {
        r = {0.0,0.0,0.0},
	E = {
	  {1.0,0.0,0.0},
	  {0.0,1.0,0.0},
	  {0.0,0.0,1.0},
	},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  geometry = {
	    sphere = { radius=0.1 },
	  }
	},
      },
    },

    {
      name = "thigh_r",
      parent = "pelvis",
      body = {
        mass = 1.0,
	com = {0.0,0.0,-0.2},
	iniertia = {
	  {0.1,0.0,0.0},
	  {0.0,0.1,0.0},
	  {0.0,0.0,0.1},
	},
      },
      joint = joints.j_spherical,
      joint_frame = {
        r = {0.0,-0.1,0.0},
	E = {
	  {1.0,0.0,0.0},
	  {0.0,1.0,0.0},
	  {0.0,0.0,1.0},
	},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = { 0.0,0.0,-0.2 },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {
	  color = { 0.2,0.8,0.2 },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.0,0.0,-0.4 },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
      },
    },

  },
}

return model
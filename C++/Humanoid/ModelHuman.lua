model = {
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
	com = {0.5,0.0,0.0},
	iniertia = {
	  {0.1,0.0,0.0},
	  {0.0,0.1,0.0},
	  {0.0,0.0,0.1},
	},
      },
      joint = {"JointTypeSpherical"},
      joint_frame = {
        r = {0.0,0.0,0.0},
	E = {
	  {1.0,0.0,0.0},
	  {0.0,1.0,0.0},
	  {0.0,0.0,1.0},
	},
      },
    },
  },
}

return model
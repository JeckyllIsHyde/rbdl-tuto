lengths = {
  pelvis = 0.1457,
  thigh = 0.4222,
  shank = 0.4403,
  foot = 0.1037,
  middle_trunk = 0.2155,
  upper_trunk = 0.2421,
  upper_arm = 0.2817,
  lower_arm = 0.2689,
  hand = 0.0862,
  head = 0.2429,
}

masses = {
  pelvis = 0.8154,
  thigh = 10.3368,
  shank = 3.1609,
  foot = 1.001,
  middle_trunk = 16.33,
  upper_trunk = 15.96,
  upper_arm = 1.9783,
  lower_arm = 1.1826,
  hand = 0.4453,
  head = 5.0662,
}

coms = {
  pelvis = { 0., 0.,  0.0891},
  thigh = { 0., 0., -0.1729},
  shank = { 0., 0., -0.1963},
  foot = { 0.1254, 0., -0.0516},
  middle_trunk = { 0., 0.,  0.1185},
  upper_trunk = { 0., 0.,  0.1195},
  upper_arm = { 0., 0., -0.1626},
  lower_arm = { 0., 0., -0.1230},
  hand = { 0., 0., -0.0680},
  head = { 0., 0.,  1.1214}
}

inertias = {
  pelvis = {--- 0.0897, 0.0855, 0.0803
    {0.0897*0.0897*lengths.pelvis*lengths.pelvis*masses.pelvis,0.0,0.0},
    {0.0,0.0855*0.0855*lengths.pelvis*lengths.pelvis*masses.pelvis,0.0},
    {0.0,0.0,0.0803*0.0803*lengths.pelvis*lengths.pelvis*masses.pelvis},
  },
  thigh = {--- 0.1389, 0.0629, 0.1389
    {0.1389*0.1389*lengths.thigh*lengths.thigh*masses.thigh,0.0,0.0},
    {0.0,0.0629*0.0629*lengths.thigh*lengths.thigh*masses.thigh,0.0},
    {0.0,0.0,0.1389*0.1389*lengths.thigh*lengths.thigh*masses.thigh},
  },
  shank = {--- 0.1123, 0.0454, 0.1096
    {0.1123*0.1123*lengths.shank*lengths.shank*masses.shank,0.0,0.0},
    {0.0,0.0454*0.0454*lengths.shank*lengths.shank*masses.shank,0.0},
    {0.0,0.0,0.1096*0.1096*lengths.shank*lengths.shank*masses.shank},
  },
  foot = {--- 0.0267, 0.0129, 0.0254
    {0.0267*0.0267*lengths.foot*lengths.foot*masses.foot,0.0,0.0},
    {0.0,0.0129*0.0129*lengths.foot*lengths.foot*masses.foot,0.0},
    {0.0,0.0,0.0254*0.0254*lengths.foot*lengths.foot*masses.foot},
  },
  middle_trunk = {--- 0.0970, 0.1009, 0.0825
    {0.0970*0.0970*lengths.middle_trunk*lengths.middle_trunk*masses.middle_trunk,0.0,0.0},
    {0.0,0.1009*0.1009*lengths.middle_trunk*lengths.middle_trunk*masses.middle_trunk,0.0},
    {0.0,0.0,0.0825*0.0825*lengths.middle_trunk*lengths.middle_trunk*masses.middle_trunk},
  },
  upper_trunk = {--- 0.1273, 0.1172, 0.0807
    {0.1273*0.1273*lengths.upper_trunk*lengths.upper_trunk*masses.upper_trunk,0.0,0.0},
    {0.0,0.1172*0.1172*lengths.upper_trunk*lengths.upper_trunk*masses.upper_trunk,0.0},
    {0.0,0.0,0.0807*0.0807*lengths.upper_trunk*lengths.upper_trunk*masses.upper_trunk},
  },
  upper_arm = {--- 0.0803, 0.0758, 0.0445
    {0.0803*0.0803*lengths.upper_arm*lengths.upper_arm*masses.upper_arm,0.0,0.0},
    {0.0,0.0758*0.0758*lengths.upper_arm*lengths.upper_arm*masses.upper_arm,0.0},
    {0.0,0.0,0.0445*0.0445*lengths.upper_arm*lengths.upper_arm*masses.upper_arm},
  },
  lower_arm = {--- 0.0742, 0.0713, 0.0325
    {0.0742*0.0742*lengths.lower_arm*lengths.lower_arm*masses.lower_arm,0.0,0.0},
    {0.0,0.0713*0.0713*lengths.lower_arm*lengths.lower_arm*masses.lower_arm,0.0},
    {0.0,0.0,0.0325*0.0325*lengths.lower_arm*lengths.lower_arm*masses.lower_arm},
  },
  hand = {--- 0.0541, 0.0442, 0.0346
    {0.0541*0.0541*lengths.hand*lengths.hand*masses.hand,0.0,0.0},
    {0.0,0.0442*0.0442*lengths.hand*lengths.hand*masses.hand,0.0},
    {0.0,0.0,0.0346*0.0346*lengths.hand*lengths.hand*masses.hand},
  },
  head = {--- 0.0736, 0.0634, 0.0765
    {0.0736*0.0736*lengths.head*lengths.head*masses.head,0.0,0.0},
    {0.0,0.0634*0.0634*lengths.head*lengths.head*masses.head,0.0},
    {0.0,0.0,0.0765*0.0765*lengths.head*lengths.head*masses.head},
  }
};

bodies = {
  pelvis = { mass = masses.pelvis, com = coms.pelvis,
    inertia = inertias.pelvis,
  },
  thigh = { mass = masses.thigh, com = coms.thigh,
    inertia = inertias.thigh,
  },
  shank = { mass = masses.shank, com = coms.shank,
    inertia = inertias.shank,
  },
  foot = { mass = masses.foot, com = coms.foot,
    inertia = inertias.foot,
  },
  middle_trunk = { mass = masses.middle_trunk, com = coms.middle_trunk,
    inertia = inertias.middle_trunk,
  },
  upper_trunk = { mass = masses.upper_trunk, com = coms.upper_trunk,
    inertia = inertias.upper_trunk,
  },
  upper_arm = { mass = masses.upper_arm, com = coms.upper_arm,
    inertia = inertias.upper_arm,
  },
  lower_arm = { mass = masses.lower_arm, com = coms.lower_arm,
    inertia = inertias.lower_arm,
  },
  hand = { mass = masses.hand, com = coms.hand,
    inertia = inertias.hand,
  },
  head = { mass = masses.head, com = coms.head,
    inertia = inertias.head,
  },
}

joints = {
  j_spherical = { "JointTypeSpherical" },
  --j_spherical = { "JointTypeEulerZYX" },
  j_translationXYZ = { "JointTypeTranslationXYZ" },
  j_rot_yz = {
    { 0., 1., 0., 0., 0., 0.},
    { 0., 0., 1., 0., 0., 0.}
  },
  j_rot_y = {
    { 0., 1., 0., 0., 0., 0.},
  },
  j_fixed = {}
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
      joint = joints.j_translationXYZ,
    },
    
    {
      name = "pelvis",
      parent = "virtual_pelvis",
      body = bodies.pelvis,
      joint = joints.j_spherical,
      visuals = {-- pelvis com
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.pelvis,
	  geometry = {
	    sphere = { radius=0.07 },
	  }
	},
      },
    },

    { -- right leg
      name = "thigh_r",
      parent = "pelvis",
      body = bodies.thigh,
      joint = joints.j_spherical,
      joint_frame = {
        r = {0.0,-0.0872,0.0},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.thigh,
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- hip
	  color = { 0.2,0.8,0.2 },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- knee
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.0,0.0,-lengths.thigh },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
      },
    },

    {
      name = "shank_r",
      parent = "thigh_r",
      body = bodies.shank,
      joint = joints.j_rot_y,
      joint_frame = {
        r = {0.0,0.0,-lengths.thigh},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.shank,
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- ankle
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.0,0.0,-lengths.shank },
	  geometry = {
	    sphere = { radius=0.015 },
	  },
	},
      },
    },

    {
      name = "foot_r",
      parent = "shank_r",
      body = bodies.foot,
      joint = joints.j_rot_y,
      joint_frame = {
        r = {0.0,0.0,-lengths.shank},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.foot,
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
        {-- heel
	  color = { 0.2,0.8,0.2 },
	  translate = { -0.01,0.0,-0.06195 },
	  geometry = {
	    sphere = { radius=0.04185 },
	  },
	},
        {-- meta5
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.1870,-0.05,-0.0787 },
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
        {-- hallux
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.1870,0.05,-0.0787 },
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
      },
    },

    { -- lef leg
      name = "thigh_l",
      parent = "pelvis",
      body = bodies.thigh,
      joint = joints.j_spherical,
      joint_frame = {
        r = {0.0,0.0872,0.0},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.thigh,
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- hip
	  color = { 0.2,0.8,0.2 },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- knee
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.0,0.0,-lengths.thigh },
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
      },
    },

    {
      name = "shank_l",
      parent = "thigh_l",
      body = bodies.shank,
      joint = joints.j_rot_y,
      joint_frame = {
        r = {0.0,0.0,-lengths.thigh},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.shank,
	  geometry = {
	    sphere = { radius=0.05 },
	  },
	},
        {-- ankle
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.0,0.0,-lengths.shank },
	  geometry = {
	    sphere = { radius=0.015 },
	  },
	},
      },
    },

    {
      name = "foot_l",
      parent = "shank_l",
      body = bodies.foot,
      joint = joints.j_rot_y,
      joint_frame = {
        r = {0.0,0.0,-lengths.shank},
      },
      visuals = {
        {
	  color = { 0.8,0.8,0.2 },
	  translate = coms.foot,
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
        {-- heel
	  color = { 0.2,0.8,0.2 },
	  translate = { -0.01,0.0,-0.06195 },
	  geometry = {
	    sphere = { radius=0.04185 },
	  },
	},
        {-- meta5
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.1870,0.05,-0.0787 },
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
        {-- hallux
	  color = { 0.2,0.8,0.2 },
	  translate = { 0.1870,-0.05,-0.0787 },
	  geometry = {
	    sphere = { radius=0.025 },
	  },
	},
      },
    },

  },
}

return model
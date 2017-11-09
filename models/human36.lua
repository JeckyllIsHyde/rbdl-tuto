meshes = {
  JointFixed = {
    name = "JointFixed",
    dimensions = { 0.1, 0.1, 0.1},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0},
    geometry = {
      box = {dimensions={1.,1.,1.}},
    },
  },
  JointSpherical = {
    name = "JointSpherical",
    dimensions = { 0.1, 0.1, 0.1},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0},
    geometry = {
      sphere = {radius=1.},
    },
  },
  JointRot = {
    name = "JointRot",
    dimensions = { 0.05, 0.05, 0.1},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0},
    geometry = {
      cylinder = {radius=0.5, length=1.0},
    },
  },
  LinkPelvis = {
    name = "LinkPelvis",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.2, 0.8},
    mesh_center = { 0, 0, 0.0891},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkPelvisBody = {
    name = "LinkPelvisBody",
    dimensions = { 0.05, 0.2, 0.1457},
    color = { 0.2, 0.2, 0.8},
    mesh_center = { 0, 0, 0.0891},
    geometry = {
      capsule = {radius=1., lenght=4.0},
    },
  },
  LinkThigh_r = {
    name = "LinkThigh_r",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1729},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkThighBody_r = {
    name = "LinkThighBody_r",
    dimensions = { 0.05, 0.05, 0.4222},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1729},
    geometry = {
      capsule = {radius=0.2, lenght=4.0},
    },
  },
  LinkThigh_l = {
    name = "LinkThigh_l",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1729},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkThighBody_l = {
    name = "LinkThighBody_l",
    dimensions = { 0.05, 0.05, 0.4222},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1729},
    geometry = {
      capsule = {radius=0.2, lenght=4.0},
    },
  },
  LinkShank_r = {
    name = "LinkShank_r",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1963},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkShankBody_r = {
    name = "LinkShankBody_r",
    dimensions = { 0.05, 0.05, 0.4403},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1963},
    geometry = {
      capsule = {radius=0.2, lenght=4.0},
    },
  },
  LinkShank_l = {
    name = "LinkShank_l",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1963},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkShankBody_l = {
    name = "LinkShankBody_l",
    dimensions = { 0.05, 0.05, 0.4403},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1963},
    geometry = {
      capsule = {radius=0.2, lenght=4.0},
    },
  },
  LinkFoot_r = {
    name = "LinkFoot_r",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0.1254, 0, -0.0516},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkHeel_r = {
    name = "LinkHeel_r",
    dimensions = { 0.1037, 0.1037, 0.1037},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { -0.01, 0, -0.05195},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkMeta5_r = {
    name = "LinkMeta5_r",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { 0.1870, 0.05, -0.0787},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkHallux_r = {
    name = "LinkHallux_r",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { 0.1870, -0.05, -0.0787},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkFoot_l = {
    name = "LinkFoot_l",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0.1254, 0, -0.0516},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkHeel_l = {
    name = "LinkHeel_l",
    dimensions = { 0.1037, 0.1037, 0.1037},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { -0.01, 0, -0.05195},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkMeta5_l = {
    name = "LinkMeta5_l",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { 0.1870, -0.05, -0.0787},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkHallux_l = {
    name = "LinkHallux_l",
    dimensions = { 0.05, 0.05, 0.05},
    color = { 0.1, 0.6, 0.1},
    mesh_center = { 0.1870, 0.05, -0.0787},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkMiddleTrunk = {
    name = "LinkMiddleTrunk",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0.1185},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkMiddleTrunkBody = {
    name = "LinkMiddleTrunkBody",
    dimensions = { 0.05, 0.24, 0.2155},
    color = { 0.2, 0.8, 0.2},
    mesh_center = { 0, 0, 0.1185},
    geometry = {
      capsule = {radius=0.2,cylinder=4.},
    },
  },
  LinkUpperTrunk = {
    name = "LinkUpperTrunk",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, 0.1195},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkUpperTrunkBody = {
    name = "LinkUpperTrunkBody",
    dimensions = { 0.05, 0.27, 0.2421},
    color = { 0.2, 0.8, 0.2},
    mesh_center = { 0, 0, 0.1195},
    geometry = {
      capsule = {radius=0.2,cylinder=4.},
    },
  },
  LinkHead = {
    name = "LinkHead",
    dimensions = { 0.12145, 0.12145, 0.12145},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0.1214},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkHeadBody = {
    name = "LinkHeadBody",
    dimensions = { 0.12145, 0.15, 0.2429},
    color = { 0.2, 0.8, 0.8},
    mesh_center = { 0, 0, 0.1214},
    geometry = {
      capsule = {radius=1.,length=4.},
    },
  },
  LinkUpperArm_r = {
    name = "LinkUpperArm_r",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1626},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkUpperArmBody_r = {
    name = "LinkUpperArmBody_r",
    dimensions = { 0.05, 0.05, 0.2817},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1026}, --- -0.1626
    geometry = {
      capsule = {radius=0.2,length=4.0},
    },
  },
  LinkLowerArm_r = {
    name = "LinkLowerArm_r",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1230},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkLowerArmBody_r = {
    name = "LinkLowerArmBody_r",
    dimensions = { 0.05, 0.05, 0.2689},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1230},
    geometry = {
      capsule = {radius=0.2,length=4.0},
    },
  },
  LinkHand_r = {
    name = "LinkHand_r",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.0680},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkUpperArm_l = {
    name = "LinkUpperArm_l",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1626},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkUpperArmBody_l = {
    name = "LinkUpperArmBody_l",
    dimensions = { 0.05, 0.05, 0.2817},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1026}, --- -0.1626
    geometry = {
      capsule = {radius=0.2,length=4.0},
    },
  },
  LinkLowerArm_l = {
    name = "LinkLowerArm_l",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.1230},
    geometry = {
      sphere = {radius=1.},
    },
  },
  LinkLowerArmBody_l = {
    name = "LinkLowerArmBody_l",
    dimensions = { 0.05, 0.05, 0.2689},
    color = { 0.8, 0.2, 0.2},
    mesh_center = { 0, 0, -0.1230},
    geometry = {
      capsule = {radius=0.2,length=4.0},
    },
  },
  LinkHand_l = {
    name = "LinkHand_l",
    dimensions = { 0.08, 0.08, 0.08},
    color = { 0.8, 0.8, 0.2},
    mesh_center = { 0, 0, -0.0680},
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
      name = "PELVIS",
      parent = "ROOT",
      joint_frame = {
        r = { 0.0, 0.0, 1.0 },
      },
      visuals = {
        meshes.LinkPelvis,
      },
    },
    {
      name = "PELVISBODY",
      parent = "PELVIS",
      visuals = {
        meshes.LinkPelvisBody,
      },
    },
    { --- RIGHT LEG
      name = "THIGH_R",
      parent = "PELVIS",
      joint_frame = {
        r = { 0.0, -0.0872, 0.0 },
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 1.0 }, 
	},
      },
      visuals = {
        meshes.LinkThigh_r,
      },
    },
    {
      name = "HIPJOINT_R",
      parent = "THIGH_R",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "THIGHBODY_R",
      parent = "THIGH_R",
      visuals = {
        meshes.LinkThighBody_r,
      },
    },
    {
      name = "SHANK_R",
      parent = "THIGH_R",
      joint_frame = {
        r = { 0.0, 0.0, -0.4222 },
      },
      visuals = {
        meshes.LinkShank_r,
      },
    },
    {
      name = "KNEEJOINT_R",
      parent = "SHANK_R",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "SHANKBODY_R",
      parent = "SHANK_R",
      visuals = {
        meshes.LinkShankBody_r,
      },
    },
    {
      name = "FOOT_R",
      parent = "SHANK_R",
      joint_frame = {
        r = { 0.0, 0.0, -0.4403 },
      },
      visuals = {
        meshes.LinkFoot_r,
      },
    },
    {
      name = "ANKLEZJOINT_R",
      parent = "FOOT_R",
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "ANKLEYJOINT_R",
      parent = "FOOT_R",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "HEEL_R",
      parent = "FOOT_R",
      visuals = {
        meshes.LinkHeel_r,
      },
    },
    {
      name = "META5_R",
      parent = "FOOT_R",
      visuals = {
        meshes.LinkMeta5_r,
      },
    },
    {
      name = "HALLUX_R",
      parent = "FOOT_R",
      visuals = {
        meshes.LinkHallux_r,
      },
    },
    { --- LEFT LEG
      name = "THIGH_L",
      parent = "PELVIS",
      joint_frame = {
        r = { 0.0, 0.0872, 0.0 },
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 1.0 }, 
	},
      },
      visuals = {
        meshes.LinkThigh_l,
      },
    },
    {
      name = "HIPJOINT_L",
      parent = "THIGH_L",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "THIGHBODY_L",
      parent = "THIGH_L",
      joint_frame = {
        r = { 0.0, 0, 0.0 },
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 1.0 }, 
	},
      },
      visuals = {
        meshes.LinkThighBody_l,
      },
    },
    {
      name = "SHANK_L",
      parent = "THIGH_L",
      joint_frame = {
        r = { 0.0, 0.0, -0.4222 },
      },
      visuals = {
        meshes.LinkShank_l,
      },
    },
    {
      name = "KNEEJOINT_L",
      parent = "SHANK_L",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "SHANKBODY_L",
      parent = "SHANK_L",
      visuals = {
        meshes.LinkShankBody_l,
      },
    },
    {
      name = "FOOT_L",
      parent = "SHANK_L",
      joint_frame = {
        r = { 0.0, 0.0, -0.4403 },
      },
      visuals = {
        meshes.LinkFoot_l,
      },
    },
    {
      name = "ANKLEZJOINT_L",
      parent = "FOOT_L",
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "ANKLEYJOINT_L",
      parent = "FOOT_L",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "HEEL_L",
      parent = "FOOT_L",
      visuals = {
        meshes.LinkHeel_l,
      },
    },
    {
      name = "META5_L",
      parent = "FOOT_L",
      visuals = {
        meshes.LinkMeta5_l,
      },
    },
    {
      name = "HALLUX_L",
      parent = "FOOT_L",
      visuals = {
        meshes.LinkHallux_l,
      },
    },
    { --- TRUNK
      name = "MIDDLE_TRUNK",
      parent = "PELVIS",
      joint_frame = {
        r = { 0.0, 0.0, 0.1457 },
      },
      visuals = {
        meshes.LinkMiddleTrunk,
      },
    },
    {
      name = "SPINEJOINT",
      parent = "MIDDLE_TRUNK",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "MIDDLE_TRUNKBODY",
      parent = "MIDDLE_TRUNK",
      visuals = {
        meshes.LinkMiddleTrunkBody,
      },
    },
    {
      name = "UPPER_TRUNK",
      parent = "MIDDLE_TRUNK",
      joint_frame = {
        r = { 0.0, 0.0, 0.2155 },
      },
      visuals = {
        meshes.LinkUpperTrunk,
      },
    },
    {
      name = "FIXEDJOINT",
      parent = "UPPER_TRUNK",
      visuals = {
        meshes.JointFixed,
      },
    },
    {
      name = "UPPER_TRUNKBODY",
      parent = "UPPER_TRUNK",
      visuals = {
        meshes.LinkUpperTrunkBody,
      },
    },
    {
      name = "HEAD",
      parent = "UPPER_TRUNK",
      joint_frame = {
        r = { 0.0, 0.0, 0.2421 },
      },
      visuals = {
        meshes.LinkHead,
      },
    },
    {
      name = "NECKJOINT",
      parent = "HEAD",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "HEADBODY",
      parent = "HEAD",
      visuals = {
        meshes.LinkHeadBody,
      },
    },
    { --- RIGHT ARM
      name = "UPPERARM_R",
      parent = "UPPER_TRUNK",
      joint_frame = {
        r = { 0.0, -0.1900, 0.2421 },
      },
      visuals = {
        meshes.LinkUpperArm_r,
      },
    },
    {
      name = "SHOULDERJOINT_R",
      parent = "UPPERARM_R",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "UPPERARMBODY_R",
      parent = "UPPERARM_R",
      visuals = {
        meshes.LinkUpperArmBody_r,
      },
    },
    {
      name = "LOWERARM_R",
      parent = "UPPERARM_R",
      joint_frame = {
        r = { 0.0, 0.0, -0.2817 },
      },
      visuals = {
        meshes.LinkLowerArm_r,
      },
    },
    {
      name = "ELBOWJOINT_R",
      parent = "LOWERARM_R",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "LOWERARMBODY_R",
      parent = "LOWERARM_R",
      visuals = {
        meshes.LinkLowerArmBody_r,
      },
    },
    {
      name = "HAND_R",
      parent = "LOWERARM_R",
      joint_frame = {
        r = { 0.0, 0.0, -0.2689 },
      },
      visuals = {
        meshes.LinkHand_r,
      },
    },
    {
      name = "WRISTZJOINT_R",
      parent = "HAND_R",
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "WRISTYJOINT_R",
      parent = "HAND_R",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    { --- LEFT ARM
      name = "UPPERARM_L",
      parent = "UPPER_TRUNK",
      joint_frame = {
        r = { 0.0, 0.1900, 0.2421 },
      },
      visuals = {
        meshes.LinkUpperArm_l,
      },
    },
    {
      name = "SHOULDERJOINT_L",
      parent = "UPPERARM_L",
      visuals = {
        meshes.JointSpherical,
      },
    },
    {
      name = "UPPERARMBODY_L",
      parent = "UPPERARM_L",
      visuals = {
        meshes.LinkUpperArmBody_l,
      },
    },
    {
      name = "LOWERARM_L",
      parent = "UPPERARM_L",
      joint_frame = {
        r = { 0.0, 0.0, -0.2817 },
      },
      visuals = {
        meshes.LinkLowerArm_l,
      },
    },
    {
      name = "ELBOWJOINT_L",
      parent = "LOWERARM_L",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "LOWERARMBODY_L",
      parent = "LOWERARM_L",
      visuals = {
        meshes.LinkLowerArmBody_l,
      },
    },
    {
      name = "HAND_L",
      parent = "LOWERARM_L",
      joint_frame = {
        r = { 0.0, 0.0, -0.2689 },
      },
      visuals = {
        meshes.LinkHand_l,
      },
    },
    {
      name = "WRISTZJOINT_L",
      parent = "HAND_L",
      visuals = {
        meshes.JointRot,
      },
    },
    {
      name = "WRISTYJOINT_L",
      parent = "HAND_L",
      joint_frame = {
	E = {
	  { 1.0, 0.0, 0.0 },
	  { 0.0, 0.0,-1.0 },
	  { 0.0, 1.0, 0.0 }, 
	},
      },
      visuals = {
        meshes.JointRot,
      },
    },
  }
}

return model

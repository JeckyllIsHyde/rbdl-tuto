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
      name = "rod",
      parent = "ROOT",
      body = {
        mass = 0.0
      },
      visuals = {
        {
	  color = { 0.8, 0.2, 0.8 },
	  rotate = {
	    axis = {0.0,1.0,0.0},
	    angle = 90
	  },
	  geometry = {
	    capsule = { radius=0.05, length=0.6 }
	  },
	},
        {
	  color = { 0.2, 0.8, 0.8 },
	  geometry = {
	    sphere = { radius=0.1 }
	  },
	},
      },
    },
    
  },
}

return model
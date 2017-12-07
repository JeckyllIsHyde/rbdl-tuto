#include "Animation.h"

#include <fstream>
#include <sstream>

#include "PhysicsEngine.h"

void makeAnimationFilesForOnlySpheres( PhysicsEngine& engine ) {
  std::string head_code = ""
    "meshes = {\n"
    "  SphereBody = {\n"
    "    name = \"SphereBody\",\n"
    "    dimensions = { 0.2, 0.2, 0.2},\n"
    "    color = { 0.8, 0.8, 0.2},\n"
    "    mesh_center = { 0, 0, 0},\n"
    "    src = \"meshes/unit_sphere_medres.obj\"\n"
    "  },\n"
    "}\n"
    "\n"
    "model = {\n"
    "  configuration = {\n"
    "    axis_front = { 1, 0, 0 },\n"
    "    axis_up    = { 0, 0, 1 },\n"
    "    axis_right = { 0, -1, 0 },\n"
    "  },\n"
    "\n"
    "  frames = {\n",
    tail_code = ""
    "  }\n"
    "}\n"
    "\n"
    "return model\n";

  std::ofstream luafile("spheres.lua");
  if (luafile.is_open()) {
    luafile << head_code;
    int i=0;
    for ( PhysicsEngine::SysIterator sys_it=engine.systems.begin();
	  sys_it<engine.systems.end(); sys_it++, i++ ) {
      std::stringstream name("BODY"); name << i;
      luafile << addSphereToLuaFile( name.str(), Vector3dZero );
    }
    luafile  << tail_code;
    luafile.close();
  } else
    std::cout << "Unable to open lua-file" << std::endl;

  head_code = ""
    "COLUMNS:\n"
    "Time,\n";
  tail_code = ""
    "DATA_FROM: data/spheres_data.csv\n";

  std::ofstream csvfile("spheres.csv");
  if (csvfile.is_open()) {
    csvfile << head_code;
    int i=0;
    for ( PhysicsEngine::SysIterator sys_it=engine.systems.begin();
	  sys_it<engine.systems.end(); sys_it++, i++ ) {
      std::stringstream name("BODY"); name << i;
      csvfile << addSphereToCsvFile( name.str() );
    }
    csvfile << tail_code;
    csvfile.close();
  } else
    std::cout << "Unable to open csv-file" << std::endl;

}

std::string addSphereToLuaFile( std::string BODYName, Vector3d pos ) {
  std::stringstream spheres;
  spheres << ""
    "    {\n"
    "      name = \"" << BODYName << "\",\n"
    "      parent = \"ROOT\",\n"
    "      visuals = {\n"
    "        meshes.SphereBody,\n"
    "      },\n"
    "      joint_frame = {\n"
    "        r = { "<< pos[0] <<","<< pos[1] <<","<< pos[2] <<" },\n"
    "      },\n"
    "    },\n";
  return spheres.str();
}


std::string addSphereToCsvFile( std::string BODYName ) {
  std::stringstream spheres;
  spheres << ""
	  << BODYName <<":T:X,\n"
	  << BODYName <<":T:Y,\n"
	  << BODYName <<":T:Z,\n"
	  << BODYName <<":R:Z,\n"
	  << BODYName <<":R:Y,\n"
	  << BODYName <<":R:X,\n";
  return spheres.str();
}

std::string addBodyMeshDefinitionToLuaFile( std::string BODYName,
					    Vector3d bbox,
					    Vector3d color,
					    Vector3d com ) {
  std::stringstream spheres;
  spheres << ""
    "  Link" << BODYName <<" = {\n"
    "    name = \"Link" << BODYName <<"\",\n"
    "    dimensions = { 0.08, 0.08, 0.08 },\n"
    "    color = { 0.8, 0.2, 0.8 },\n"
    "    mesh_center = { "<< com[0] <<","<< com[1] <<","<< com[2] <<" },\n"
    "    geometry = {\n"
    "      sphere = {radius=1.},\n"
    "    },\n"
    "  },\n";
  spheres << ""
    "  LinkBody" << BODYName <<" = {\n"
    "    name = \"LinkBody" << BODYName <<"\",\n"
    "    dimensions = { "<< bbox[0] <<","<< bbox[1] <<","<< bbox[2] <<"},\n"
    "    color = { "<< color[0] <<","<< color[1] <<","<< color[2] <<" },\n"
    "    mesh_center = { "<< com[0] <<","<< com[1] <<","<< com[2] <<" },\n"
    "    geometry = {\n"
    "      capsule = {radius=0.2, length=4.0},\n"
    "    },\n"
    "  },\n";

  return spheres.str();
}

std::string addJointMeshToGraphToLuaFile( JointTypeForFile jtype ) {
  
}

std::string addBodyToGraphToLuaFile( std::string BODYName,
				     std::string PARENTBODYName,
				     SpatialTransform X,
				     JointTypeForFile jtype ) {
  std::stringstream spheres;
  spheres << ""
    "    {\n"
    "      name = \"" << BODYName << "\",\n"
    "      parent = \"" << PARENTBODYName << "\",\n"
    "      joint_frame = {\n"
    "        r = { "<< X.r[0] <<","<< X.r[1] <<","<< X.r[2] <<" },\n"
    "	     E = {\n"
    "	       { "<< X.E[0][0] <<", "<< X.E[0][1] <<", "<< X.E[0][2] <<" },\n"
    "	       { "<< X.E[1][0] <<", "<< X.E[1][1] <<", "<< X.E[1][2] <<" },\n"
    "	       { "<< X.E[2][0] <<", "<< X.E[2][1] <<", "<< X.E[2][2] <<" },\n"
    "	     },\n"
    "      },\n"
    "      visuals = {\n"
    "        meshes.Link" << BODYName << ",\n"
    "      },\n"
    "    },\n"
    "    {\n"
    "      name = \"" << BODYName << "\",\n"
    "      parent = \"" << BODYName << "\",\n"
    "      visuals = {\n"
    "        meshes." << getJointNameToLuaFile(jtype) << ",\n"
    "      },\n"
    "    },\n";
  return spheres.str();
}

std::string getJointNameToLuaFile( JointTypeForFile jtype ) {
  std::stringstream spheres;
  if ( jtype==FREEFLAYER ) {
    spheres << "JointFree";
  } else if (jtype==ROTYXZ) {
    spheres << "JointSpherical";
  } else if (jtype==ROTYZ ||
	     jtype==ROTY) {
    spheres << "JointRot";
  } else {
    spheres << "";
  }
  return spheres.str();
}

std::string addBodyToCsvFile( std::string BODYName, JointTypeForFile jtype ) {
  std::stringstream spheres;
  if ( jtype==FREEFLAYER ) {
    spheres << ""
	    << BODYName <<":T:X,\n"
	    << BODYName <<":T:Y,\n"
	    << BODYName <<":T:Z,\n"
	    << BODYName <<":R:Y,\n"
	    << BODYName <<":R:Z,\n"
	    << BODYName <<":R:X,\n";
  } else if (jtype==ROTYXZ) {
    spheres << ""
	    << BODYName <<":R:Y,\n"
	    << BODYName <<":R:X,\n"
	    << BODYName <<":R:Z,\n";
  } else if (jtype==ROTYZ) {
    spheres << ""
	    << BODYName <<":R:Y,\n"
	    << BODYName <<":R:Z,\n";
  } else if (jtype==ROTY) {
    spheres << ""
	    << BODYName <<":R:Y,\n";
  } else {
    spheres << "";
      std::cout << "define!!!" << std::endl;
  }
  return spheres.str();
}

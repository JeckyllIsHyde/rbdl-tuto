#include "Animation.h"

#include <fstream>
#include <sstream>

#include "PhysicsEngine.h"

void makeAnimationFiles( PhysicsEngine& engine ) {
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

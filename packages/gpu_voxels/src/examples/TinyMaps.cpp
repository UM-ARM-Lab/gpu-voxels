// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Brad Saund <brad.saund@gmail.com>
* \date    2018-05-01
*
*
* This example uses very small voxel maps. 
* This does not demonstrate typical usage, but 
* instead is simple enough to directly view the numerical representation.
*/
//----------------------------------------------------------------------



#include <signal.h>


#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

gpu_voxels::GpuVoxelsSharedPtr gvl;

void ctrlchandler(int)
{
  gvl.reset();
  exit(EXIT_SUCCESS);
}
void killhandler(int)
{
  gvl.reset();
  exit(EXIT_SUCCESS);
}

int main(int argc, char* argv[])
{
  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  icl_core::logging::initialize(argc, argv);

  /*
   * First, we generate an API class, which defines the
   * volume of our space and the resolution.
   * Be careful here! The size is limited by the memory
   * of your GPU. Even if an empty Octree is small, a
   * Voxelmap will always require the full memory.
   */
  gvl = gpu_voxels::GpuVoxels::getInstance();
  gvl->initialize(2, 2, 2, 1.0);



  // Now we can add different types of maps and assign a name.
  gvl->addMap(gpu_voxels::MT_BITVECTOR_VOXELMAP, "myBitmapVoxmap");           // 3D-Array of deterministic Voxels (identified by their Voxelmap-like Pointer adress) that hold a Bitvector
  gvl->addMap(gpu_voxels::MT_BITVECTOR_VOXELLIST, "myBitmapVoxlist");         // List of     deterministic Voxels (identified by their Voxelmap-like Pointer adress) that hold a Bitvector
  gvl->addMap(gpu_voxels::MT_BITVECTOR_OCTREE, "myBitmapOctree");             // Octree of   deterministic Voxels (identified by a Morton Code)                      that hold a Bitvector


  gvl->addMap(gpu_voxels::MT_PROBAB_VOXELMAP, "myProbabVoxmap");              // 3D-Array of probabilistic Voxels (identified by their Voxelmap-like Pointer adress) that hold a Probability
  gvl->addMap(gpu_voxels::MT_PROBAB_OCTREE, "myProbabOctree");                // Octree of   probabilistic Voxels (identified by a Morton Code)                      that hold a Probability

  const char* args[] = {"myBitmapVoxmap", "myBitmapVoxlist", "myBitmapOctree",
                        "myProbabVoxmap", "myProbabOctree"};
  std::vector<std::string> all_maps(args, args+5);



  

  // Add collisiion elements to voxelmaps
  std::cout << "\n\nAdding Collision\n";
  for(std::vector<std::string>::size_type i=0; i<all_maps.size(); i++)
  {
      gpu_voxels::GpuVoxelsMapSharedPtr map = gvl->getMap(all_maps[i]);

      std::cout << all_maps[i] << ": " << map->getMemoryUsage() << "\n";
  }
  std::cout << "\n";




  // Printing memory usage
  std::cout << "\n\nMemory Usage\n";
  for(std::vector<std::string>::size_type i=0; i<all_maps.size(); i++)
  {
      gpu_voxels::GpuVoxelsMapSharedPtr map = gvl->getMap(all_maps[i]);

      std::cout << all_maps[i] << ": " << map->getMemoryUsage() << "\n";
  }

  gpu_voxels::Vector3f center_box1_min(0.0,0.0,0.0);
  gpu_voxels::Vector3f center_box1_max(0.5,0.5,0.5);
  gvl->insertBoxIntoMap(center_box1_min,center_box1_max,"myBitmapVoxmap",gpu_voxels::eBVM_OCCUPIED);
  gvl->insertBoxIntoMap(center_box1_min,center_box1_max,"myProbabVoxmap",gpu_voxels::eBVM_OCCUPIED);

  
  gpu_voxels::Vector3f center_box2_min(1.0,1.0,1.0);
  gpu_voxels::Vector3f center_box2_max(1.5,1.5,1.5);
  gvl->insertBoxIntoMap(center_box2_min,center_box2_max,"myBitmapVoxmap",gpu_voxels::eBVM_OCCUPIED);
  


  // gvl->getMap("myProbabVoxmap")->as<voxelmap::ProbVoxelMap>()->printVoxelMapData();
  // gvl->getMap("myBitmapVoxmap")->as<voxelmap::BitVectorVoxelMap>()->printVoxelMapData();
  
  voxelmap::ProbVoxelMap* pmap = gvl->getMap("myProbabVoxmap")->as<voxelmap::ProbVoxelMap>();

  
  for(uint32_t i=0; i<pmap->getVoxelMapSize(); i++)
  {
      // std::cout << pmap->getConstDeviceDataPtr()[i] << "\n";
  }



  while(true)
  {
    // Tell the visualier that maps have changed.
    gvl->visualizeMap("myBitmapVoxmap");

    usleep(100000);

  }

}


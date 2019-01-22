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
* \author  Brad Saund
* \date    2019-01-22
*
*/
//----------------------------------------------------------------------


#ifndef GPU_VOXELS_VOXELLIST_PROBVOXELLIST_HPP
#define GPU_VOXELS_VOXELLIST_PROBVOXELLIST_HPP

#include "ProbVoxelList.h"
#include <gpu_voxels/logging/logging_voxellist.h>

namespace gpu_voxels {
namespace voxellist {

ProbVoxelList::ProbVoxelList(const Vector3ui ref_map_dim,
                             const float voxel_side_length,
                             const MapType map_type)
        : TemplateVoxelList<ProbabilisticVoxel, MapVoxelID>(ref_map_dim, voxel_side_length, map_type)
{

}


ProbVoxelList::~ProbVoxelList()
{
}

void ProbVoxelList::clearBitVoxelMeaning(BitVoxelMeaning voxel_meaning)
{
  LOGGING_ERROR_C(
    VoxellistLog, CountingVoxelList, GPU_VOXELS_MAP_OPERATION_NOT_YET_SUPPORTED << endl);
}

 bool ProbVoxelList::merge(const voxelmap::ProbVoxelMap *other, const Vector3i &voxel_offset)
{
    const gpu_voxels::voxelmap::ProbVoxelMap* m = other->as<voxelmap::ProbVoxelMap>();
    thrust::device_vector<Vector3ui> occupied_coords = m->getDeviceOccupiedCoords();
    insertCoordinateList(thrust::raw_pointer_cast(&occupied_coords.front()),
                         occupied_coords.size(), eBVM_OCCUPIED);
    return true;

}


}
}


#endif

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

#ifndef GPU_VOXELS_VOXELLIST_PROBVOXELLIST_H
#define GPU_VOXELS_VOXELLIST_PROBVOXELLIST_H

#include <gpu_voxels/voxel/ProbabilisticVoxel.h>
#include <gpu_voxels/voxellist/TemplateVoxelList.h>
#include <cstddef>

namespace gpu_voxels {
namespace voxellist {

class ProbVoxelList : public TemplateVoxelList<ProbabilisticVoxel, MapVoxelID>
{
public:
  // This can either represent a MORTON or Voxelmap Bitvector Voxel List:
  //  typedef CountingVoxelList<VoxelIDType> TemplatedCountingVoxelList;

  ProbVoxelList(const gpu_voxels::Vector3ui ref_map_dim,
                const float voxel_side_length,
                const gpu_voxels::MapType map_type);

  ~ProbVoxelList();

  virtual void clearBitVoxelMeaning(BitVoxelMeaning voxel_meaning);

  virtual MapType getTemplateType() { return this->m_map_type; }
  virtual bool merge(const voxelmap::ProbVoxelMap *other, const Vector3i &voxel_offset = Vector3i());

  using TemplateVoxelList<ProbabilisticVoxel, MapVoxelID>::merge;
                     




private:

};

} // end namespace voxellist
} // end namespace gpu_voxels


#endif

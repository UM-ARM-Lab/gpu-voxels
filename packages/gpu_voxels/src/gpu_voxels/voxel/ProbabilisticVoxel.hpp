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
 * \author  Florian Drews
 * \date    2014-07-08
 *
 */
//----------------------------------------------------------------------/*
#ifndef GPU_VOXELS_VOXEL_PROBABILISTIC_VOXEL_HPP_INCLUDED
#define GPU_VOXELS_VOXEL_PROBABILISTIC_VOXEL_HPP_INCLUDED

#include "ProbabilisticVoxel.h"

namespace gpu_voxels {

#ifdef __CUDACC__
#define MIN(x,y) min(x,y)
#define MAX(x,y) max(x,y)
#else
#define MIN(x,y) std::min(x,y)
#define MAX(x,y) std::max(x,y)
#endif

__host__ __device__
ProbabilisticVoxel::ProbabilisticVoxel() :
    m_occupancy(UNKNOWN_PROBABILITY)
{

}

__host__ __device__
ProbabilisticVoxel::~ProbabilisticVoxel()
{

}

__host__ __device__
Probability ProbabilisticVoxel::updateOccupancy(const Probability occupancy)
{
  // watch out for overflow: cast to int32_t
  m_occupancy = MIN(MAX( int32_t(int32_t(m_occupancy) + int32_t(occupancy)), int32_t(MIN_PROBABILITY)), int32_t(MAX_PROBABILITY));
  return m_occupancy;
}

__host__ __device__
Probability& ProbabilisticVoxel::occupancy()
{
  return m_occupancy;
}

__host__ __device__
const Probability& ProbabilisticVoxel::occupancy() const
{
  return m_occupancy;
}

__host__ __device__
Probability ProbabilisticVoxel::getOccupancy() const
{
  return m_occupancy;
}

__host__ __device__
void ProbabilisticVoxel::insert(const uint32_t voxel_meaning)
{
  // m_occupancy = MAX_PROBABILITY;
    m_occupancy = (Probability)voxel_meaning + UNKNOWN_PROBABILITY;
}

__host__ __device__
bool ProbabilisticVoxel::isOccupied(float col_threshold) const
{
  return m_occupancy >= col_threshold;
}

__host__ __device__
ProbabilisticVoxel ProbabilisticVoxel::reduce(const ProbabilisticVoxel voxel, const ProbabilisticVoxel other_voxel)
{
  ProbabilisticVoxel res = voxel;
  res.updateOccupancy(other_voxel.getOccupancy());
  return res;
}

} // end of ns
#endif

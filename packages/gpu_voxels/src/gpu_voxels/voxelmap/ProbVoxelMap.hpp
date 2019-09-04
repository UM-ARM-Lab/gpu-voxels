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
 * \date    2014-07-09
 *
 */
//----------------------------------------------------------------------
#ifndef GPU_VOXELS_VOXELMAP_PROB_VOXELMAP_HPP_INCLUDED
#define GPU_VOXELS_VOXELMAP_PROB_VOXELMAP_HPP_INCLUDED

#include "ProbVoxelMap.h"
#include <gpu_voxels/voxelmap/TemplateVoxelMap.hpp>
#include <gpu_voxels/voxelmap/kernels/VoxelMapOperations.hpp>
#include <gpu_voxels/voxel/BitVoxel.hpp>
#include <gpu_voxels/voxel/ProbabilisticVoxel.hpp>
#include <thrust/copy.h>

namespace gpu_voxels {
namespace voxelmap {

ProbVoxelMap::ProbVoxelMap(const Vector3ui dim, const float voxel_side_length, const MapType map_type) :
    Base(dim, voxel_side_length, map_type)
{

}

ProbVoxelMap::ProbVoxelMap(Voxel* dev_data, const Vector3ui dim, const float voxel_side_length, const MapType map_type) :
    Base(dev_data, dim, voxel_side_length, map_type)
{

}

ProbVoxelMap::~ProbVoxelMap()
{

}

void ProbVoxelMap::subtract(const ProbVoxelMap *other)
{
    kernelSubtractMaps<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, other->m_dev_data);
}    

void ProbVoxelMap::add(const ProbVoxelMap *other)
{
    kernelAddMaps<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, other->m_dev_data);
}

void ProbVoxelMap::copy(const ProbVoxelMap *other)
{
    kernelCopyMaps<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, other->m_dev_data);
}

void ProbVoxelMap::dialate(const ProbVoxelMap *other, int dist)
{
    kernelDialateMap<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, getDimensions(),
                                              other->m_dev_data, dist);
}

template<std::size_t length>
void ProbVoxelMap::insertSensorData(const PointCloud &global_points, const Vector3f &sensor_pose, const bool enable_raycasting,
                                    const bool cut_real_robot, const BitVoxelMeaning robot_voxel_meaning,
                                    BitVoxel<length>* robot_map)
{
  lock_guard guard(this->m_mutex);

  computeLinearLoad(global_points.getPointCloudSize(), &m_blocks,
                           &m_threads);

  if (enable_raycasting)
  {
    kernelInsertSensorData<<<m_blocks, m_threads>>>(
        m_dev_data, m_voxelmap_size, m_dim, m_voxel_side_length, sensor_pose,
        global_points.getConstDevicePointer(), global_points.getPointCloudSize(), cut_real_robot, robot_map, robot_voxel_meaning, RayCaster());
    CHECK_CUDA_ERROR();
  }
  else
  {
    kernelInsertSensorData<<<m_blocks, m_threads>>>(
        m_dev_data, m_voxelmap_size, m_dim, m_voxel_side_length, sensor_pose,
        global_points.getConstDevicePointer(), global_points.getPointCloudSize(), cut_real_robot, robot_map, robot_voxel_meaning, DummyRayCaster());
    CHECK_CUDA_ERROR();
  }
  HANDLE_CUDA_ERROR(cudaDeviceSynchronize());
}

bool ProbVoxelMap::insertMetaPointCloudWithSelfCollisionCheck(const MetaPointCloud *robot_links,
                                                              const std::vector<BitVoxelMeaning>& voxel_meanings,
                                                              const std::vector<BitVector<BIT_VECTOR_LENGTH> >& collision_masks,
                                                              BitVector<BIT_VECTOR_LENGTH>* colliding_meanings)
{
  LOGGING_ERROR_C(VoxelmapLog, ProbVoxelMap, GPU_VOXELS_MAP_OPERATION_NOT_SUPPORTED << endl);
  return true;
}

void ProbVoxelMap::clearBitVoxelMeaning(BitVoxelMeaning voxel_meaning)
{
  if(voxel_meaning != eBVM_OCCUPIED)
     LOGGING_ERROR_C(VoxelmapLog, ProbVoxelMap, GPU_VOXELS_MAP_ONLY_SUPPORTS_BVM_OCCUPIED << endl);
  else
    this->clearMap();
}

//Collsion Interface Implementations

size_t ProbVoxelMap::collideWith(const BitVectorVoxelMap *map, float coll_threshold, const Vector3i &offset)
{
  DefaultCollider collider(coll_threshold);
  return collisionCheckWithCounterRelativeTransform((TemplateVoxelMap*)map, collider, offset); //does the locking

}

size_t ProbVoxelMap::collideWith(const ProbVoxelMap *map, float coll_threshold, const Vector3i &offset)
{
  DefaultCollider collider(coll_threshold);
  return collisionCheckWithCounterRelativeTransform((TemplateVoxelMap*)map, collider, offset); //does the locking
}

bool ProbVoxelMap::overlapsWith(const voxelmap::ProbVoxelMap* other, float coll_threshold) const 
{
  DefaultCollider collider(coll_threshold);
  bool *dev_overlap_result;
  bool host_overlap_result = false;
  cudaMalloc(&dev_overlap_result, sizeof(bool));
  cudaMemcpy(dev_overlap_result, &host_overlap_result, sizeof(bool), cudaMemcpyHostToDevice);
  
  kernelOverlaps<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, other->m_dev_data,
                                          collider, dev_overlap_result);
  
  cudaMemcpy(&host_overlap_result, dev_overlap_result, sizeof(bool), cudaMemcpyDeviceToHost);
  cudaFree(dev_overlap_result);
  return host_overlap_result;
}



size_t ProbVoxelMap::countOccupied() const
{
    HANDLE_CUDA_ERROR(cudaDeviceSynchronize());
    kernelCountOccupied<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size,
                                                 m_dev_collision_check_results_counter);
    CHECK_CUDA_ERROR();
    HANDLE_CUDA_ERROR(cudaDeviceSynchronize());
    HANDLE_CUDA_ERROR(
        cudaMemcpy(m_collision_check_results_counter, m_dev_collision_check_results_counter,
                   cMAX_NR_OF_BLOCKS * sizeof(uint16_t), cudaMemcpyDeviceToHost));


    uint32_t num_occupied = 0;
    for(uint32_t i=0; i< m_blocks; i++)
    {
        num_occupied += m_collision_check_results_counter[i];
    }
    return num_occupied;
}

std::vector<Vector3f> ProbVoxelMap::getOccupiedCenters() const
{
    thrust::device_vector<uint32_t> dev_occ_indices(m_voxelmap_size);
    thrust::counting_iterator<uint32_t> first(0);
    thrust::counting_iterator<uint32_t> last = first + m_voxelmap_size;
    
    typedef thrust::device_vector<uint32_t>::iterator IndexIterator;
    IndexIterator dev_occ_indices_end = thrust::copy_if(first, last,
                                                        thrust::device_pointer_cast(m_dev_data),
                                                        dev_occ_indices.begin(),
                                                        check_is_occupied());


    thrust::device_vector<Vector3f> dev_centers(dev_occ_indices_end - dev_occ_indices.begin());
    thrust::transform(dev_occ_indices.begin(), dev_occ_indices_end,
                      dev_centers.begin(),
                      map_to_voxel_centers(m_dim, m_voxel_side_length));
    
    thrust::host_vector<Vector3f> centers(dev_centers);
    std::vector<Vector3f> stdcenters(centers.begin(), centers.end());
    return stdcenters;
}

thrust::device_vector<Vector3ui> ProbVoxelMap::getDeviceOccupiedCoords() const
{
    thrust::device_vector<uint32_t> dev_occ_indices(m_voxelmap_size);
    thrust::counting_iterator<uint32_t> first(0);
    thrust::counting_iterator<uint32_t> last = first + m_voxelmap_size;
    
    typedef thrust::device_vector<uint32_t>::iterator IndexIterator;
    IndexIterator dev_occ_indices_end = thrust::copy_if(first, last,
                                                        thrust::device_pointer_cast(m_dev_data),
                                                        dev_occ_indices.begin(),
                                                        check_is_occupied());

    thrust::device_vector<Vector3ui> dev_coords(dev_occ_indices_end - dev_occ_indices.begin());
    thrust::transform(dev_occ_indices.begin(), dev_occ_indices_end,
                      dev_coords.begin(),
                      map_to_voxels(m_dim));
    return dev_coords;
}

void ProbVoxelMap::copyIthOccupied(const voxelmap::ProbVoxelMap* other, unsigned long copy_index) const
{
    unsigned long long int* d_counter;
    unsigned long long int h_counter = 0;
    cudaMalloc(&d_counter, sizeof(unsigned long long int));
    cudaMemcpy(d_counter, &h_counter, sizeof(unsigned long long int), cudaMemcpyHostToDevice);

    // kernelCountOccupied<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, d_counter);
    // cudaMemcpy(&h_counter, d_counter, sizeof(unsigned long long int), cudaMemcpyDeviceToHost);

    // std::default_random_engine generator;
    // std::uniform_int_distribution<unsigned long> dist(0, m_voxelmap_size);
    // unsigned long rand_index = distribution(generator);

    kernelCopyIthOccupied<<<m_blocks, m_threads>>>(m_dev_data, m_voxelmap_size, other->m_dev_data, d_counter, copy_index);

    // cudaMemcpy(&h_counter, d_counter, sizeof(unsigned long long int), cudaMemcpyDeviceToHost);
    
    // std::cout << "num blocks: " << m_blocks << "\n";
    // std::cout << "num threads: " << m_threads << "\n";
    // std::cout << "counter: " << h_counter << "\n";
    
    
    cudaFree(d_counter);
}

} // end of namespace
} // end of namespace

#endif

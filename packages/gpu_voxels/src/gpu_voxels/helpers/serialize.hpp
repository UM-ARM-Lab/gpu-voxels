#ifndef GPU_VOXELS_SERIALIZE_HPP
#define GPU_VOXELS_SERIALIZE_HPP

#include <vector>
#include <cstring>
#include <stdint.h>
#include <gpu_voxels/logging/logging_gpu_voxels_helpers.h>

namespace gpu_voxels
{
    /**********************************************************
     **  Serializes data of known size into an existing buffer
     **  Returns the number of bytes written to the buffer
     *********************************************************/
    template<typename T>
    inline uint64_t serialize(const T& item_to_serialize, std::vector<uint8_t>& buffer);

    template<typename T>
    inline uint64_t serialize(const T& first, uint64_t num_bytes,
                                   std::vector<uint8_t>& buffer);

    /**********************************************************
     **  Deserializes data of known size from an existing buffer
     *********************************************************/

    template<typename T>
    inline void deserialize(T& deserialized, const std::vector<uint8_t> &buffer,
                                uint64_t &buffer_index);

    template<typename T>
    inline void deserialize(T& deserialized_first, uint64_t num_bytes,
                                const std::vector<uint8_t> &buffer, uint64_t &buffer_index);
    


    

    /************************************************************
     **                 Implementations
     ***********************************************************/

    template<typename T>
    inline uint64_t serialize(
        const T& item_to_serialize,
        std::vector<uint8_t>& buffer)
    {
        return serialize(item_to_serialize, sizeof(item_to_serialize), buffer);
    }


    template<typename T>
    inline uint64_t serialize(
        const T& first, uint64_t num_bytes,
        std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // Fixed-size serialization via memcpy
        std::vector<uint8_t> temp_buffer(num_bytes, 0x00);
        std::memcpy(&temp_buffer[0], &first, num_bytes);
        // Move to buffer
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        // Figure out how many bytes were written
        return  buffer.size() - start_buffer_size;
    }



    template<typename T>
    inline void deserialize(T& deserialized, const std::vector<uint8_t> &buffer, uint64_t &buffer_index)
    {
        deserialize(deserialized, sizeof(deserialized), buffer, buffer_index);
    }

    template<typename T>
    inline void deserialize(T& deserialized_first, uint64_t num_bytes,
                            const std::vector<uint8_t> &buffer, uint64_t &buffer_index)
    {
        if(buffer_index + num_bytes > buffer.size())
        {
            LOGGING_ERROR(Gpu_voxels_helpers, "Deserialization Failed: attempted to deserialize from " <<
                            buffer_index << " to " << buffer_index + num_bytes << " on a buffer of size " <<
                            buffer.size());
        }
        std::memcpy(&deserialized_first, &buffer[buffer_index], num_bytes);
        
        buffer_index += num_bytes;
    }
}


#endif

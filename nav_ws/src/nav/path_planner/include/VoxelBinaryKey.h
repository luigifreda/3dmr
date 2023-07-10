#ifndef VOXEL_BINARY_KEY_H
#define VOXEL_BINARY_KEY_H

template<typename PointT>
inline uint64_t voxelBinaryKey(const PointT& p, double leaf_size, uint8_t key_size = 64)
{
	const uint8_t bpp = key_size / 3; // bits per each component
	const uint64_t mask = (1 << bpp) - 1;
	uint64_t ix = (1 << (bpp - 1)) + round(p.x / leaf_size);
	uint64_t iy = (1 << (bpp - 1)) + round(p.y / leaf_size);
	uint64_t iz = (1 << (bpp - 1)) + round(p.z / leaf_size);
    ix &= mask;
    iy &= mask;
    iz &= mask;
    iy <<= bpp;
    iz <<= bpp;
    iz <<= bpp;
    const uint64_t ret = ix | iy | iz;
    return ret;
}

#endif // VOXEL_BINARY_KEY_H

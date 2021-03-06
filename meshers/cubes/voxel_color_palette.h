#ifndef VOXEL_COLOR_PALETTE_H
#define VOXEL_COLOR_PALETTE_H

#include "../../math/color8.h"
#include "../../util/fixed_array.h"
#include <core/resource.h>
#include <vector>

// Associates small numbers to colors, so colored voxels can be specified using less memory.
class VoxelColorPalette : public Resource {
	GDCLASS(VoxelColorPalette, Resource)
public:
	static const unsigned int MAX_COLORS = 256;

	void set_color(int index, Color color);
	Color get_color(int index) const;

	void clear();

	// Internal

	inline void set_color8(uint8_t i, Color8 c) {
		_colors[i] = c;
	}

	inline Color8 get_color8(uint8_t i) const {
		return _colors[i];
	}

private:
	PoolIntArray _b_get_data() const;
	void _b_set_data(PoolIntArray colors);

	static void _bind_methods();

	FixedArray<Color8, MAX_COLORS> _colors;
};

#endif // VOXEL_COLOR_PALETTE_H

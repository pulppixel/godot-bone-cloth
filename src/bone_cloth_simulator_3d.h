// Bone cloth: spring chains linked to their neighbours so a skirt of several chains moves as one surface and stays out of the legs.

#pragma once

#include <godot_cpp/classes/skeleton_modifier3d.hpp>

namespace godot {

	class bone_cloth_simulator_3d : public SkeletonModifier3D {
		GDCLASS(BoneClothSimulator3D, SkeletonModifier3D)

	protected:
		static void _bind_methods();

	public:
		void _process_modification_with_delta(double p_delta) override;
	};


}



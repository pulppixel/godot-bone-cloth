// A plane on a bone, SpringBoneCollisionPlane3D's: endless, through the node and facing its Y; the joints stay on the side it faces.

#pragma once

#include "bone_cloth_collision_3d.h"

namespace godot {

	class BoneClothCollisionPlane3D : public BoneClothCollision3D {
		GDCLASS(BoneClothCollisionPlane3D, BoneClothCollision3D)

	protected:
		static void _bind_methods();

	public:
		Vector3 collide(const Vector3& p_point, float p_radius) const override;
		void collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const override;

	};
}

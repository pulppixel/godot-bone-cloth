// A capsule on a bone, SpringBoneCollisionCapsule3D's: the joints stay out of it, or in it with inside on. The axis is the node's Y.
// The push is KawaiiPhysics' capsule collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "bone_cloth_collision_3d.h"


namespace godot {

	class BoneClothCollisionCapsule3D : public BoneClothCollision3D {
		GDCLASS(BoneClothCollisionCapsule3D, BoneClothCollision3D)

			float radius = 0.1f;
		float height = 0.5f;
		bool inside = false;

		void _get_head_and_tail(Vector3& r_head, Vector3& r_tail) const;

	protected:
		static void _bind_methods();

	public:
		void set_radius(float p_radius);
		float get_radius() const;
		void set_height(float p_height);
		float get_height() const;
		void set_mid_height(float p_mid_height);
		float get_mid_height() const;
		void set_inside(bool p_enabled);
		bool is_inside() const;

		Vector3 collide(const Vector3& p_point, float p_radius) const override;
		void collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const override;
	};

}

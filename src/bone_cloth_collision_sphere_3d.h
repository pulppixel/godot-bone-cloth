// A sphere on a bone, SpringBoneCollisionSphere3D's: the joints stay out of it, or in it with inside on.
// The push is KawaiiPhysics' sphere collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "bone_cloth_collision_3d.h"


namespace godot {

	class BoneClothCollisionCapsule3D;

	class BoneClothCollisionSphere3D : public BoneClothCollision3D {
		GDCLASS(BoneClothCollisionSphere3D, BoneClothCollision3D)

			friend class BoneClothCollisionCapsule3D;

		float radius = 0.1f;
		bool inside = false;

	protected:
		static void _bind_methods();

		static Vector3 _collide_sphere(const Vector3& p_center, float p_radius, bool p_inside, const Vector3& p_fallback, const Vector3& p_point, float p_point_radius);

	public:
		void set_radius(float p_radius);
		float get_radius() const;
		void set_inside(bool p_enabled);
		bool is_inside() const;

		Vector3 collide(const Vector3& p_point, float p_radius) const override;
		void collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const override;

	};

}

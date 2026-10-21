// bone에 붙은 구.
// The push is KawaiiPhysics' sphere collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionCapsule3D;
class BoneClothCollisionTaperedCapsule3D;

class BoneClothCollisionSphere3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionSphere3D, BoneClothCollision3D)

	friend class BoneClothCollisionCapsule3D;
	friend class BoneClothCollisionTaperedCapsule3D;

	float radius = 0.1f;
	bool inside = false;

protected:
	static void _bind_methods();

	Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const override;
	void _collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const override;
	float _segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const override;

	static Vector3 _collide_sphere(const Vector3 &p_center, float p_radius, bool p_inside, const Vector3 &p_fallback, const Vector3 &p_point, float p_point_radius);

public:
	void set_radius(float p_radius);
	float get_radius() const;
	void set_inside(bool p_enabled);
	bool is_inside() const;
};

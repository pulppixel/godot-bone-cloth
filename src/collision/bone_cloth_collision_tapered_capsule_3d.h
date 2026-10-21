// 양쪽의 굵기가 다른 캡슐(예: 허벅지).
// The push is KawaiiPhysics' tapered capsule collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md,
// with the axis point moved so it collides with the hull itself.

#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionTaperedCapsule3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionTaperedCapsule3D, BoneClothCollision3D)

	float top_radius = 0.1f;
	float bottom_radius = 0.05f;
	float height = 0.5f;
	bool inside = false;

	void _get_head_and_tail(const Transform3D &p_pose, Vector3 &r_head, Vector3 &r_tail) const;
	void _get_nearest_sphere(const Vector3 &p_head, const Vector3 &p_tail, const Vector3 &p_point, Vector3 &r_center, float &r_radius) const;

protected:
	static void _bind_methods();

	Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const override;
	void _collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const override;
	float _segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const override;

public:
	void set_top_radius(float p_radius);
	float get_top_radius() const;
	void set_bottom_radius(float p_radius);
	float get_bottom_radius() const;
	void set_height(float p_height);
	float get_height() const;
	void set_inside(bool p_enabled);
	bool is_inside() const;
};

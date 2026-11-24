// The push is KawaiiPhysics' box collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionBox3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionBox3D, BoneClothCollision3D)

	Vector3 size = Vector3(0.2f, 0.2f, 0.2f);

	float _get_distance(const Transform3D &p_pose, const Vector3 &p_point, Vector3 &r_normal) const;
	float _find_deepest(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, Vector3 &r_point, Vector3 &r_normal) const;

protected:
	static void _bind_methods();

	Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const override;
	void _collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const override;
	float _segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const override;

public:
	void set_size(const Vector3 &p_size);
	Vector3 get_size() const;
};

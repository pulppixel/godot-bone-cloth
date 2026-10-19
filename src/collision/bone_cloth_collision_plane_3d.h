// bone에 붙은 끝없는 평면. 노드의 Y 방향이 정면이다.

#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionPlane3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionPlane3D, BoneClothCollision3D)

protected:
	static void _bind_methods();

	Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const override;
	void _collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const override;
};

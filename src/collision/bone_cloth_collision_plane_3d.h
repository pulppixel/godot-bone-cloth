#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionPlane3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionPlane3D, BoneClothCollision3D)

protected:
	static void _bind_methods();

	Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const override;
};

#include "collision/bone_cloth_collision_plane_3d.h"

using namespace godot;

void BoneClothCollisionPlane3D::_bind_methods() {
}

Vector3 BoneClothCollisionPlane3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	const Vector3 normal = p_pose.basis.get_column(1).normalized();
	const float distance = (p_point - p_pose.origin).dot(normal) - p_radius;
	if (distance >= 0.0f) {
		return p_point;
	}

	return p_point - normal * distance;
}

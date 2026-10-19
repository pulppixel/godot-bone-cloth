#include "collision/bone_cloth_collision_plane_3d.h"

using namespace godot;

void BoneClothCollisionPlane3D::_bind_methods() {
}

// joint가 plane 뒤에 있거나 반지름보다 가까우면, 거리가 반지름이 될 때까지 앞으로 밀어낸다.
// 한 step에 plane을 뚫고 지나간 joint도 다시 앞으로 돌아온다.
Vector3 BoneClothCollisionPlane3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	const Vector3 normal = p_pose.basis.get_column(1).normalized();
	const float distance = (p_point - p_pose.origin).dot(normal) - p_radius;
	if (distance >= 0.0f) {
		return p_point;
	}

	return p_point - normal * distance;
}

// 선분 충돌은 양 끝 joint만 밀어낸다.
void BoneClothCollisionPlane3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	r_a = _collide(p_pose, r_a, p_radius);
	r_b = _collide(p_pose, r_b, p_radius);
}

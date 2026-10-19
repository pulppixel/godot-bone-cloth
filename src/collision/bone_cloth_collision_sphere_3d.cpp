#include "collision/bone_cloth_collision_sphere_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothCollisionSphere3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothCollisionSphere3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothCollisionSphere3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_inside", "enabled"), &BoneClothCollisionSphere3D::set_inside);
	ClassDB::bind_method(D_METHOD("is_inside"), &BoneClothCollisionSphere3D::is_inside);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inside"), "set_inside", "is_inside");
}

void BoneClothCollisionSphere3D::set_radius(float p_radius) {
	radius = p_radius;
	update_gizmos();
}

float BoneClothCollisionSphere3D::get_radius() const {
	return radius;
}

void BoneClothCollisionSphere3D::set_inside(bool p_enabled) {
	inside = p_enabled;
	update_gizmos();
}

bool BoneClothCollisionSphere3D::is_inside() const {
	return inside;
}

// 구 충돌(캡슐과 tapered capsule도 이 함수를 쓴다).
// 밖: 중심에서 "두 반지름의 합"만큼 떨어지게 밀어낸다.
// 안: 중심에서 "구 반지름 - joint 반지름"까지 당겨준다.
Vector3 BoneClothCollisionSphere3D::_collide_sphere(const Vector3 &p_center, float p_radius, bool p_inside, const Vector3 &p_fallback, const Vector3 &p_point, float p_point_radius) {
	const Vector3 offset = p_point - p_center;
	const float distance_squared = offset.length_squared();
	if (p_inside) {
		const float limit = MAX(p_radius - p_point_radius, 0.0f);
		if (distance_squared <= limit * limit) {
			return p_point;
		}
		return p_center + offset / Math::sqrt(distance_squared) * limit;
	}

	const float limit = p_radius + p_point_radius;
	if (distance_squared >= limit * limit) {
		return p_point;
	}

	const Vector3 direction = distance_squared > CMP_EPSILON2 ? offset / Math::sqrt(distance_squared) : p_fallback;
	return p_center + direction * limit;
}

// 밀어낼 방향이 없을 때는 노드의 X 방향으로 밀어낸다.
Vector3 BoneClothCollisionSphere3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	return _collide_sphere(p_pose.origin, radius, inside, p_pose.basis.get_column(0).normalized(), p_point, p_radius);
}

// 선분 충돌. inside면 양 끝 joint만 안으로 당긴다(양 끝이 안에 있으면 선분 전체가 안에 있다).
// 밖이면 선분에서 중심에 가장 가까운 점을 찾아 밀어낸다.
void BoneClothCollisionSphere3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	if (inside) {
		r_a = _collide(p_pose, r_a, p_radius);
		r_b = _collide(p_pose, r_b, p_radius);
		return;
	}

	const Vector3 on_segment = Geometry3D::get_singleton()->get_closest_point_to_segment(p_pose.origin, r_a, r_b);
	_push_segment_out(r_a, r_b, on_segment, p_pose.origin, radius + p_radius, p_pose.basis.get_column(0).normalized());
}

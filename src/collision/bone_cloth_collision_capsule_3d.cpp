#include "collision/bone_cloth_collision_capsule_3d.h"

#include "collision/bone_cloth_collision_sphere_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothCollisionCapsule3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothCollisionCapsule3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothCollisionCapsule3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &BoneClothCollisionCapsule3D::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &BoneClothCollisionCapsule3D::get_height);
	ClassDB::bind_method(D_METHOD("set_mid_height", "mid_height"), &BoneClothCollisionCapsule3D::set_mid_height);
	ClassDB::bind_method(D_METHOD("get_mid_height"), &BoneClothCollisionCapsule3D::get_mid_height);
	ClassDB::bind_method(D_METHOD("set_inside", "enabled"), &BoneClothCollisionCapsule3D::set_inside);
	ClassDB::bind_method(D_METHOD("is_inside"), &BoneClothCollisionCapsule3D::is_inside);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_height", "get_height");
	// 스크립트용이라 인스펙터에는 보이지 않는다 (저장도 하지 않는다).
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mid_height", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", PROPERTY_USAGE_NONE), "set_mid_height", "get_mid_height");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inside"), "set_inside", "is_inside");
}

// height는 반구를 포함한 전체 길이. 반지름이 height의 절반을 넘지 않게 맞춘다.
// (반지름을 키우면 height가 늘어나고, height를 줄이면 반지름이 줄어든다).
void BoneClothCollisionCapsule3D::set_radius(float p_radius) {
	radius = p_radius;
	if (radius > height * 0.5f) {
		height = radius * 2.0f;
	}

	update_gizmos();
}

float BoneClothCollisionCapsule3D::get_radius() const {
	return radius;
}

void BoneClothCollisionCapsule3D::set_height(float p_height) {
	height = p_height;
	if (radius > height * 0.5f) {
		radius = height * 0.5f;
	}

	update_gizmos();
}

float BoneClothCollisionCapsule3D::get_height() const {
	return height;
}

void BoneClothCollisionCapsule3D::set_mid_height(float p_mid_height) {
	ERR_FAIL_COND_MSG(p_mid_height < 0.0f, "BoneClothCollisionCapsule3D mid-height cannot be negative.");
	height = p_mid_height + radius * 2.0f;
	update_gizmos();
}

float BoneClothCollisionCapsule3D::get_mid_height() const {
	return height - radius * 2.0f;
}

void BoneClothCollisionCapsule3D::set_inside(bool p_enabled) {
	inside = p_enabled;
	update_gizmos();
}

bool BoneClothCollisionCapsule3D::is_inside() const {
	return inside;
}

void BoneClothCollisionCapsule3D::_get_head_and_tail(const Transform3D &p_pose, Vector3 &r_head, Vector3 &r_tail) const {
	const Vector3 half_axis = p_pose.basis.xform(Vector3(0.0f, height * 0.5f - radius, 0.0f));
	r_head = p_pose.origin + half_axis;
	r_tail = p_pose.origin - half_axis;
}

// Y축 위에서 joint에 가장 가까운 점에서 구 충돌을 한다(캡슐 = 축을 따라 늘어선 구).
Vector3 BoneClothCollisionCapsule3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	const Vector3 axis = tail - head;
	const float axis_length_squared = axis.length_squared();
	const float t = axis_length_squared > 0.0f ? CLAMP((p_point - head).dot(axis) / axis_length_squared, 0.0f, 1.0f) : 0.0f;
	return BoneClothCollisionSphere3D::_collide_sphere(head + axis * t, radius, inside, p_pose.basis.get_column(0).normalized(), p_point, p_radius);
}

// 선분 충돌. inside면 양 끝 joint만 안으로 당긴다(양 끝이 안에 있으면 선분 전체가 안에 있다).
// 밖이면 선분과 축 사이에서 가장 가까운 두 점을 찾아 밀어낸다.
void BoneClothCollisionCapsule3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	if (inside) {
		r_a = _collide(p_pose, r_a, p_radius);
		r_b = _collide(p_pose, r_b, p_radius);
		return;
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(r_a, r_b, head, tail);
	_push_segment_out(r_a, r_b, closest[0], closest[1], radius + p_radius, p_pose.basis.get_column(0).normalized());
}

// 선분과 축 사이에서 가장 가까운 두 점의 깊이.
float BoneClothCollisionCapsule3D::_segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const {
	if (inside) {
		return BoneClothCollision3D::_segment_depth(p_pose, p_a, p_b, p_radius);
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(p_a, p_b, head, tail);
	return MAX(radius + p_radius - closest[0].distance_to(closest[1]), 0.0f);
}

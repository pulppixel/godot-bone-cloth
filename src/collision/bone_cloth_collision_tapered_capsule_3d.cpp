#include "collision/bone_cloth_collision_tapered_capsule_3d.h"

#include "collision/bone_cloth_collision_sphere_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothCollisionTaperedCapsule3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_top_radius", "radius"), &BoneClothCollisionTaperedCapsule3D::set_top_radius);
	ClassDB::bind_method(D_METHOD("get_top_radius"), &BoneClothCollisionTaperedCapsule3D::get_top_radius);
	ClassDB::bind_method(D_METHOD("set_bottom_radius", "radius"), &BoneClothCollisionTaperedCapsule3D::set_bottom_radius);
	ClassDB::bind_method(D_METHOD("get_bottom_radius"), &BoneClothCollisionTaperedCapsule3D::get_bottom_radius);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &BoneClothCollisionTaperedCapsule3D::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &BoneClothCollisionTaperedCapsule3D::get_height);
	ClassDB::bind_method(D_METHOD("set_inside", "enabled"), &BoneClothCollisionTaperedCapsule3D::set_inside);
	ClassDB::bind_method(D_METHOD("is_inside"), &BoneClothCollisionTaperedCapsule3D::is_inside);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "top_radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_top_radius", "get_top_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bottom_radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_bottom_radius", "get_bottom_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_height", "get_height");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inside"), "set_inside", "is_inside");
}

// height는 캡슐처럼 전체 길이(두 반지름이 같으면 BoneClothCollisionCapsule3D와 같은 모양).
// 반지름이 height의 절반을 넘지 않게 맞춘다.
void BoneClothCollisionTaperedCapsule3D::set_top_radius(float p_radius) {
	top_radius = p_radius;
	if (top_radius > height * 0.5f) {
		height = top_radius * 2.0f;
	}

	update_gizmos();
}

float BoneClothCollisionTaperedCapsule3D::get_top_radius() const {
	return top_radius;
}

void BoneClothCollisionTaperedCapsule3D::set_bottom_radius(float p_radius) {
	bottom_radius = p_radius;
	if (bottom_radius > height * 0.5f) {
		height = bottom_radius * 2.0f;
	}

	update_gizmos();
}

float BoneClothCollisionTaperedCapsule3D::get_bottom_radius() const {
	return bottom_radius;
}

void BoneClothCollisionTaperedCapsule3D::set_height(float p_height) {
	height = p_height;
	if (top_radius > height * 0.5f) {
		top_radius = height * 0.5f;
	}
	if (bottom_radius > height * 0.5f) {
		bottom_radius = height * 0.5f;
	}

	update_gizmos();
}

float BoneClothCollisionTaperedCapsule3D::get_height() const {
	return height;
}

void BoneClothCollisionTaperedCapsule3D::set_inside(bool p_enabled) {
	inside = p_enabled;
	update_gizmos();
}

bool BoneClothCollisionTaperedCapsule3D::is_inside() const {
	return inside;
}

// head는 +Y 쪽(top_radius), tail은 -Y 쪽(bottom_radius).
void BoneClothCollisionTaperedCapsule3D::_get_head_and_tail(const Transform3D &p_pose, Vector3 &r_head, Vector3 &r_tail) const {
	r_head = p_pose.origin + p_pose.basis.xform(Vector3(0.0f, height * 0.5f - top_radius, 0.0f));
	r_tail = p_pose.origin - p_pose.basis.xform(Vector3(0.0f, height * 0.5f - bottom_radius, 0.0f));
}

// joint가 부딪칠 구 하나를 고른다. 이 모양은 굵기가 조금씩 변하는 구를 축을 따라 이어 붙인 것으로 본다.
// 굵기가 다르면 옆면이 기울어서, 그 구는 joint 바로 옆보다 굵은 쪽으로 조금 옮겨져 있다(joint와 축의 거리 * 옆면 기울기만큼).
void BoneClothCollisionTaperedCapsule3D::_get_nearest_sphere(const Vector3 &p_head, const Vector3 &p_tail, const Vector3 &p_point, Vector3 &r_center, float &r_radius) const {
	const Vector3 axis = p_tail - p_head;
	const float length = axis.length();
	const float shrink = top_radius - bottom_radius;

	// height가 최소면 작은 구가 큰 구 안에 들어간다.
	if (length <= Math::abs(shrink)) {
		r_center = shrink >= 0.0f ? p_head : p_tail;
		r_radius = MAX(top_radius, bottom_radius);
		return;
	}

	const Vector3 direction = axis / length;
	const Vector3 offset = p_point - p_head;
	const float along = offset.dot(direction);
	const float across = (offset - direction * along).length();
	const float slope = shrink / Math::sqrt(length * length - shrink * shrink);
	const float t = CLAMP((along - across * slope) / length, 0.0f, 1.0f);
	r_center = p_head + axis * t;
	r_radius = Math::lerp(top_radius, bottom_radius, t);
}

// 선택한 구로 충돌을 한다.
Vector3 BoneClothCollisionTaperedCapsule3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	Vector3 center;
	float radius;
	_get_nearest_sphere(head, tail, p_point, center, radius);
	return BoneClothCollisionSphere3D::_collide_sphere(center, radius, inside, p_pose.basis.get_column(0).normalized(), p_point, p_radius);
}

void BoneClothCollisionTaperedCapsule3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	if (inside) {
		r_a = _collide(p_pose, r_a, p_radius);
		r_b = _collide(p_pose, r_b, p_radius);
		return;
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(r_a, r_b, head, tail);
	Vector3 center;
	float radius;
	_get_nearest_sphere(head, tail, closest[0], center, radius);
	_push_segment_out(r_a, r_b, closest[0], center, radius + p_radius, p_pose.basis.get_column(0).normalized());
}

// 선분과 축 사이에서 가장 가까운 점에 고른 구의 깊이.
float BoneClothCollisionTaperedCapsule3D::_segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const {
	if (inside) {
		return BoneClothCollision3D::_segment_depth(p_pose, p_a, p_b, p_radius);
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(p_pose, head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(p_a, p_b, head, tail);
	Vector3 center;
	float radius;
	_get_nearest_sphere(head, tail, closest[0], center, radius);
	return MAX(radius + p_radius - closest[0].distance_to(center), 0.0f);
}

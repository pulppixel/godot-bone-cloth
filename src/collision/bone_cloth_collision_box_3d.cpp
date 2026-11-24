#include "collision/bone_cloth_collision_box_3d.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

static constexpr int SEGMENT_SEARCH_STEPS = 20;
static constexpr float GOLDEN_SHARE = 0.618034f;

void BoneClothCollisionBox3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &BoneClothCollisionBox3D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &BoneClothCollisionBox3D::get_size);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m"), "set_size", "get_size");
}

void BoneClothCollisionBox3D::set_size(const Vector3 &p_size) {
	ERR_FAIL_COND_MSG(p_size.x < 0.0f || p_size.y < 0.0f || p_size.z < 0.0f, "BoneClothCollisionBox3D size cannot be negative.");
	size = p_size;
	update_gizmos();
}

Vector3 BoneClothCollisionBox3D::get_size() const {
	return size;
}

static float get_box_distance(const Transform3D &p_frame, const Vector3 &p_half, const Vector3 &p_point, Vector3 &r_normal) {
	const Basis &rotation = p_frame.basis;
	const Vector3 local = rotation.xform_inv(p_point - p_frame.origin);
	const Vector3 over = local.abs() - p_half;
	if (over.x > 0.0f || over.y > 0.0f || over.z > 0.0f) {
		const Vector3 outside(SIGN(local.x) * MAX(over.x, 0.0f), SIGN(local.y) * MAX(over.y, 0.0f), SIGN(local.z) * MAX(over.z, 0.0f));
		const float distance = outside.length();
		r_normal = rotation.xform(outside / distance);
		return distance;
	}

	const int axis = (over.x >= over.y && over.x >= over.z) ? 0 : (over.y >= over.z ? 1 : 2);
	Vector3 normal;
	normal[axis] = local[axis] >= 0.0f ? 1.0f : -1.0f;
	r_normal = rotation.xform(normal);
	return over[axis];
}

static Transform3D get_box_frame(const Transform3D &p_pose) {
	return Transform3D(p_pose.basis.orthonormalized(), p_pose.origin);
}

float BoneClothCollisionBox3D::_find_deepest(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, Vector3 &r_point, Vector3 &r_normal) const {
	const Transform3D frame = get_box_frame(p_pose);
	const Vector3 half = size * 0.5f * _get_scale(p_pose.basis);
	float low = 0.0f;
	float high = 1.0f;
	Vector3 normal;
	for (int i = 0; i < SEGMENT_SEARCH_STEPS; i++) {
		const float left = high - (high - low) * GOLDEN_SHARE;
		const float right = low + (high - low) * GOLDEN_SHARE;
		if (get_box_distance(frame, half, p_a.lerp(p_b, left), normal) < get_box_distance(frame, half, p_a.lerp(p_b, right), normal)) {
			high = right;
		} else {
			low = left;
		}
	}

	r_point = p_a.lerp(p_b, (low + high) * 0.5f);
	return get_box_distance(frame, half, r_point, r_normal);
}

Vector3 BoneClothCollisionBox3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	Vector3 normal;
	const float distance = get_box_distance(get_box_frame(p_pose), size * 0.5f * _get_scale(p_pose.basis), p_point, normal);
	if (distance >= p_radius) {
		return p_point;
	}

	return p_point + normal * (p_radius - distance);
}

void BoneClothCollisionBox3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	Vector3 point;
	Vector3 normal;
	const float distance = _find_deepest(p_pose, r_a, r_b, point, normal);
	if (distance >= p_radius) {
		return;
	}

	_push_segment(r_a, r_b, point, normal * (p_radius - distance));
}

float BoneClothCollisionBox3D::_segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const {
	Vector3 point;
	Vector3 normal;
	return MAX(p_radius - _find_deepest(p_pose, p_a, p_b, point, normal), 0.0f);
}

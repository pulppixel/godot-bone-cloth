#include "collision/bone_cloth_collision_sphere_3d.h"

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
	ERR_FAIL_COND_MSG(p_radius < 0.0f, "BoneClothCollisionSphere3D radius cannot be negative.");
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

Vector3 BoneClothCollisionSphere3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	const Vector3 scale = _get_scale(p_pose.basis);
	return _collide_sphere(p_pose.origin, radius * (scale.x + scale.y + scale.z) / 3.0f, inside, p_pose.basis.get_column(0).normalized(), p_point, p_radius);
}

void BoneClothCollisionSphere3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	if (inside) {
		BoneClothCollision3D::_collide_segment(p_pose, r_a, r_b, p_radius);
		return;
	}

	const Vector3 scale = _get_scale(p_pose.basis);
	const Vector3 on_segment = _get_closest_point_to_segment(p_pose.origin, r_a, r_b);
	_push_segment_out(r_a, r_b, on_segment, p_pose.origin, radius * (scale.x + scale.y + scale.z) / 3.0f + p_radius, p_pose.basis.get_column(0).normalized());
}

float BoneClothCollisionSphere3D::_segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const {
	if (inside) {
		return BoneClothCollision3D::_segment_depth(p_pose, p_a, p_b, p_radius);
	}

	const Vector3 scale = _get_scale(p_pose.basis);
	const Vector3 on_segment = _get_closest_point_to_segment(p_pose.origin, p_a, p_b);
	return MAX(radius * (scale.x + scale.y + scale.z) / 3.0f + p_radius - on_segment.distance_to(p_pose.origin), 0.0f);
}

float BoneClothCollisionSphere3D::_get_reach(const Transform3D &p_pose) const {
	if (inside) {
		return -1.0f;
	}
	const Vector3 scale = _get_scale(p_pose.basis);
	return radius * MAX(MAX(scale.x, scale.y), scale.z);
}

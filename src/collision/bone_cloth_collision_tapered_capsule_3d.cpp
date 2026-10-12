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

// Height is end to end, as the capsule's, so equal radii make the same shape as BoneClothCollisionCapsule3D. It never gets shorter than
// the larger end sphere, so neither end sphere holds the other.
void BoneClothCollisionTaperedCapsule3D::set_top_radius(float p_radius) {
	top_radius = p_radius;
	if (top_radius > height * 0.5f) {
		height = top_radius * 2.0f;
	}
}

float BoneClothCollisionTaperedCapsule3D::get_top_radius() const {
	return top_radius;
}

void BoneClothCollisionTaperedCapsule3D::set_bottom_radius(float p_radius) {
	bottom_radius = p_radius;
	if (bottom_radius > height * 0.5f) {
		height = bottom_radius * 2.0f;
	}
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
}

float BoneClothCollisionTaperedCapsule3D::get_height() const {
	return height;
}

void BoneClothCollisionTaperedCapsule3D::set_inside(bool p_enabled) {
	inside = p_enabled;
}

bool BoneClothCollisionTaperedCapsule3D::is_inside() const {
	return inside;
}

// The centres of the two end spheres in skeleton space, head at +Y with top_radius.
void BoneClothCollisionTaperedCapsule3D::_get_head_and_tail(Vector3 &r_head, Vector3 &r_tail) const {
	r_head = pose.origin + pose.basis.xform(Vector3(0.0f, height * 0.5f - top_radius, 0.0f));
	r_tail = pose.origin - pose.basis.xform(Vector3(0.0f, height * 0.5f - bottom_radius, 0.0f));
}

// The hull is the union of spheres along the axis whose radius goes linearly from one end to the other; the one whose surface is
// nearest the point collides. KawaiiPhysics' AdjustByTaperedCapsuleCollision takes the sphere at the axis point nearest the point,
// which is that sphere only when the radii are equal. On a taper the hull's side touches each sphere off its middle, toward the thin
// end, so the nearest sphere sits back toward the thick end by the point's distance from the axis times the tangent of the side's slope.
// On Lilith's thigh (8.4 to 5.2 cm over 0.458 m) KawaiiPhysics' shape is 0.2 mm inside the hull at most; on 10 to 3 cm over 0.15 m, 10 mm
void BoneClothCollisionTaperedCapsule3D::_get_nearest_sphere(const Vector3 &p_head, const Vector3 &p_tail, const Vector3 &p_point, Vector3 &r_center, float &r_radius) const {
	const Vector3 axis = p_tail - p_head;
	const float length = axis.length();
	const float shrink = top_radius - bottom_radius;

	// At the least height the larger end sphere holds the other and is the whole shape.
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

// The sphere collision against the nearest sphere. A point on the axis has no direction out and takes the node's X.
// Inside, the room is the hull of the end spheres less the joint's radius, each never below zero, as the sphere's.
Vector3 BoneClothCollisionTaperedCapsule3D::collide(const Vector3 &p_point, float p_radius) const {
	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(head, tail);
	Vector3 center;
	float radius;
	_get_nearest_sphere(head, tail, p_point, center, radius);
	return BoneClothCollisionSphere3D::_collide_sphere(center, radius, inside, pose.basis.get_column(0).normalized(), p_point, p_radius);
}

// Outside, the line's point nearest the axis against the sphere nearest that point, as the capsule's. Inside, the room is convex,
// so the line stays in when both of its ends do.
void BoneClothCollisionTaperedCapsule3D::collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	if (inside) {
		r_a = collide(r_a, p_radius);
		r_b = collide(r_b, p_radius);
		return;
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(r_a, r_b, head, tail);
	Vector3 center;
	float radius;
	_get_nearest_sphere(head, tail, closest[0], center, radius);
	_push_segment_out(r_a, r_b, closest[0], center, radius + p_radius, pose.basis.get_column(0).normalized());
}

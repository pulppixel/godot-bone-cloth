#include "bone_cloth_collision_capsule_3d.h"
#include "bone_cloth_collision_sphere_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>


using namespace godot;

void BoneClothCollisionCapsule3D::_bind_methods()
{
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
	// The straight part alone, for scripts; not shown and not saved, as the engine's.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mid_height", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", PROPERTY_USAGE_NONE), "set_mid_height", "get_mid_height");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inside"), "set_inside", "is_inside");
}

// Height is end to end, hemispheres included, as CapsuleShape3D and SpringBoneCollisionCapsule3D; it never gets shorter than the two hemispheres.
void BoneClothCollisionCapsule3D::set_radius(float p_radius)
{
	radius = p_radius;
	if (radius > height * 0.5f) {
		height = radius * 2.0f;
	}
}

float BoneClothCollisionCapsule3D::get_radius() const
{
	return radius;
}

void BoneClothCollisionCapsule3D::set_height(float p_height)
{
	height = p_height;
	if (radius > height * 0.5f) {
		radius = height * 0.5f;
	}
}

float BoneClothCollisionCapsule3D::get_height() const
{
	return height;
}

void BoneClothCollisionCapsule3D::set_mid_height(float p_mid_height)
{
	ERR_FAIL_COND_MSG(p_mid_height < 0.0f, "BoneClothCollisionCapsule3D mid-height cannot be negative.");
	height = p_mid_height + radius * 2.0f;
}

float BoneClothCollisionCapsule3D::get_mid_height() const
{
	return height - radius * 2.0f;
}

void BoneClothCollisionCapsule3D::set_inside(bool p_enabled)
{
	inside = p_enabled;
}

bool BoneClothCollisionCapsule3D::is_inside() const
{
	return inside;
}

// The two ends of the axis, the hemispheres' centres, in skeleton space.
void BoneClothCollisionCapsule3D::_get_head_and_tail(Vector3& r_head, Vector3& r_tail) const
{
	const Vector3 half_axis = pose.basis.xform(Vector3(0.0f, height * 0.5f - radius, 0.0f));
	r_head = pose.origin + half_axis;
	r_tail = pose.origin - half_axis;
}

// KawaiiPhysics' AdjustByCapsuleCollision: the sphere collision at the point of the axis nearest the joint.
// A point on the axis has no direction out and takes the capsule's X.
Vector3 BoneClothCollisionCapsule3D::collide(const Vector3& p_point, float p_radius) const
{
	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(head, tail);
	const Vector3 axis = tail - head;
	const float axis_length_squared = axis.length_squared();
	const float t = axis_length_squared > 0.0f ? CLAMP((p_point - head).dot(axis) / axis_length_squared, 0.0f, 1.0f) : 0.0f;
	return BoneClothCollisionSphere3D::_collide_sphere(head + axis * t, radius, inside, pose.basis.get_column(0).normalized(), p_point, p_radius);
}

// Inside, the room left for the line is a thinner capsule and convex, so the line stays in when both of its ends do.
void BoneClothCollisionCapsule3D::collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const
{
	if (inside) {
		r_a = collide(r_a, p_radius);
		r_b = collide(r_b, p_radius);
		return;
	}

	Vector3 head;
	Vector3 tail;
	_get_head_and_tail(head, tail);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(r_a, r_b, head, tail);
	_push_segment_out(r_a, r_b, closest[0], closest[1], radius + p_radius, pose.basis.get_column(0).normalized());
}

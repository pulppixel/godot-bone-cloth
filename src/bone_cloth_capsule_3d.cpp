#include "bone_cloth_capsule_3d.h"
#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>


using namespace godot;

void BoneClothCapsule3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_skeleton"), &BoneClothCapsule3D::get_skeleton);
	ClassDB::bind_method(D_METHOD("set_bone_name", "name"), &BoneClothCapsule3D::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &BoneClothCapsule3D::get_bone_name);
	ClassDB::bind_method(D_METHOD("set_position_offset", "offset"), &BoneClothCapsule3D::set_position_offset);
	ClassDB::bind_method(D_METHOD("get_position_offset"), &BoneClothCapsule3D::get_position_offset);
	ClassDB::bind_method(D_METHOD("set_rotation_offset", "offset"), &BoneClothCapsule3D::set_rotation_offset);
	ClassDB::bind_method(D_METHOD("get_rotation_offset"), &BoneClothCapsule3D::get_rotation_offset);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothCapsule3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothCapsule3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &BoneClothCapsule3D::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &BoneClothCapsule3D::get_height);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "bone_name"), "set_bone_name", "get_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_height", "get_height");

	ADD_GROUP("Offset", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position_offset", PROPERTY_HINT_NONE, "suffix:m"), "set_position_offset", "get_position_offset");
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "rotation_offset"), "set_rotation_offset", "get_rotation_offset");
}

// The bone name as a drop-down of the skeleton's bones with a pencil for typing, as SpringBoneCollision3D does.
void BoneClothCapsule3D::_validate_property(PropertyInfo& p_property) const
{
	if (p_property.name != StringName("bone_name")) {
		return;
	}

	Skeleton3D* skeleton = get_skeleton();
	if (skeleton) {
		p_property.hint = PROPERTY_HINT_ENUM_SUGGESTION;
		p_property.hint_string = skeleton->get_concatenated_bone_names();
	}
}

PackedStringArray BoneClothCapsule3D::_get_configuration_warnings() const
{
	PackedStringArray warnings;
	if (!Object::cast_to<BoneClothSimulator3D>(get_parent())) {
		warnings.push_back("Parent node should be a BoneClothSimulator3D node.");
	}

	return warnings;
}

Skeleton3D* BoneClothCapsule3D::get_skeleton() const
{
	BoneClothSimulator3D* parent = Object::cast_to<BoneClothSimulator3D>(get_parent());
	return parent ? parent->get_skeleton() : nullptr;
}


void BoneClothCapsule3D::set_bone_name(const String& p_name)
{
	bone_name = p_name;
}

String BoneClothCapsule3D::get_bone_name() const
{
	return bone_name;
}

void BoneClothCapsule3D::set_position_offset(const Vector3& p_offset)
{
	position_offset = p_offset;
}

Vector3 BoneClothCapsule3D::get_position_offset() const
{
	return position_offset;
}

void BoneClothCapsule3D::set_rotation_offset(const Quaternion& p_offset)
{
	rotation_offset = p_offset;
}

Quaternion BoneClothCapsule3D::get_rotation_offset() const
{
	return rotation_offset;
}

// Height is end to end, hemispheres included, as CapsuleShape3D and SpringBoneCollisionCapsule3D; it never gets shorter than the two hemispheres.
void BoneClothCapsule3D::set_radius(float p_radius)
{
	radius = p_radius;
	if (height < radius * 2.0f) {
		height = radius * 2.0f;
	}
}

float BoneClothCapsule3D::get_radius() const
{
	return radius;
}

void BoneClothCapsule3D::set_height(float p_height)
{
	height = p_height;
	if (radius > height * 0.5f) {
		radius = height * 0.5f;
	}
}

float BoneClothCapsule3D::get_height() const
{
	return height;
}

// Moves the node to its bone's pose this frame plus the offsets, as SpringBoneCollision3D::sync_pose does, so the editor shows it where it collides.
// Without a bone the node stays where it was put. The axis is the node's Y.
void BoneClothCapsule3D::sync_pose()
{
	Skeleton3D* skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D skeleton_global = skeleton->get_global_transform();
	const int bone = skeleton->find_bone(bone_name);
	Transform3D local;
	if (bone >= 0) {
		local = skeleton->get_bone_global_pose(bone);
		local.origin += local.basis.get_rotation_quaternion().xform(position_offset);
		local.basis *= Basis(rotation_offset);
		set_global_transform(skeleton_global * local);
	}
	else {
		local = skeleton_global.affine_inverse() * get_global_transform();
	}

	const Vector3 half_axis = local.basis.xform(Vector3(0.0f, height * 0.5f - radius, 0.0f));
	head = local.origin + half_axis;
	tail = local.origin - half_axis;
	fallback_direction = local.basis.get_column(0).normalized();
}

// KawaiiPhysics' AdjustByCapsuleCollision: a point nearer the axis than the two radii goes out to that distance from the nearest point on the axis.
// A point on the axis has no direction out and takes the capsule's X.
Vector3 BoneClothCapsule3D::collide(const Vector3& p_point, float p_radius) const
{
	const float limit = radius + p_radius;
	const Vector3 axis = tail - head;
	const float axis_length_squared = axis.length_squared();
	const float t = axis_length_squared > 0.0f ? CLAMP((p_point - head).dot(axis) / axis_length_squared, 0.0f, 1.0f) : 0.0f;
	const Vector3 closest = head + axis * t;

	const Vector3 offset = p_point - closest;
	const float distance_squared = offset.length_squared();
	if (distance_squared >= limit * limit) {
		return p_point;
	}

	const Vector3 direction = distance_squared > CMP_EPSILON2 ? offset / Math::sqrt(distance_squared) : fallback_direction;
	return closest + direction * limit;
}

// The line from r_a to r_b, thick by p_radius, out of the capsule, so a leg cannot pass between two joints that it does not touch.
// The push at the line's nearest point is split between the two ends by how near that point is to each, scaled so the nearest point
// moves by exactly the push: the usual position-based split, as Magica Cloth 2's edge collision does.
void BoneClothCapsule3D::collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const
{
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(r_a, r_b, head, tail);
	const Vector3 offset = closest[0] - closest[1];
	const float limit = radius + p_radius;
	const float distance_squared = offset.length_squared();
	if (distance_squared >= limit * limit) {
		return;
	}

	const float distance = Math::sqrt(distance_squared);
	const Vector3 direction = distance > CMP_EPSILON ? offset / distance : fallback_direction;
	const Vector3 push = direction * (limit - distance);

	const Vector3 line = r_b - r_a;
	const float line_length_squared = line.length_squared();
	const float s = line_length_squared > 0.0f ? (closest[0] - r_a).dot(line) / line_length_squared : 0.0f;
	const float scale = 1.0f / ((1.0f - s) * (1.0f - s) + s * s);
	r_a += push * ((1.0f - s) * scale);
	r_b += push * (s * scale);
}

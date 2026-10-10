#include "bone_cloth_collision_3d.h"
#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>


using namespace godot;

void BoneClothCollision3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_skeleton"), &BoneClothCollision3D::get_skeleton);

	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &BoneClothCollision3D::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &BoneClothCollision3D::get_bone_name);
	ClassDB::bind_method(D_METHOD("set_bone", "bone"), &BoneClothCollision3D::set_bone);
	ClassDB::bind_method(D_METHOD("get_bone"), &BoneClothCollision3D::get_bone);

	ClassDB::bind_method(D_METHOD("set_position_offset", "offset"), &BoneClothCollision3D::set_position_offset);
	ClassDB::bind_method(D_METHOD("get_position_offset"), &BoneClothCollision3D::get_position_offset);
	ClassDB::bind_method(D_METHOD("set_rotation_offset", "offset"), &BoneClothCollision3D::set_rotation_offset);
	ClassDB::bind_method(D_METHOD("get_rotation_offset"), &BoneClothCollision3D::get_rotation_offset);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "bone_name"), "set_bone_name", "get_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_bone", "get_bone");

	ADD_GROUP("Offset", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position_offset", PROPERTY_HINT_NONE, "suffix:m"), "set_position_offset", "get_position_offset");
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "rotation_offset"), "set_rotation_offset", "get_rotation_offset");
}

// In the editor the bone name is a drop-down of the skeleton's bones with a pencil for typing; the offsets show only with a bone,
// as SpringBoneCollision3D::_validate_property.
void BoneClothCollision3D::_validate_property(PropertyInfo& p_property) const
{
	if (Engine::get_singleton()->is_editor_hint() && p_property.name == StringName("bone_name")) {
		Skeleton3D* skeleton = get_skeleton();
		if (skeleton) {
			p_property.hint = PROPERTY_HINT_ENUM_SUGGESTION;
			p_property.hint_string = skeleton->get_concatenated_bone_names();
		}
		else {
			p_property.hint = PROPERTY_HINT_NONE;
			p_property.hint_string = "";
		}
	}
	else if (bone < 0 && (p_property.name == StringName("position_offset") || p_property.name == StringName("rotation_offset"))) {
		p_property.usage = PROPERTY_USAGE_NONE;
	}
}

// The node finds its skeleton through the simulator only once it is under it; then the name wins and the saved index is the fallback,
// as SpringBoneCollision3D::_validate_bone_name.
void BoneClothCollision3D::_notification(int p_what)
{
	switch (p_what) {
	case NOTIFICATION_ENTER_TREE:
	case NOTIFICATION_PARENTED: {
		_validate_bone_name();
	} break;
	}
}

// The name is copied first: godot-cpp's String assignment has no self check, and set_bone_name(bone_name) would empty it.
void BoneClothCollision3D::_validate_bone_name()
{
	const String name = bone_name;
	if (!name.is_empty()) {
		set_bone_name(name);
	}
	else if (bone != -1) {
		set_bone(bone);
	}
}

PackedStringArray BoneClothCollision3D::_get_configuration_warnings() const
{
	PackedStringArray warnings;
	if (!Object::cast_to<BoneClothSimulator3D>(get_parent())) {
		warnings.push_back("Parent node should be a BoneClothSimulator3D node.");
	}

	return warnings;
}

Skeleton3D* BoneClothCollision3D::get_skeleton() const
{
	BoneClothSimulator3D* parent = Object::cast_to<BoneClothSimulator3D>(get_parent());
	return parent ? parent->get_skeleton() : nullptr;
}


// A name not on the skeleton warns, where the engine's says "index '-1' is out of range"; an empty name is no bone, and the node stays where it is put.
void BoneClothCollision3D::set_bone_name(const String& p_bone_name)
{
	bone_name = p_bone_name;
	Skeleton3D* skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(bone_name);
		if (found < 0 && !bone_name.is_empty()) {
			WARN_PRINT_ED("Bone not found: '" + bone_name + "'.");
		}
		set_bone(found);
	}
}

String BoneClothCollision3D::get_bone_name() const
{
	return bone_name;
}

// -1 is no bone and keeps the name; an index past the skeleton's bones warns, as the engine's does.
void BoneClothCollision3D::set_bone(int p_bone)
{
	bone = p_bone;
	Skeleton3D* skeleton = get_skeleton();
	if (skeleton && bone >= 0) {
		if (bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Bone index " + itos(bone) + " is out of range.");
			bone = -1;
		}
		else {
			bone_name = skeleton->get_bone_name(bone);
		}
	}

	notify_property_list_changed();
}

int BoneClothCollision3D::get_bone() const
{
	return bone;
}

void BoneClothCollision3D::set_position_offset(const Vector3& p_offset)
{
	if (position_offset == p_offset) {
		return;
	}
	position_offset = p_offset;
	sync_pose();
}

Vector3 BoneClothCollision3D::get_position_offset() const
{
	return position_offset;
}

void BoneClothCollision3D::set_rotation_offset(const Quaternion& p_offset)
{
	if (rotation_offset == p_offset) {
		return;
	}
	rotation_offset = p_offset;
	sync_pose();
}

Quaternion BoneClothCollision3D::get_rotation_offset() const
{
	return rotation_offset;
}

// Moves the node to its bone's pose this frame plus the offsets, as SpringBoneCollision3D::sync_pose does, so the editor shows it where it collides.
// Without a bone the node stays where it was put.
void BoneClothCollision3D::sync_pose()
{
	Skeleton3D* skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D skeleton_global = skeleton->get_global_transform();
	if (bone >= 0) {
		pose = skeleton->get_bone_global_pose(bone);
		pose.origin += pose.basis.get_rotation_quaternion().xform(position_offset);
		pose.basis *= Basis(rotation_offset);
		set_global_transform(skeleton_global * pose);
	}
	else {
		pose = skeleton_global.affine_inverse() * get_global_transform();
	}
}

// The base collides with nothing. It is registered as a virtual class, as the engine's, so the editor offers only the three shapes.
Vector3 BoneClothCollision3D::collide(const Vector3& p_point, float p_radius) const
{
	return p_point;
}

void BoneClothCollision3D::collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const
{
}

// The line from r_a to r_b out of a shape, so a leg cannot pass between two joints that it does not touch. p_on_segment is the line's point
// nearest the shape and p_on_shape the shape's centre or axis point nearest it, which keep p_limit apart. The push at that point is split
// between the two ends by how near it is to each, scaled so the point moves by exactly the push: the usual position-based split,
// as Magica Cloth 2's edge collision does. A line through the centre has no direction out and takes p_fallback.
void BoneClothCollision3D::_push_segment_out(Vector3& r_a, Vector3& r_b, const Vector3& p_on_segment, const Vector3& p_on_shape, float p_limit, const Vector3& p_fallback)
{
	const Vector3 offset = p_on_segment - p_on_shape;
	const float distance_squared = offset.length_squared();
	if (distance_squared >= p_limit * p_limit) {
		return;
	}

	const float distance = Math::sqrt(distance_squared);
	const Vector3 direction = distance > CMP_EPSILON ? offset / distance : p_fallback;
	const Vector3 push = direction * (p_limit - distance);

	const Vector3 line = r_b - r_a;
	const float line_length_squared = line.length_squared();
	const float s = line_length_squared > 0.0f ? (p_on_segment - r_a).dot(line) / line_length_squared : 0.0f;
	const float scale = 1.0f / ((1.0f - s) * (1.0f - s) + s * s);
	r_a += push * ((1.0f - s) * scale);
	r_b += push * (s * scale);
}

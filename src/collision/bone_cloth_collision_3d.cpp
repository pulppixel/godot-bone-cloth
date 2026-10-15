#include "collision/bone_cloth_collision_3d.h"

#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothCollision3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_skeleton"), &BoneClothCollision3D::get_skeleton);

	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &BoneClothCollision3D::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &BoneClothCollision3D::get_bone_name);
	ClassDB::bind_method(D_METHOD("set_bone", "bone"), &BoneClothCollision3D::set_bone);
	ClassDB::bind_method(D_METHOD("get_bone"), &BoneClothCollision3D::get_bone);
	ClassDB::bind_method(D_METHOD("set_mirror_bone_name", "bone_name"), &BoneClothCollision3D::set_mirror_bone_name);
	ClassDB::bind_method(D_METHOD("get_mirror_bone_name"), &BoneClothCollision3D::get_mirror_bone_name);
	ClassDB::bind_method(D_METHOD("set_mirror_bone", "bone"), &BoneClothCollision3D::set_mirror_bone);
	ClassDB::bind_method(D_METHOD("get_mirror_bone"), &BoneClothCollision3D::get_mirror_bone);

	ClassDB::bind_method(D_METHOD("set_position_offset", "offset"), &BoneClothCollision3D::set_position_offset);
	ClassDB::bind_method(D_METHOD("get_position_offset"), &BoneClothCollision3D::get_position_offset);
	ClassDB::bind_method(D_METHOD("set_rotation_offset", "offset"), &BoneClothCollision3D::set_rotation_offset);
	ClassDB::bind_method(D_METHOD("get_rotation_offset"), &BoneClothCollision3D::get_rotation_offset);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "bone_name"), "set_bone_name", "get_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_bone", "get_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "mirror_bone_name"), "set_mirror_bone_name", "get_mirror_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mirror_bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_mirror_bone", "get_mirror_bone");

	ADD_GROUP("Offset", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position_offset", PROPERTY_HINT_NONE, "suffix:m"), "set_position_offset", "get_position_offset");
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "rotation_offset"), "set_rotation_offset", "get_rotation_offset");
}

// In the editor the two bone names are drop-downs of the skeleton's bones with a pencil for typing; the offsets show only with a bone,
// as SpringBoneCollision3D::_validate_property, and so does the mirror bone, whose offsets are those. The mirror bone is still saved
// without one, as Camera3D's fov outside perspective: ClassDB records a default only for a property a new node stores or shows
// (class_get_default_property_value), and hidden from both it would be saved empty on every collision.
void BoneClothCollision3D::_validate_property(PropertyInfo &p_property) const {
	if (Engine::get_singleton()->is_editor_hint() && (p_property.name == StringName("bone_name") || p_property.name == StringName("mirror_bone_name"))) {
		Skeleton3D *skeleton = get_skeleton();
		if (skeleton) {
			p_property.hint = PROPERTY_HINT_ENUM_SUGGESTION;
			p_property.hint_string = skeleton->get_concatenated_bone_names();
		} else {
			p_property.hint = PROPERTY_HINT_NONE;
			p_property.hint_string = "";
		}
	}
	if (bone < 0 && (p_property.name == StringName("position_offset") || p_property.name == StringName("rotation_offset"))) {
		p_property.usage = PROPERTY_USAGE_NONE;
	} else if (bone < 0 && p_property.name == StringName("mirror_bone_name")) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

// The node finds its skeleton through the simulator only once it is under it; then the name wins and the saved index is the fallback,
// as SpringBoneCollision3D::_validate_bone_name.
void BoneClothCollision3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
		case NOTIFICATION_PARENTED: {
			_validate_bone_name();
		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			_take_offsets_from_transform();
		} break;
	}
}

// The names are copied first: godot-cpp's String assignment has no self check, and set_bone_name(bone_name) would empty it.
void BoneClothCollision3D::_validate_bone_name() {
	const String name = bone_name;
	if (!name.is_empty()) {
		set_bone_name(name);
	} else if (bone != -1) {
		set_bone(bone);
	}

	const String mirror_name = mirror_bone_name;
	if (!mirror_name.is_empty()) {
		set_mirror_bone_name(mirror_name);
	} else if (mirror_bone != -1) {
		set_mirror_bone(mirror_bone);
	}
}

PackedStringArray BoneClothCollision3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	if (!Object::cast_to<BoneClothSimulator3D>(get_parent())) {
		warnings.push_back("Parent node should be a BoneClothSimulator3D node.");
	}

	return warnings;
}

Skeleton3D *BoneClothCollision3D::get_skeleton() const {
	BoneClothSimulator3D *parent = Object::cast_to<BoneClothSimulator3D>(get_parent());
	return parent ? parent->get_skeleton() : nullptr;
}

// A name not on the skeleton warns, where the engine's says "index '-1' is out of range"; an empty name is no bone, and the node stays where it is put.
void BoneClothCollision3D::set_bone_name(const String &p_bone_name) {
	bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(bone_name);
		if (found < 0 && !bone_name.is_empty()) {
			WARN_PRINT_ED("Bone not found: '" + bone_name + "'.");
		}
		set_bone(found);
	}
}

String BoneClothCollision3D::get_bone_name() const {
	return bone_name;
}

// -1 is no bone and keeps the name; an index past the skeleton's bones warns, as the engine's does.
void BoneClothCollision3D::set_bone(int p_bone) {
	bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && bone >= 0) {
		if (bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Bone index " + itos(bone) + " is out of range.");
			bone = -1;
		} else {
			bone_name = skeleton->get_bone_name(bone);
		}
	}

	notify_property_list_changed();
}

int BoneClothCollision3D::get_bone() const {
	return bone;
}

// The bone on the other side, picked from the skeleton's bones as the bone is, so any naming works; Magica Cloth 2 also takes its
// target as a Transform. The bone itself is allowed: it mirrors the shape across the skeleton's middle on that bone, as Magica's
// Hips to Hips. A name not on the skeleton warns, as the bone's.
void BoneClothCollision3D::set_mirror_bone_name(const String &p_bone_name) {
	mirror_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(mirror_bone_name);
		if (found < 0 && !mirror_bone_name.is_empty()) {
			WARN_PRINT_ED("Mirror bone not found: '" + mirror_bone_name + "'.");
		}
		set_mirror_bone(found);
	}
}

String BoneClothCollision3D::get_mirror_bone_name() const {
	return mirror_bone_name;
}

// -1 is no mirror and keeps the name, as the bone's.
void BoneClothCollision3D::set_mirror_bone(int p_bone) {
	mirror_bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && mirror_bone >= 0) {
		if (mirror_bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Mirror bone index " + itos(mirror_bone) + " is out of range.");
			mirror_bone = -1;
		} else {
			mirror_bone_name = skeleton->get_bone_name(mirror_bone);
		}
	}

	sync_pose();
	update_gizmos();
}

int BoneClothCollision3D::get_mirror_bone() const {
	return mirror_bone;
}

bool BoneClothCollision3D::has_mirror() const {
	return bone >= 0 && mirror_bone >= 0;
}

void BoneClothCollision3D::set_position_offset(const Vector3 &p_offset) {
	if (position_offset == p_offset) {
		return;
	}
	position_offset = p_offset;
	sync_pose();
	update_gizmos();
}

Vector3 BoneClothCollision3D::get_position_offset() const {
	return position_offset;
}

void BoneClothCollision3D::set_rotation_offset(const Quaternion &p_offset) {
	if (rotation_offset == p_offset) {
		return;
	}
	rotation_offset = p_offset;
	sync_pose();
	update_gizmos();
}

Quaternion BoneClothCollision3D::get_rotation_offset() const {
	return rotation_offset;
}

// A bone's pose this frame plus offsets along its axes, in skeleton space.
static Transform3D get_pose_on_bone(const Skeleton3D *p_skeleton, int p_bone, const Vector3 &p_position_offset, const Quaternion &p_rotation_offset) {
	Transform3D result = p_skeleton->get_bone_global_pose(p_bone);
	result.origin += result.basis.get_rotation_quaternion().xform(p_position_offset);
	result.basis *= Basis(p_rotation_offset);
	return result;
}

// Moves the node to its bone's pose this frame plus the offsets, as SpringBoneCollision3D::sync_pose does, so the editor shows it where it collides.
// Without a bone the node stays where it was put.
// The mirror is KawaiiPhysics' (MirrorOffsetLocation, MirrorOffsetRotation): the offsets in skeleton space at rest, mirrored on the
// skeleton's X, then taken along the mirror bone's rest axes, so the bones' own axes need not mirror each other. X is left and right
// in Godot's humanoid profile and KawaiiPhysics' default. The mirrored rotation is a rotation, not a reflection: the shape's Y, its
// axis, is the mirror of the node's, and its X, the fallback for a point on the axis, points opposite to the mirror of the node's.
// The gizmo draws the mirror relative to the node, which changes with the pose, so it is redrawn each time.
void BoneClothCollision3D::sync_pose() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D skeleton_global = skeleton->get_global_transform();
	if (bone >= 0) {
		pose = get_pose_on_bone(skeleton, bone, position_offset, rotation_offset);
		syncing = true;
		set_global_transform(skeleton_global * pose);
		syncing = false;
	} else {
		pose = skeleton_global.affine_inverse() * get_global_transform();
	}

	if (!has_mirror()) {
		return;
	}
	const Quaternion rest = skeleton->get_bone_global_rest(bone).basis.get_rotation_quaternion();
	const Quaternion mirror_rest = skeleton->get_bone_global_rest(mirror_bone).basis.get_rotation_quaternion();
	const Vector3 position = rest.xform(position_offset);
	const Quaternion rotation = rest * rotation_offset;
	const Vector3 mirrored_position(-position.x, position.y, position.z);
	const Quaternion mirrored_rotation(rotation.x, -rotation.y, -rotation.z, rotation.w);
	mirror_pose = get_pose_on_bone(skeleton, mirror_bone, mirror_rest.inverse().xform(mirrored_position), mirror_rest.inverse() * mirrored_rotation);
	update_gizmos();
}

// A node on a bone moved by anything but sync_pose, the editor's move and rotate tools and their undo among them, keeps its new place
// as the offsets from the bone, as BoneAttachment3D's override_pose writes a moved attachment back into its bone and KawaiiPhysics
// writes its widget's drags into a collision's offsets (DoTranslation, DoRotation). SpringBoneCollision3D does not, and the next
// sync puts the node back. A scale is dropped: the shape's size is its own properties.
// Before the node is in the tree, a scene being loaded sets its saved transform, and the saved offsets win.
void BoneClothCollision3D::_take_offsets_from_transform() {
	if (syncing || bone < 0 || !is_inside_tree()) {
		return;
	}
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D bone_pose = skeleton->get_bone_global_pose(bone);
	const Transform3D moved = skeleton->get_global_transform().affine_inverse() * get_global_transform();
	position_offset = bone_pose.basis.get_rotation_quaternion().inverse().xform(moved.origin - bone_pose.origin);
	rotation_offset = (bone_pose.basis.inverse() * moved.basis).get_rotation_quaternion();
	sync_pose();
}

// The shape at the node's pose, or with p_mirror at the mirror's.
Vector3 BoneClothCollision3D::collide(const Vector3 &p_point, float p_radius, bool p_mirror) const {
	return _collide(p_mirror ? mirror_pose : pose, p_point, p_radius);
}

void BoneClothCollision3D::collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius, bool p_mirror) const {
	_collide_segment(p_mirror ? mirror_pose : pose, r_a, r_b, p_radius);
}

// The base collides with nothing. It is registered as a virtual class, as the engine's, so the editor offers only the three shapes.
Vector3 BoneClothCollision3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	return p_point;
}

void BoneClothCollision3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
}

// The line from r_a to r_b out of a shape, so a leg cannot pass between two joints that it does not touch. p_on_segment is the line's point
// nearest the shape and p_on_shape the shape's center or axis point nearest it, which keep p_limit apart. The push at that point is split
// between the two ends by how near it is to each, scaled so the point moves by exactly the push: the usual position-based split,
// as Magica Cloth 2's edge collision does. A line through the center has no direction out and takes p_fallback.
void BoneClothCollision3D::_push_segment_out(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_on_segment, const Vector3 &p_on_shape, float p_limit, const Vector3 &p_fallback) {
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

BoneClothCollision3D::BoneClothCollision3D() {
	set_notify_local_transform(true);
}

// A shape on a bone that the cloth joints collide with, the base of BoneClothSimulator3D's sphere, capsule and plane children,
// as SpringBoneCollision3D is of SpringBoneSimulator3D: a bone name with its hidden index and two offsets from the bone.
// A mirror bone makes the same shape collide a second time on that bone, mirrored, as Magica Cloth 2's Symmetry Mode and KawaiiPhysics'
// MirrorDataTable do, so one node covers both legs.

#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>

using namespace godot;

class BoneClothCollision3D : public Node3D {
	GDCLASS(BoneClothCollision3D, Node3D)

	friend class BoneClothCollision3DGizmoPlugin;

	String bone_name;
	int bone = -1;
	String mirror_bone_name;
	int mirror_bone = -1;

	Vector3 position_offset;
	Quaternion rotation_offset;

	// Set while sync_pose moves the node, so the move is not read back as the user's.
	bool syncing = false;

	void _validate_bone_name();
	void _take_offsets_from_transform();

protected:
	// The node's transform in skeleton space from the last sync_pose, where the shapes collide.
	Transform3D pose;
	// The pose on mirror_bone from the last sync_pose, while the node has a bone and a mirror bone.
	Transform3D mirror_pose;

	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;
	void _notification(int p_what);

	static void _push_segment_out(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_on_segment, const Vector3 &p_on_shape, float p_limit, const Vector3 &p_fallback);

	virtual Vector3 _collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const;
	virtual void _collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const;

public:
	PackedStringArray _get_configuration_warnings() const override;

	Skeleton3D *get_skeleton() const;

	void set_bone_name(const String &p_bone_name);
	String get_bone_name() const;
	void set_bone(int p_bone);
	int get_bone() const;
	void set_mirror_bone_name(const String &p_bone_name);
	String get_mirror_bone_name() const;
	void set_mirror_bone(int p_bone);
	int get_mirror_bone() const;
	bool has_mirror() const;

	void set_position_offset(const Vector3 &p_offset);
	Vector3 get_position_offset() const;
	void set_rotation_offset(const Quaternion &p_offset);
	Quaternion get_rotation_offset() const;

	void sync_pose();
	Vector3 collide(const Vector3 &p_point, float p_radius, bool p_mirror) const;
	void collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius, bool p_mirror) const;

	BoneClothCollision3D();
};

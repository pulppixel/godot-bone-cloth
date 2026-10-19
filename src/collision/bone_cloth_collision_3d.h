// bone에 붙어 천의 joint를 밀어내는 Collision.
// mirror bone을 정하면 그 bone에도 좌우 반전된 모양으로 생긴다.

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

	// sync_pose가 노드를 옮기는 중이면 true.
	bool syncing = false;

	void _validate_bone_name();
	void _take_offsets_from_transform();

protected:
	// 충돌에 쓰는 위치와 회전(skeleton 공간).
	Transform3D pose;
	// mirror bone 쪽 모양의 위치와 회전.
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

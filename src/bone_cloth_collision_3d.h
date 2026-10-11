// A shape on a bone that the cloth joints collide with, the base of BoneClothSimulator3D's sphere, capsule and plane children,
// as SpringBoneCollision3D is of SpringBoneSimulator3D: a bone name with its hidden index and two offsets from the bone.

#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>


namespace godot {

	class BoneClothCollision3D : public Node3D {
		GDCLASS(BoneClothCollision3D, Node3D)

			String bone_name;
		int bone = -1;

		Vector3 position_offset;
		Quaternion rotation_offset;

		void _validate_bone_name();

	protected:
		// The node's transform in skeleton space from the last sync_pose, where the shapes collide.
		Transform3D pose;

		static void _bind_methods();
		void _validate_property(PropertyInfo& p_property) const;
		void _notification(int p_what);

		static void _push_segment_out(Vector3& r_a, Vector3& r_b, const Vector3& p_on_segment, const Vector3& p_on_shape, float p_limit, const Vector3& p_fallback);

	public:
		PackedStringArray _get_configuration_warnings() const override;

		Skeleton3D* get_skeleton() const;

		void set_bone_name(const String& p_bone_name);
		String get_bone_name() const;
		void set_bone(int p_bone);
		int get_bone() const;

		void set_position_offset(const Vector3& p_offset);
		Vector3 get_position_offset() const;
		void set_rotation_offset(const Quaternion& p_offset);
		Quaternion get_rotation_offset() const;

		void sync_pose();
		virtual Vector3 collide(const Vector3& p_point, float p_radius) const;
		virtual void collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const;

	};
}

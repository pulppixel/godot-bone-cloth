// A capsule on a bone that pushes the cloth joints out of it, a child of BoneClothSimulator3D as SpringBoneCollisionCapsule3D is of SpringBoneSimulator3D.
// The push is KawaiiPhysics' capsule collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>


namespace godot {

	class BoneClothCapsule3D : public Node3D {
		GDCLASS(BoneClothCapsule3D, Node3D)

		String bone_name;
		Vector3 position_offset;
		Quaternion rotation_offset;
		float radius = 0.1f;
		float height = 0.5f;

		// The axis ends and a push direction for a point on the axis, in skeleton space, from the last sync_pose.
		Vector3 head;
		Vector3 tail;
		Vector3 fallback_direction;

	protected:
		static void _bind_methods();
		void _validate_property(PropertyInfo& p_property) const;

	public:
		PackedStringArray _get_configuration_warnings() const override;

		Skeleton3D* get_skeleton() const;

		void set_bone_name(const String& p_name);
		String get_bone_name() const;
		void set_position_offset(const Vector3& p_offset);
		Vector3 get_position_offset() const;
		void set_rotation_offset(const Quaternion& p_offset);
		Quaternion get_rotation_offset() const;
		void set_radius(float p_radius);
		float get_radius() const;
		void set_height(float p_height);
		float get_height() const;

		void sync_pose();
		Vector3 collide(const Vector3& p_point, float p_radius) const;
	};


}

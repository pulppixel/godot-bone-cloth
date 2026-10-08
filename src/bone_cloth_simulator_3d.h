// Bone cloth: spring chains linked to their neighbours so a skirt of several chains moves as one surface and stays out of the legs.
// The solver is ported from KawaiiPhysics (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/templates/local_vector.hpp>


namespace godot {

	class BoneClothSimulator3D : public SkeletonModifier3D {
		GDCLASS(BoneClothSimulator3D, SkeletonModifier3D)

		// Everything is in skeleton space.
		struct Joint {
			int bone = -1;
			Vector3 location;
			Vector3 prev_location;
			Vector3 pose_location;
			Basis pose_basis;
		};

		String root_bone;
		String end_bone;
		float end_bone_length = 0.1f;
		float damping = 0.1f;
		float stiffness = 0.05f;
		Vector3 gravity;

		LocalVector<Joint> joints;
		Vector3 end_axis;
		bool joints_dirty = true;
		bool needs_reset = true;
		double delta_old = 0.0;

		// State
		bool _build_joints(Skeleton3D* p_skeleton);
		void _read_pose(Skeleton3D* p_skeleton);
		void _simulate(double p_delta, const Vector3& p_gravity);
		void _restore_lengths();
		void _write_rotations(Skeleton3D* p_skeleton);

	protected:
		static void _bind_methods();
		void _validate_property(PropertyInfo& p_property) const;

	public:
		void set_root_bone(const String& p_bone);
		String get_root_bone() const;
		void set_end_bone(const String& p_bone);
		String get_end_bone() const;
		void set_end_bone_length(float p_length);
		float get_end_bone_length() const;
		void set_damping(float p_damping);
		float get_damping() const;
		void set_stiffness(float p_stiffness);
		float get_stiffness() const;
		void set_gravity(const Vector3& p_gravity);
		Vector3 get_gravity() const;

		void reset();
		void _process_modification_with_delta(double p_delta) override;
	};


}



// Bone cloth: spring chains linked to their neighbours so a skirt of several chains moves as one surface and stays out of the legs.
// The solver is ported from KawaiiPhysics (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "bone_cloth_capsule_3d.h"

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/templates/local_vector.hpp>


namespace godot {

	class BoneClothSimulator3D : public SkeletonModifier3D {
		GDCLASS(BoneClothSimulator3D, SkeletonModifier3D)

	public:
		enum LinkMode {
			LINK_MODE_NONE,
			LINK_MODE_SEQUENTIAL,
			LINK_MODE_LOOP,
		};

	private:
		// Everything is in skeleton space.
		struct Joint {
			int bone = -1;
			Vector3 location;
			Vector3 prev_location;
			Vector3 pose_location;
			Basis pose_basis;
		};

		struct Chain {
			String root_bone;
			String end_bone;
			LocalVector<Joint> joints;
			Vector3 end_axis;
		};

		// Two joints at the same depth of neighbouring chains, kept at their rest distance.
		struct Link {
			uint32_t chain_a = 0;
			uint32_t chain_b = 0;
			uint32_t depth = 0;
			float length = 0.0f;
		};

		LocalVector<Chain> chains;
		LinkMode link_mode = LINK_MODE_NONE;
		float end_bone_length = 0.1f;
		float damping = 0.1f;
		float stiffness = 0.05f;
		float radius = 0.03f;
		Vector3 gravity;

		LocalVector<Link> links;
		bool joints_dirty = true;
		bool needs_reset = true;
		double delta_old = 0.0;

		// State
		bool _build_joints(Skeleton3D* p_skeleton, Chain& p_chain);
		void _build_links(Skeleton3D* p_skeleton);
		void _read_pose(Skeleton3D* p_skeleton, Chain& p_chain);
		void _simulate(Chain& p_chain, double p_delta, const Vector3& p_gravity);
		void _solve_links(double p_delta);
		void _collide(Chain& p_chain, const LocalVector<BoneClothCapsule3D*>& p_capsules);
		void _restore_lengths(Chain& p_chain);
		void _write_rotations(Skeleton3D* p_skeleton, const Chain& p_chain);

	protected:
		static void _bind_methods();
		bool _set(const StringName& p_name, const Variant& p_value);
		bool _get(const StringName& p_name, Variant& r_ret) const;
		void _get_property_list(List<PropertyInfo>* p_list) const;

	public:
		void set_chain_count(int p_count);
		int get_chain_count() const;
		void set_link_mode(LinkMode p_mode);
		LinkMode get_link_mode() const;
		void set_end_bone_length(float p_length);
		float get_end_bone_length() const;
		void set_damping(float p_damping);
		float get_damping() const;
		void set_stiffness(float p_stiffness);
		float get_stiffness() const;
		void set_radius(float p_radius);
		float get_radius() const;
		void set_gravity(const Vector3& p_gravity);
		Vector3 get_gravity() const;

		void reset();
		void _process_modification_with_delta(double p_delta) override;
	};


}

VARIANT_ENUM_CAST(BoneClothSimulator3D::LinkMode);


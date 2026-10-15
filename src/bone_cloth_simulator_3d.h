// Bone cloth: spring chains linked to their neighbors so a skirt of several chains moves as one surface and stays out of the legs.
// The solver is ported from KawaiiPhysics (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "bone_cloth_preset.h"
#include "collision/bone_cloth_collision_3d.h"

#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/templates/local_vector.hpp>

using namespace godot;

class BoneClothSimulator3DGizmoPlugin;

class BoneClothSimulator3D : public SkeletonModifier3D {
	GDCLASS(BoneClothSimulator3D, SkeletonModifier3D)

	// The gizmo draws the solver's state as it is.
	friend class BoneClothSimulator3DGizmoPlugin;

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
		// pose_location is the target of the step running, between prev_pose_location (last frame) and current_pose_location.
		Vector3 pose_location;
		Vector3 prev_pose_location;
		Vector3 current_pose_location;
		// Where the joint is drawn, which moves every frame even when no step runs.
		Vector3 display_location;
		Basis pose_basis;

		// A later chain's root is this joint: it keeps its pose rotation, as KawaiiPhysics keeps a bone with several children.
		bool branched = false;

		// The joint's share of the chain's rest length from the root, where the curves are read, and its settings scaled by them.
		float length_rate = 0.0f;
		float drag = 0.0f;
		float stiffness = 0.0f;
		float radius = 0.0f;
		float limit_angle = 0.0f;
		float gravity = 0.0f;
	};

	struct Chain {
		// SpringBoneSimulator3D's pair: the name is what the user picks and wins on load, the index is saved beside it.
		String root_bone_name;
		int root_bone = -1;
		String end_bone_name;
		int end_bone = -1;
		// SpringBoneSimulator3D's extended end bone: a joint past the end bone, without which the end bone has no point below it and cannot turn.
		// On by default, unlike the engine's, as skirt and hair rigs seldom carry a tail bone.
		bool extend_end_bone = true;
		BoneDirection end_bone_direction = BONE_DIRECTION_FROM_PARENT;
		float end_bone_length = 0.1f;
		LocalVector<Joint> joints;

		Vector3 end_axis;

		// A branch: the root bone is a joint of an earlier chain, which the root follows instead of the pose.
		int parent_chain = -1;
		uint32_t parent_joint = 0;

		// SpringBoneSimulator3D's collision lists: every child collision but the excluded ones, or only the listed ones.
		bool enable_all_child_collisions = true;
		LocalVector<NodePath> exclude_collisions;
		LocalVector<NodePath> collisions;
		// Which of this frame's posed collisions push the chain, by their index there.
		LocalVector<bool> collides;
	};

	// Two joints at the same depth of neighboring chains, kept at their rest distance.
	struct Link {
		uint32_t chain_a = 0;
		uint32_t chain_b = 0;
		uint32_t depth = 0;
		float length = 0.0f;
	};

	// A collision at one of its two poses, its own or its mirror's.
	struct PosedCollision {
		const BoneClothCollision3D *collision = nullptr;
		bool mirror = false;
	};

	LocalVector<Chain> chains;
	LinkMode link_mode = LINK_MODE_NONE;
	float link_stiffness = 0.0f;
	float drag = 0.1f;
	Ref<Curve> drag_damping_curve;
	float stiffness = 0.05f;
	Ref<Curve> stiffness_damping_curve;
	float radius = 0.03f;
	Ref<Curve> radius_damping_curve;
	float limit_angle = 0.0f;
	Ref<Curve> limit_angle_damping_curve;
	float gravity = 0.0f;
	Ref<Curve> gravity_damping_curve;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	float inertia = 1.0f;
	float inertia_smoothing = 0.4f;
	float movement_speed_limit = 5.0f;
	float rotation_speed_limit = Math::deg_to_rad(720.0f);
	float joint_speed_limit = 4.0f;
	float teleport_distance = 0.5f;
	float teleport_angle = Math::deg_to_rad(90.0f);

	LocalVector<Link> links;
	bool joints_dirty = true;
	bool needs_reset = true;
	double step_time = 0.0;
	Transform3D prev_skeleton_transform;
	Vector3 prev_step_move;
	Quaternion prev_step_turn;
	// The skeleton's speed as the cloth feels it, eased toward its real speed step by step, in world space so a turn does not carry it round.
	Vector3 smoothing_velocity;

	// State
	bool _build_joints(Skeleton3D *p_skeleton, Chain &p_chain);
	void _build_branches();
	void _build_links(Skeleton3D *p_skeleton);
	bool _is_child_collision(const NodePath &p_node_path) const;
	void _find_chain_collisions(const LocalVector<PosedCollision> &p_collisions);
	void _read_pose(Skeleton3D *p_skeleton, Chain &p_chain);
	void _update_joint_settings(Chain &p_chain);
	void _step(const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn, const LocalVector<PosedCollision> &p_collisions);
	void _follow_parent_chain(Chain &p_chain);
	void _simulate(Chain &p_chain, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn);
	void _solve_links();
	void _collide(Chain &p_chain, const LocalVector<PosedCollision> &p_collisions);
	void _collide_links(const LocalVector<PosedCollision> &p_collisions);
	void _restore_limits_and_lengths(Chain &p_chain);
	void _update_display(Chain &p_chain, float p_follow);
	void _write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain);

protected:
	static void _bind_methods();
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	void _validate_property(PropertyInfo &p_property) const;

public:
	void apply_preset(const Ref<BoneClothPreset> &p_preset);
	Ref<BoneClothPreset> make_preset() const;

	void set_chain_count(int p_count);
	int get_chain_count() const;
	void set_root_bone_name(int p_index, const String &p_bone_name);
	String get_root_bone_name(int p_index) const;
	void set_root_bone(int p_index, int p_bone);
	int get_root_bone(int p_index) const;
	void set_end_bone_name(int p_index, const String &p_bone_name);
	String get_end_bone_name(int p_index) const;
	void set_end_bone(int p_index, int p_bone);
	int get_end_bone(int p_index) const;
	void set_extend_end_bone(int p_index, bool p_enabled);
	bool is_end_bone_extended(int p_index) const;
	void set_end_bone_direction(int p_index, BoneDirection p_bone_direction);
	BoneDirection get_end_bone_direction(int p_index) const;
	void set_end_bone_length(int p_index, float p_length);
	float get_end_bone_length(int p_index) const;
	void set_enable_all_child_collisions(int p_index, bool p_enabled);
	bool are_all_child_collisions_enabled(int p_index) const;
	void set_exclude_collision_count(int p_index, int p_count);
	int get_exclude_collision_count(int p_index) const;
	void set_exclude_collision_path(int p_index, int p_collision, const NodePath &p_node_path);
	NodePath get_exclude_collision_path(int p_index, int p_collision) const;
	void clear_exclude_collisions(int p_index);
	void set_collision_count(int p_index, int p_count);
	int get_collision_count(int p_index) const;
	void set_collision_path(int p_index, int p_collision, const NodePath &p_node_path);
	NodePath get_collision_path(int p_index, int p_collision) const;
	void clear_collisions(int p_index);
	void set_link_mode(LinkMode p_mode);
	LinkMode get_link_mode() const;
	void set_link_stiffness(float p_stiffness);
	float get_link_stiffness() const;
	void set_drag(float p_drag);
	float get_drag() const;
	void set_drag_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_drag_damping_curve() const;
	void set_stiffness(float p_stiffness);
	float get_stiffness() const;
	void set_stiffness_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_stiffness_damping_curve() const;
	void set_radius(float p_radius);
	float get_radius() const;
	void set_radius_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_radius_damping_curve() const;
	void set_limit_angle(float p_angle);
	float get_limit_angle() const;
	void set_limit_angle_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_limit_angle_damping_curve() const;
	void set_gravity(float p_gravity);
	float get_gravity() const;
	void set_gravity_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_gravity_damping_curve() const;
	void set_gravity_direction(const Vector3 &p_direction);
	Vector3 get_gravity_direction() const;
	void set_inertia(float p_inertia);
	float get_inertia() const;
	void set_inertia_smoothing(float p_smoothing);
	float get_inertia_smoothing() const;
	void set_movement_speed_limit(float p_limit);
	float get_movement_speed_limit() const;
	void set_rotation_speed_limit(float p_limit);
	float get_rotation_speed_limit() const;
	void set_joint_speed_limit(float p_limit);
	float get_joint_speed_limit() const;
	void set_teleport_distance(float p_distance);
	float get_teleport_distance() const;
	void set_teleport_angle(float p_angle);
	float get_teleport_angle() const;

	void reset();
	void _validate_bone_names() override;
	void _process_modification_with_delta(double p_delta) override;
};

VARIANT_ENUM_CAST(BoneClothSimulator3D::LinkMode);

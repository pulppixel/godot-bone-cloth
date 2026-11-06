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

	friend class BoneClothSimulator3DGizmoPlugin;

public:
	enum LinkMode {
		LINK_MODE_NONE,
		LINK_MODE_SEQUENTIAL,
		LINK_MODE_LOOP,
	};

private:
	struct Joint {
		int bone = -1;
		Vector3 location;
		Vector3 prev_location;

		Vector3 pose_location;
		Vector3 prev_pose_location;
		Vector3 current_pose_location;

		Vector3 animated_location;

		Vector3 rest_location;

		Vector3 display_location;
		Basis pose_basis;

		bool branched = false;

		float length_rate = 0.0f;

		float drag = 0.0f;
		float stiffness = 0.0f;
		float radius = 0.0f;
		float limit_angle = 0.0f;
		float gravity = 0.0f;
		float spacing = 0.0f;

		LocalVector<Vector3> collision_pushes;
	};

	struct Chain {
		String root_bone_name;
		int root_bone = -1;
		String end_bone_name;
		int end_bone = -1;

		bool extend_end_bone = true;
		BoneDirection end_bone_direction = BONE_DIRECTION_FROM_PARENT;
		float end_bone_length = 0.1f;
		LocalVector<Joint> joints;

		Vector3 end_axis;

		LocalVector<float> sync_shares;

		int parent_chain = -1;
		uint32_t parent_joint = 0;

		bool enable_all_child_collisions = true;
		LocalVector<NodePath> exclude_collisions;
		LocalVector<NodePath> collisions;

		LocalVector<bool> collides;
	};

	struct SyncBone {
		String bone_name;
		int bone = -1;
	};

	struct Link {
		uint32_t chain_a = 0;
		uint32_t chain_b = 0;
		uint32_t depth = 0;
		float length = 0.0f;
	};

	struct PosedCollision {
		const BoneClothCollision3D *collision = nullptr;
		bool mirror = false;
	};

	struct FrameMotion {
		Transform3D skeleton_transform;
		Vector3 move_location;
		Quaternion move_rotation;
		bool teleported = false;
		double elapsed = 0.0;
		double dropped = 0.0;
		int step_count = 0;
		Vector3 step_move;
		Quaternion step_turn;
		bool smoothing = false;
		float smoothing_rate = 0.0f;
		Vector3 frame_velocity;
		Basis to_skeleton;
		Vector3 gravity_direction;
	};

	struct StepMotion {
		Vector3 gravity_direction;
		Vector3 move;
		Quaternion turn;
		Vector3 center_move;
		Quaternion center_turn;
		float move_share = 0.0f;
		float turn_share = 0.0f;
	};

	LocalVector<Chain> chains;
	LinkMode link_mode = LINK_MODE_NONE;
	float link_stiffness = 0.0f;
	bool link_split = false;
	float link_spacing = 0.0f;
	Ref<Curve> link_spacing_damping_curve;
	float drag = 0.1f;
	Ref<Curve> drag_damping_curve;
	float stiffness = 0.05f;
	Ref<Curve> stiffness_damping_curve;
	float radius = 0.03f;
	Ref<Curve> radius_damping_curve;
	bool rest_overlap = true;
	float limit_angle = 0.0f;
	Ref<Curve> limit_angle_damping_curve;
	float limit_angle_stiffness = 1.0f;
	float gravity = 0.0f;
	Ref<Curve> gravity_damping_curve;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	LocalVector<SyncBone> sync_bones;
	float sync = 1.0f;
	float inertia = 1.0f;
	float inertia_smoothing = 0.4f;
	float depth_inertia = 0.0f;
	float local_movement_speed_limit = 0.0f;
	float local_rotation_speed_limit = 0.0f;
	float movement_speed_limit = 5.0f;
	float rotation_speed_limit = Math::deg_to_rad(720.0f);
	float joint_speed_limit = 4.0f;
	float teleport_distance = 0.5f;
	float teleport_angle = Math::deg_to_rad(90.0f);

	float root_spring_stiffness = 0.0f;
	float root_spring_limit = 0.04f;
	float root_spring_forward_limit = 0.005f;

	float units_per_meter = 1.0f;

	LocalVector<Link> links;
	bool joints_dirty = true;
	bool needs_reset = true;
	double step_time = 0.0;
	Transform3D prev_skeleton_transform;
	Vector3 prev_step_move;
	Quaternion prev_step_turn;

	Vector3 prev_frame_center;
	Quaternion prev_frame_center_rotation;
	Vector3 frame_center;
	Quaternion frame_center_rotation;
	Vector3 step_center;
	Quaternion step_center_rotation;

	Vector3 rest_center;

	Vector3 smoothing_velocity;

	float _get_units_per_meter(const Skeleton3D *p_skeleton) const;
	bool _build_joints(Skeleton3D *p_skeleton, Chain &p_chain);
	void _build_branches();
	void _build_links();
	void _build_sync(Skeleton3D *p_skeleton);
	bool _is_child_collision(const NodePath &p_node_path) const;
	void _find_chain_collisions(const LocalVector<PosedCollision> &p_collisions);
	void _read_pose(Skeleton3D *p_skeleton, Chain &p_chain);
	void _update_joint_settings(Chain &p_chain);
	void _read_center(Skeleton3D *p_skeleton);
	void _update_local_shares(StepMotion &r_step, const Vector3 &p_center, const Quaternion &p_center_rotation) const;
	Quaternion _get_sync_swing(Skeleton3D *p_skeleton, int p_bone) const;
	void _apply_sync(Skeleton3D *p_skeleton);
	void _step(const StepMotion &p_step, const LocalVector<PosedCollision> &p_collisions);
	void _follow_parent_chain(Chain &p_chain);
	void _move_joint(Joint &r_joint, const StepMotion &p_step, const Vector3 &p_gravity_direction);
	void _move_root(Chain &r_chain, const StepMotion &p_step);
	void _simulate(Chain &p_chain, const StepMotion &p_step);
	void _solve_links();
	void _space_links();
	void _collide_segment(Joint &r_a, Joint &r_b, float p_radius, const LocalVector<PosedCollision> &p_collisions, const LocalVector<bool> &p_collides_a, const LocalVector<bool> &p_collides_b, Vector3 Joint::*p_field, bool p_split);
	void _collide(const LocalVector<PosedCollision> &p_collisions, Vector3 Joint::*p_field);
	void _restore_limits_and_lengths(Chain &p_chain);
	void _restore_lengths(Chain &p_chain, Vector3 Joint::*p_field, Vector3 Joint::*p_pose_field);
	void _update_display(Chain &p_chain, float p_follow);
	void _write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain);

	void _rebuild_if_changed(Skeleton3D *p_skeleton);
	void _read_frame(Skeleton3D *p_skeleton);
	void _reset(const Transform3D &p_skeleton_transform);
	void _gather_collisions(LocalVector<PosedCollision> &r_collisions) const;
	FrameMotion _measure_motion(double p_delta, const Transform3D &p_skeleton_transform);
	void _run_steps(FrameMotion &r_motion, const LocalVector<PosedCollision> &p_collisions);
	void _settle_display(double p_delta, const FrameMotion &p_motion, const LocalVector<PosedCollision> &p_collisions);
	void _end_frame(Skeleton3D *p_skeleton);

protected:
	static void _bind_methods();
	void _notification(int p_what);
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
	void set_link_split_enabled(bool p_enabled);
	bool is_link_split_enabled() const;
	void set_link_spacing(float p_spacing);
	float get_link_spacing() const;
	void set_link_spacing_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_link_spacing_damping_curve() const;
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
	void set_rest_overlap_enabled(bool p_enabled);
	bool is_rest_overlap_enabled() const;
	void set_limit_angle(float p_angle);
	float get_limit_angle() const;
	void set_limit_angle_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_limit_angle_damping_curve() const;
	void set_limit_angle_stiffness(float p_stiffness);
	float get_limit_angle_stiffness() const;
	void set_gravity(float p_gravity);
	float get_gravity() const;
	void set_gravity_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_gravity_damping_curve() const;
	void set_gravity_direction(const Vector3 &p_direction);
	Vector3 get_gravity_direction() const;
	void set_sync_bone_count(int p_count);
	int get_sync_bone_count() const;
	void set_sync_bone_name(int p_index, const String &p_bone_name);
	String get_sync_bone_name(int p_index) const;
	void set_sync_bone(int p_index, int p_bone);
	int get_sync_bone(int p_index) const;
	void set_sync(float p_sync);
	float get_sync() const;
	void set_inertia(float p_inertia);
	float get_inertia() const;
	void set_inertia_smoothing(float p_smoothing);
	float get_inertia_smoothing() const;
	void set_depth_inertia(float p_depth_inertia);
	float get_depth_inertia() const;
	void set_local_movement_speed_limit(float p_limit);
	float get_local_movement_speed_limit() const;
	void set_local_rotation_speed_limit(float p_limit);
	float get_local_rotation_speed_limit() const;
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
	void set_root_spring_stiffness(float p_stiffness);
	float get_root_spring_stiffness() const;
	void set_root_spring_limit(float p_limit);
	float get_root_spring_limit() const;
	void set_root_spring_forward_limit(float p_limit);
	float get_root_spring_forward_limit() const;

	void reset();
	float get_units_per_meter() const;
	PackedStringArray _get_configuration_warnings() const override;
	void _validate_bone_names() override;
	void _process_modification_with_delta(double p_delta) override;

	static void add_monitors();
	static void remove_monitors();
};

VARIANT_ENUM_CAST(BoneClothSimulator3D::LinkMode);

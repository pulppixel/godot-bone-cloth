// The solver is ported from KawaiiPhysics (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include "bone_cloth_angle_limit.h"
#include "bone_cloth_preset.h"
#include "collision/bone_cloth_collision_3d.h"

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/templates/local_vector.hpp>

using namespace godot;

#define BONE_CLOTH_VALUE(m_type, m_name) \
	void set_##m_name(m_type p_value) { \
		m_name = p_value; \
	} \
	m_type get_##m_name() const { \
		return m_name; \
	}

class BoneClothSimulator3DGizmoPlugin;

inline uint64_t pack_edge(uint32_t p_a, uint32_t p_b) {
	return (uint64_t(MIN(p_a, p_b)) << 32) | uint64_t(MAX(p_a, p_b));
}

class BoneClothSimulator3D : public SkeletonModifier3D {
	GDCLASS(BoneClothSimulator3D, SkeletonModifier3D)

	friend class BoneClothSimulator3DGizmoPlugin;

public:
	enum LinkMode {
		LINK_MODE_NONE,
		LINK_MODE_SEQUENTIAL,
		LINK_MODE_LOOP,
	};

	enum CameraCulling {
		CAMERA_CULLING_OFF,
		CAMERA_CULLING_RESET,
		CAMERA_CULLING_KEEP,
	};

	enum LimitAngleMode {
		LIMIT_ANGLE_MODE_CONE,
		LIMIT_ANGLE_MODE_PER_AXIS,
	};

	enum TeleportMode {
		TELEPORT_MODE_KEEP,
		TELEPORT_MODE_RESET,
	};

	enum SyncDirection {
		SYNC_DIRECTION_BOTH,
		SYNC_DIRECTION_POSITIVE,
		SYNC_DIRECTION_NEGATIVE,
		SYNC_DIRECTION_NONE,
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
		Basis animated_basis;

		Vector3 rest_location;

		Vector3 display_location;
		Basis pose_basis;

		bool branched = false;

		float length_rate = 0.0f;

		float drag = 0.0f;
		float stiffness = 0.0f;
		float stiffness_damping = 0.0f;
		float radius = 0.0f;
		float limit_angle = 0.0f;
		int twist_axis = 1;
		SwingLimit swing_limit;
		float gravity = 0.0f;
		float wind = 0.0f;
		float contact = 0.0f;
		Vector3 contact_normal;
		float static_friction = 0.0f;
		float spacing = 0.0f;
		float max_distance = 0.0f;
		float backstop_distance = 0.0f;

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

		LocalVector<bool> sync_inherited;

		Vector3 shock_root;
		Vector3 shock_next;
		Vector3 shock_next_before;

		uint32_t wind_index = 0;

		int parent_chain = -1;
		uint32_t parent_joint = 0;
		int same_as = -1;
		bool enable_all_child_collisions = true;
		LocalVector<NodePath> exclude_collisions;
		LocalVector<NodePath> collisions;

		LocalVector<bool> collides;
	};

	struct SyncBone {
		String bone_name;
		int bone = -1;
		SyncDirection directions[3] = { SYNC_DIRECTION_BOTH, SYNC_DIRECTION_BOTH, SYNC_DIRECTION_BOTH };
		float falloff_inner = 0.0f;
		float falloff_outer = 0.0f;
		float falloff_amount = 1.0f;
	};

	struct JointRef {
		uint32_t chain = 0;
		uint32_t depth = 0;
	};

	struct Link {
		JointRef a;
		JointRef b;
		float length = 0.0f;
	};

	struct Bend {
		uint32_t joints[4] = {};
		bool volume = false;
	};

	struct MeshWeights {
		int bones[4] = { -1, -1, -1, -1 };
		float weights[4] = {};
	};

	struct MeshPoints {
		LocalVector<Vector3> positions;
		LocalVector<bool> fixed;
		LocalVector<MeshWeights> weights;
		LocalVector<uint32_t> indices;
	};

	static constexpr float BEND_VOLUME_SCALE = 1000.0f;
	static float get_bend_angle(const Vector3 *p_points);
	static float get_bend_volume(const Vector3 *p_points);

	struct PosedCollision {
		const BoneClothCollision3D *collision = nullptr;
		bool mirror = false;
	};

	struct Forces {
		Vector3 gravity_direction;
		float gravity_share = 1.0f;
		Vector3 wind_forward;
		Vector3 wind_up;
		Vector3 wind_side;
		float wind_speed = 0.0f;
		float wind_turbulence = 0.0f;
	};

	struct FrameMotion {
		Transform3D skeleton_transform;
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
		Forces forces;
		float wind_rate = 0.0f;
	};

	struct StepMotion {
		Forces forces;
		float wind_swell = 1.0f;
		Vector3 move;
		Quaternion turn;
		Vector3 center_move;
		Quaternion center_turn;
		Vector3 spin;
		float move_share = 0.0f;
		float turn_share = 0.0f;
		float velocity_share = 1.0f;
		Vector3 backstop_center;
	};

	LocalVector<Chain> chains;
	LinkMode link_mode = LINK_MODE_NONE;
	float link_stiffness = 0.0f;
	float link_stretch_limit = 0.0f;
	bool link_split = false;
	float link_spacing = 0.0f;
	Ref<Curve> link_spacing_damping_curve;
	float link_bending = 0.0f;
	float drag = 0.1f;
	Ref<Curve> drag_damping_curve;
	float stiffness = 0.05f;
	Ref<Curve> stiffness_damping_curve;
	float stiffness_velocity_damping = 0.0f;
	float radius = 0.03f;
	Ref<Curve> radius_damping_curve;
	bool rest_overlap = true;
	float collision_friction = 0.0f;
	float limit_angle = 0.0f;
	Ref<Curve> limit_angle_damping_curve;
	float limit_angle_stiffness = 1.0f;
	LimitAngleMode limit_angle_mode = LIMIT_ANGLE_MODE_CONE;
	Vector3 limit_angle_lower = Vector3(1, 1, 1) * Math::deg_to_rad(-30.0f);
	Vector3 limit_angle_upper = Vector3(1, 1, 1) * Math::deg_to_rad(30.0f);
	float gravity = 5.0f;
	Ref<Curve> gravity_damping_curve;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	float gravity_falloff = 0.0f;
	float wind_influence = 1.0f;
	Ref<Curve> wind_influence_damping_curve;
	float wind_sync = 0.7f;
	float wind_turbulence = 1.0f;
	float wind_frequency = 1.0f;
	double wind_time = 0.0;
	double wind_seconds = 0.0;
	double wind_seed = 0.0;
	double wind_swell_seed = 0.0;
	LocalVector<SyncBone> sync_bones;
	float sync = 1.0f;
	NodePath anchor;
	float anchor_inertia = 0.0f;
	float inertia = 1.0f;
	float inertia_smoothing = 0.4f;
	float depth_inertia = 0.0f;
	float centrifugal_acceleration = 0.0f;
	float local_movement_speed_limit = 0.0f;
	float local_rotation_speed_limit = 0.0f;
	float movement_speed_limit = 5.0f;
	float rotation_speed_limit = Math::deg_to_rad(720.0f);
	float joint_speed_limit = 4.0f;
	float shock_propagation = 0.0f;
	float shock_threshold = 0.0f;
	float teleport_distance = 0.5f;
	float teleport_angle = Math::deg_to_rad(90.0f);
	TeleportMode teleport_mode = TELEPORT_MODE_KEEP;

	float root_spring_stiffness = 0.0f;
	float root_spring_limit = 0.04f;
	float root_spring_forward_limit = 0.005f;
	float root_spring_noise = 0.0f;

	float max_distance = 0.0f;
	Ref<Curve> max_distance_damping_curve;
	String backstop_bone_name;
	int backstop_bone = -1;
	Vector3 backstop_position_offset;
	float backstop_distance = 0.0f;
	Ref<Curve> backstop_damping_curve;
	float backstop_radius = 10.0f;
	float movement_limit_stiffness = 1.0f;
	float tether_limit = 0.0f;

	int ticks_per_second = 60;
	int max_steps_per_frame = 4;
	float stabilization_time = 0.0f;

	CameraCulling camera_culling = CAMERA_CULLING_OFF;
	float distance_culling = 0.0f;
	float distance_culling_fade = 0.0f;

	float units_per_meter = 1.0f;

	NodePath mesh_instance;
	int64_t mesh_surfaces = 0;
	float mesh_point_spacing = 0.05f;
	LocalVector<String> mesh_fixed_bones;
	LocalVector<JointRef> mesh_point_joints;
	LocalVector<MeshWeights> mesh_point_weights;
	LocalVector<Vector3i> mesh_triangles;
	LocalVector<Quaternion> mesh_triangle_rests;
	LocalVector<int> mesh_skin_bones;
	LocalVector<Transform3D> mesh_skins;
	LocalVector<Transform3D> mesh_poses;
	LocalVector<StringName> mesh_instance_parameters;
	ObjectID mesh_original;
	ObjectID mesh_copy_skeleton;
	ObjectID mesh_copy;
	ObjectID mesh_skeleton;
	LocalVector<int> mesh_follow_bones;
	bool mesh_copy_shown = false;

	LocalVector<Link> links;
	LocalVector<JointRef> bend_joints;
	LocalVector<Bend> bends;
	LocalVector<Vector3> bend_sums;
	LocalVector<int> bend_counts;
	bool joints_dirty = true;
	bool needs_reset = true;
	bool culled = false;
	double step_seconds = 1.0 / 60.0;
	float step_scale = 1.0f;
	double step_time = 0.0;
	float stabilization = 1.0f;
	float distance_weight = 1.0f;
	Transform3D prev_skeleton_transform;
	Transform3D prev_anchor_transform;
	bool has_prev_anchor = false;
	Vector3 prev_step_move;
	Quaternion prev_step_turn;

	Vector3 prev_frame_center;
	Quaternion prev_frame_center_rotation;
	Vector3 frame_center;
	Quaternion frame_center_rotation;
	Vector3 step_center;
	Quaternion step_center_rotation;
	Vector3 prev_backstop_center;
	Vector3 backstop_center;

	Vector3 smoothing_velocity;
	Basis reset_to_skeleton;

	Joint &_get_joint(const JointRef &p_ref) { return chains[p_ref.chain].joints[p_ref.depth]; }
	const Joint &_get_joint(const JointRef &p_ref) const { return chains[p_ref.chain].joints[p_ref.depth]; }
	float _get_units_per_meter(const Skeleton3D *p_skeleton) const;
	bool _build_joints(Skeleton3D *p_skeleton, Chain &p_chain);
	void _build_branches();
	void _build_same_chains();
	void _sort_around(LocalVector<uint32_t> &r_chains) const;
	void _build_links();
	void _build_bends();
	void _add_bends(const LocalVector<Vector3i> &p_triangles);
	bool _read_mesh(Skeleton3D *p_skeleton, MeshPoints &r_points, LocalVector<LocalVector<bool>> &r_cloth) const;
	void _reduce_mesh(MeshPoints &r_points) const;
	void _join_pieces(MeshPoints &r_points) const;
	void _grow_mesh_tree(const MeshPoints &p_points, const LocalVector<LocalVector<uint32_t>> &p_neighbors, LocalVector<int> &r_parents) const;
	void _build_mesh(Skeleton3D *p_skeleton);
	void _make_mesh_copy(Skeleton3D *p_skeleton, const LocalVector<Vector3> &p_rests, const LocalVector<LocalVector<uint32_t>> &p_neighbors, float p_weight_length, const LocalVector<LocalVector<bool>> &p_cloth);
	void _follow_skeleton();
	void _clear_mesh_copy();
	void _show_mesh_copy(bool p_shown);
	void _refresh_mesh_copy();
	void _read_mesh_pose(Skeleton3D *p_skeleton);
	void _write_mesh(float p_weight);
	void _build_sync(Skeleton3D *p_skeleton);
	bool _is_child_collision(const NodePath &p_node_path) const;
	void _find_chain_collisions(const LocalVector<PosedCollision> &p_collisions);
	void _read_pose(Skeleton3D *p_skeleton, Chain &p_chain);
	void _update_joint_settings(Chain &p_chain);
	void _read_center(Skeleton3D *p_skeleton);
	void _read_backstop_center(Skeleton3D *p_skeleton);
	void _update_local_shares(StepMotion &r_step, const Vector3 &p_center, const Quaternion &p_center_rotation) const;
	Quaternion _get_sync_swing(Skeleton3D *p_skeleton, const SyncBone &p_sync_bone) const;
	float _get_sync_follow(const SyncBone &p_sync_bone, const Vector3 &p_origin, const Vector3 &p_location) const;
	void _apply_sync(Skeleton3D *p_skeleton);
	void _propagate_shock();
	void _step(const StepMotion &p_step, const LocalVector<PosedCollision> &p_collisions);
	void _follow_parent_chain(Chain &p_chain);
	void _tether(Chain &p_chain);
	void _apply_friction();
	Vector3 _get_centrifugal_push(const Joint &p_joint, const StepMotion &p_step, const Vector3 &p_velocity) const;
	void _move_joint(Joint &r_joint, const StepMotion &p_step, const Vector3 &p_force);
	void _move_root(Chain &r_chain, uint32_t p_index, const StepMotion &p_step);
	void _simulate(Chain &p_chain, const StepMotion &p_step);
	Vector3 _get_wind_force(const Joint &p_joint, const Chain &p_chain, const StepMotion &p_step) const;
	void _build_wind(Skeleton3D *p_skeleton);
	void _solve_links();
	void _limit_link_stretch();
	void _bend();
	void _space_links();
	void _collide_segment(Joint &r_a, Joint &r_b, float p_radius, const LocalVector<PosedCollision> &p_collisions, const LocalVector<bool> &p_collides_a, const LocalVector<bool> &p_collides_b, Vector3 Joint::*p_field, bool p_split);
	void _collide(const LocalVector<PosedCollision> &p_collisions, Vector3 Joint::*p_field);
	Vector3 _limit_to_axes(const Vector3 &p_offset, const Vector3 &p_pose_vector, const Joint &p_parent, const Joint &p_joint, float p_stiffness) const;
	void _restore_limits_and_lengths(Chain &p_chain);
	void _restore_lengths(Chain &p_chain, Vector3 Joint::*p_field, Vector3 Joint::*p_pose_field);
	void _limit_movement(Chain &p_chain, Vector3 Joint::*p_field, Vector3 Joint::*p_pose_field, const Vector3 &p_backstop_center);
	Vector3 _get_root_display(const Chain &p_chain) const;
	void _update_display(Chain &p_chain, float p_follow);
	void _write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain, float p_weight);

	bool _can_edit_collisions(int p_index, bool p_exclude) const;
	void _set_collision_count(int p_index, int p_count, bool p_exclude);
	void _set_collision_path(int p_index, int p_collision, const NodePath &p_node_path, bool p_exclude);
	void _rebuild_if_changed(Skeleton3D *p_skeleton);
	void _make_joints_dirty();
	void _read_frame(Skeleton3D *p_skeleton);
	bool _update_culling(Skeleton3D *p_skeleton);
	bool _is_on_screen(const Skeleton3D *p_skeleton, const Camera3D *p_camera) const;
	void _reset(const Transform3D &p_skeleton_transform);
	void _reset_motion(const Transform3D &p_skeleton_transform);
	bool _is_teleported(const Transform3D &p_skeleton_transform) const;
	void _gather_collisions(LocalVector<PosedCollision> &r_collisions) const;
	void _follow_anchor();
	FrameMotion _measure_motion(double p_delta, const Transform3D &p_skeleton_transform, bool p_teleported);
	void _run_steps(FrameMotion &r_motion, const LocalVector<PosedCollision> &p_collisions);
	void _settle_display(double p_delta, const FrameMotion &p_motion, const LocalVector<PosedCollision> &p_collisions);
	void _copy_same_chains();
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

	void set_mesh_instance(const NodePath &p_path);
	NodePath get_mesh_instance() const;
	void set_mesh_surfaces(int64_t p_surfaces);
	int64_t get_mesh_surfaces() const;
	void set_mesh_point_spacing(float p_spacing);
	float get_mesh_point_spacing() const;
	void set_mesh_fixed_bone_count(int p_count);
	int get_mesh_fixed_bone_count() const;
	void set_mesh_fixed_bone(int p_index, const String &p_bone_name);
	String get_mesh_fixed_bone(int p_index) const;

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
	BONE_CLOTH_VALUE(float, link_stiffness)
	BONE_CLOTH_VALUE(float, link_stretch_limit)
	void set_link_split_enabled(bool p_enabled);
	bool is_link_split_enabled() const;
	BONE_CLOTH_VALUE(float, link_spacing)
	BONE_CLOTH_VALUE(Ref<Curve>, link_spacing_damping_curve)
	BONE_CLOTH_VALUE(float, link_bending)
	BONE_CLOTH_VALUE(float, drag)
	BONE_CLOTH_VALUE(Ref<Curve>, drag_damping_curve)
	BONE_CLOTH_VALUE(float, stiffness)
	BONE_CLOTH_VALUE(Ref<Curve>, stiffness_damping_curve)
	BONE_CLOTH_VALUE(float, stiffness_velocity_damping)
	BONE_CLOTH_VALUE(float, radius)
	BONE_CLOTH_VALUE(Ref<Curve>, radius_damping_curve)
	void set_rest_overlap_enabled(bool p_enabled);
	bool is_rest_overlap_enabled() const;
	BONE_CLOTH_VALUE(float, collision_friction)
	BONE_CLOTH_VALUE(float, limit_angle)
	BONE_CLOTH_VALUE(Ref<Curve>, limit_angle_damping_curve)
	BONE_CLOTH_VALUE(float, limit_angle_stiffness)
	void set_limit_angle_mode(LimitAngleMode p_mode);
	LimitAngleMode get_limit_angle_mode() const;
	BONE_CLOTH_VALUE(Vector3, limit_angle_lower)
	BONE_CLOTH_VALUE(Vector3, limit_angle_upper)
	BONE_CLOTH_VALUE(float, gravity)
	BONE_CLOTH_VALUE(Ref<Curve>, gravity_damping_curve)
	void set_gravity_direction(const Vector3 &p_direction);
	Vector3 get_gravity_direction() const;
	BONE_CLOTH_VALUE(float, gravity_falloff)
	BONE_CLOTH_VALUE(float, wind_influence)
	BONE_CLOTH_VALUE(Ref<Curve>, wind_influence_damping_curve)
	BONE_CLOTH_VALUE(float, wind_sync)
	BONE_CLOTH_VALUE(float, wind_turbulence)
	BONE_CLOTH_VALUE(float, wind_frequency)
	void set_sync_bone_count(int p_count);
	int get_sync_bone_count() const;
	void set_sync_bone_name(int p_index, const String &p_bone_name);
	String get_sync_bone_name(int p_index) const;
	void set_sync_bone(int p_index, int p_bone);
	int get_sync_bone(int p_index) const;
	void set_sync_bone_direction(int p_index, Vector3::Axis p_axis, SyncDirection p_direction);
	SyncDirection get_sync_bone_direction(int p_index, Vector3::Axis p_axis) const;
	void set_sync_bone_falloff_inner(int p_index, float p_radius);
	float get_sync_bone_falloff_inner(int p_index) const;
	void set_sync_bone_falloff_outer(int p_index, float p_radius);
	float get_sync_bone_falloff_outer(int p_index) const;
	void set_sync_bone_falloff_amount(int p_index, float p_amount);
	float get_sync_bone_falloff_amount(int p_index) const;
	BONE_CLOTH_VALUE(float, sync)
	void set_anchor(const NodePath &p_anchor);
	NodePath get_anchor() const;
	BONE_CLOTH_VALUE(float, anchor_inertia)
	BONE_CLOTH_VALUE(float, inertia)
	BONE_CLOTH_VALUE(float, inertia_smoothing)
	BONE_CLOTH_VALUE(float, depth_inertia)
	BONE_CLOTH_VALUE(float, centrifugal_acceleration)
	BONE_CLOTH_VALUE(float, local_movement_speed_limit)
	BONE_CLOTH_VALUE(float, local_rotation_speed_limit)
	BONE_CLOTH_VALUE(float, movement_speed_limit)
	BONE_CLOTH_VALUE(float, rotation_speed_limit)
	BONE_CLOTH_VALUE(float, joint_speed_limit)
	BONE_CLOTH_VALUE(float, shock_propagation)
	BONE_CLOTH_VALUE(float, shock_threshold)
	BONE_CLOTH_VALUE(float, teleport_distance)
	BONE_CLOTH_VALUE(float, teleport_angle)
	BONE_CLOTH_VALUE(TeleportMode, teleport_mode)
	BONE_CLOTH_VALUE(float, root_spring_stiffness)
	BONE_CLOTH_VALUE(float, root_spring_limit)
	BONE_CLOTH_VALUE(float, root_spring_forward_limit)
	BONE_CLOTH_VALUE(float, root_spring_noise)
	BONE_CLOTH_VALUE(float, max_distance)
	BONE_CLOTH_VALUE(Ref<Curve>, max_distance_damping_curve)
	void set_backstop_bone_name(const String &p_bone_name);
	String get_backstop_bone_name() const;
	void set_backstop_bone(int p_bone);
	int get_backstop_bone() const;
	BONE_CLOTH_VALUE(Vector3, backstop_position_offset)
	BONE_CLOTH_VALUE(float, backstop_distance)
	BONE_CLOTH_VALUE(Ref<Curve>, backstop_damping_curve)
	BONE_CLOTH_VALUE(float, backstop_radius)
	BONE_CLOTH_VALUE(float, movement_limit_stiffness)
	BONE_CLOTH_VALUE(float, tether_limit)

	void set_ticks_per_second(int p_ticks);
	int get_ticks_per_second() const;
	void set_max_steps_per_frame(int p_steps);
	int get_max_steps_per_frame() const;
	void set_stabilization_time(float p_seconds);
	float get_stabilization_time() const;

	BONE_CLOTH_VALUE(CameraCulling, camera_culling)
	void set_distance_culling(float p_distance);
	float get_distance_culling() const;
	void set_distance_culling_fade(float p_fade);
	float get_distance_culling_fade() const;

	void reset();
	float get_units_per_meter() const;
	PackedStringArray _get_configuration_warnings() const override;
	void _validate_bone_names() override;
	void _skeleton_changed(Skeleton3D *p_old_skeleton, Skeleton3D *p_new_skeleton) override;
	void _process_modification_with_delta(double p_delta) override;

	static void add_monitors();
	static void remove_monitors();
};

VARIANT_ENUM_CAST(BoneClothSimulator3D::LinkMode);
VARIANT_ENUM_CAST(BoneClothSimulator3D::CameraCulling);
VARIANT_ENUM_CAST(BoneClothSimulator3D::LimitAngleMode);
VARIANT_ENUM_CAST(BoneClothSimulator3D::TeleportMode);
VARIANT_ENUM_CAST(BoneClothSimulator3D::SyncDirection);

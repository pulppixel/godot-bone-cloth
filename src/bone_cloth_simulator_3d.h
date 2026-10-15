// Bone cloth: 이웃한 spring chain을 연결해서, 여러 chain으로 된 치마가 하나의 면처럼 움직이고 다리에 뚫리지 않게 한다.

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

	// Gizmo는 simulator의 계산을 읽고 화면에 그린다.
	friend class BoneClothSimulator3DGizmoPlugin;

public:
	enum LinkMode {
		LINK_MODE_NONE,
		LINK_MODE_SEQUENTIAL,
		LINK_MODE_LOOP,
	};

private:
	// ※ 위치와 회전은 모두 Skeleton 공간 기준이다.
	struct Joint {
		int bone = -1;
		Vector3 location;
		Vector3 prev_location;

		// 실행 중인 step의 목표 위치  (*step: 천을 계산하는 단위, 1/60초에 한 번 실행된다).
		// prev_pose_location(지난 frame)과 current_pose_location 사이를 보간한 값이다.
		Vector3 pose_location;
		Vector3 prev_pose_location;
		Vector3 current_pose_location;

		// 화면에 그리는 위치 벡터 (step이 없는 frame에도 동작한다).
		Vector3 display_location;
		Basis pose_basis;

		// 다른 chain의 root가 이 joint다. 자식 chain마다 방향이 달라 회전 하나로 맞출 수 없으므로, pose 회전은 그대로 둔다.
		bool branched = false;

		// root부터의 rest 길이의 비율값. (root: 0 ~ rest: 1)
		float length_rate = 0.0f;
		// joint마다 사용할 값. 인스펙터의 값에 위 length_rate에 해당하는 위치의 curve 값을 곱한다.
		float drag = 0.0f;
		float stiffness = 0.0f;
		float radius = 0.0f;
		float limit_angle = 0.0f;
		float gravity = 0.0f;
	};

	struct Chain {
		// inspector에서 bone 이름을 고르면 내부적으로 index도 같이 저장한다. skeleton의 bone 순서가 바뀌어도 같은 bone을 찾도록 이름을 키 값으로 사용한다.
		String root_bone_name;
		int root_bone = -1;
		String end_bone_name;
		int end_bone = -1;
		// end bone 뒤에 joint 하나를 더한다. 이 joint가 없으면 end bone이 회전할 수 없다.
		// 치마와 머리카락 rig에는 끝 bone이 없는 경우가 많기 때문에, 기본값을 '켜짐'으로 했다.
		bool extend_end_bone = true;
		BoneDirection end_bone_direction = BONE_DIRECTION_FROM_PARENT;
		float end_bone_length = 0.1f;
		LocalVector<Joint> joints;

		Vector3 end_axis;

		/*
			Hair1 - Hair2 - Hair3 - Hair4   (chain 0)
					  |				    
					  +-- HairB1 - HairB2     (chain 1, root bone: Hair2)

			위처럼 Chain 중간에서 파생된 Chain이면, 파생된 Chain과 joint의 값을 저장, 아니면 -1
			parent joint(Hair2)는 흔들리기 때문에, 이 chain의 root는 해당 joint(Hair2)의 위치를 따라간다.
		*/
		int parent_chain = -1;
		uint32_t parent_joint = 0;

		// true: 자식 collision 중 exclude_collisions에 없는 것 전부와 충돌한다.
		// false: collisions에 있는 collision하고만 충돌한다.
		bool enable_all_child_collisions = true;
		LocalVector<NodePath> exclude_collisions;
		LocalVector<NodePath> collisions;

		/*
			BoneClothSimulator3D
			├ Thighs
			├ Chest
			└ Calves

			만약, exclude_collisions에 Thights를 넣었다면 아래와 같이 된다.

			칸 번호:   0       1       2
			list:     Thighs  Chest  Calves
			collides: false   true   true

			여기서 true인 collision만 충돌시킨다는 뜻이다.
		*/
		LocalVector<bool> collides;
	};

	/*
				  chain 0    chain 1    chain 2
		depth 0     R0         R1         R2       <- root끼리는 연결하지 않는다.
					|          |          |
		depth 1     a1 ------- b1 ------- c1
					|          |          |
		depth 2     a2 ------- b2 ------- c2
					|          |          |
		depth 3     a3 ------- b3 ------- c3

		위 그림에서 |는 bone(chain으로 연결되는 부모-자식 관계)
		----가 바로 Link이다.

		Link는 chain끼리 연결하는 역할을 한다 (이게 없다면 천이 벌어지고, 그 틈으로 body가 뚫고 나오게 된다).
		즉, 여러 chain이 한 면처럼 움직일 수 있도록, 이웃한 두 chain의 "동일한 depth를 가진 joint를 연결한 선 하나"
	*/
	struct Link {
		uint32_t chain_a = 0;
		uint32_t chain_b = 0;
		uint32_t depth = 0;
		float length = 0.0f;
	};

	// 충돌에 쓸 collision. mirror가 존재하는 collision일 경우, 2개가 들어간다. (mirror는 좌우대칭 콜리전, 자동으로 값 따라감)
	struct PosedCollision {
		const BoneClothCollision3D *collision = nullptr; // 어느 collision 노드인지.
		bool mirror = false;							 // true: mirror bone, false: 본인의 bone
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
	// 급출발, 급정지 때 천이 튀지 않도록, 천이 감지하는 캐릭터 속도를 실제 속도 쪽으로 천천히 바꿔 간다.
	// 캐릭터가 돌아도 이 속도의 방향이 같이 돌지 않도록 world 공간으로 저장한다.
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

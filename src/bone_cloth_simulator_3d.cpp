#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

#include <atomic>
#include <chrono>

using namespace godot;

// frame rate가 달라도 천이 자연스럽게 움직이도록, 1/60초짜리 step으로 계산한다.
// step은 한 frame에 4번까지 돈다. 느린 frame에서 연산이 계속 돌지 않게 하려는 것이다 (초과한 시간은 버린다).
// drag, stiffness, link_stiffness는 step 1번 기준 값이다.
static constexpr double STEP = 1.0 / 60.0;
static constexpr int MAX_STEPS = 4;

// link의 부드러움 정도 (사실상 거의 늘어나지 않는다. 세기는 link_stiffness로 정할 수 있다).
static constexpr float LINK_COMPLIANCE = 1e-9f;

// link_spacing이 미는 힘이 다 차는 거리. 반지름 합의 30%만큼 더 가까워지면 다 민다.
static constexpr float LINK_SPACING_RAMP = 0.3f;

// Debugger > Monitors 탭의 Bone Cloth 값 하나(시간 또는 joint 수).
struct MonitorSum {
	std::atomic<uint64_t> sum = 0;
	uint64_t read_frame = 0;
	double average = 0.0;

	// 같은 frame에 다시 읽으면 지난 평균을 그대로 돌려준다.
	double read() {
		const uint64_t frame = Engine::get_singleton()->get_process_frames();
		if (frame != read_frame) {
			average = double(sum.exchange(0)) / double(frame - read_frame);
			read_frame = frame;
		}
		return average;
	}
};

static MonitorSum monitor_time; // _process_modification_with_delta에 쓴 시간(ns)
static MonitorSum monitor_joints; // 계산한 joint 수

// MONITOR_TYPE_TIME은 초로 받아서 ms로 보여 준다.
static double get_monitor_time() {
	return monitor_time.read() / 1e9;
}

static int64_t get_monitor_joints() {
	return int64_t(Math::round(monitor_joints.read()));
}

// tip: end bone 뒤에 붙이는 가상의 joint.
// bone은 아래 joint를 향해 회전하므로, 이 joint가 있어야 end bone도 회전한다.
// tip joint를 end bone에서 어느 방향으로 붙일지 구한다. 결과는 end bone 자신의 축 기준이라, end bone이 돌면 tip도 같이 돈다.
// +X 같은 축을 고르면 그 축을 쓰고, FromParent를 고르면 rest 자세에서 부모 bone이 end bone으로 향하는 방향을 그대로 이어 간다.
static Vector3 end_bone_axis(Skeleton3D *p_skeleton, int p_end_bone, SkeletonModifier3D::BoneDirection p_direction) {
	if (p_direction != SkeletonModifier3D::BONE_DIRECTION_FROM_PARENT) {
		static const Vector3 axes[] = { Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0), Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(0, 0, -1) };
		return axes[p_direction];
	}

	const Transform3D rest = p_skeleton->get_bone_rest(p_end_bone);
	return rest.basis.xform_inv(rest.origin).normalized();
}

// chain의 bone마다 joint를 만들고(tip 포함), 각 joint에 root부터의 rest 길이를 length_rate에 넣는다.
// bone마다 길이가 다르기 때문에, bone 개수가 아니라 길이를 잰다(아직 skeleton 단위 길이다. 아래 _build_branches가 0 ~ 1로 바꾼다).
bool BoneClothSimulator3D::_build_joints(Skeleton3D *p_skeleton, Chain &p_chain) {
	p_chain.joints.clear();
	const int root = p_chain.root_bone;
	const int end = p_chain.end_bone;

	// bone이 없거나 end bone이 root bone 아래에 없으면 chain을 만들지 않는다(그 chain은 움직이지 않는다).
	// 이유는 configuration warning으로 보여 준다.
	if (root < 0 || end < 0) {
		return false;
	}

	LocalVector<int> bones;
	for (int bone = end; bone != root; bone = p_skeleton->get_bone_parent(bone)) {
		if (bone < 0) {
			return false;
		}

		bones.push_back(bone);
	}

	bones.push_back(root);
	bones.reverse();

	for (int bone : bones) {
		Joint joint;
		joint.bone = bone;
		p_chain.joints.push_back(joint);
	}

	// end bone이 부모 bone과 같은 위치에 있으면 tip을 만들지 않는다.
	p_chain.end_axis = end_bone_axis(p_skeleton, end, p_chain.end_bone_direction);
	if (p_chain.extend_end_bone && p_chain.end_bone_length > 0.0f && !p_chain.end_axis.is_zero_approx()) {
		p_chain.joints.push_back(Joint());
	}

	float length = 0.0f;
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Vector3 parent_rest = p_skeleton->get_bone_global_rest(p_chain.joints[i - 1].bone).origin;
		length += joint.bone >= 0 ? (p_skeleton->get_bone_global_rest(joint.bone).origin - parent_rest).length() : p_chain.end_bone_length * units_per_meter;
		joint.length_rate = length;
	}

	for (uint32_t i = 0; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		if (joint.bone >= 0) {
			joint.rest_location = p_skeleton->get_bone_global_rest(joint.bone).origin;
		} else {
			const Transform3D end = p_skeleton->get_bone_global_rest(p_chain.joints[i - 1].bone);
			joint.rest_location = end.origin + end.basis.orthonormalized().xform(p_chain.end_axis) * (p_chain.end_bone_length * units_per_meter);
		}
	}

	return true;
}

// 머리카락처럼 중간에서 갈라지는 bone을 쓰기 위해, 파생 chain(root bone이 다른 chain의 joint인 chain)을 찾아 parent chain에 연결한다.
// 그다음 length_rate를 0 ~ 1로 바꾼다. 파생 chain은 parent joint의 길이에서 이어 재고, parent chain과 함께 가장 긴 끝으로 나눈다.
// 그래야 갈라진 곳에서 curve 값이 끊기지 않는다(예: Hair2에서 갈라진 HairB1은 Hair3과 비슷한 값).
void BoneClothSimulator3D::_build_branches() {
	LocalVector<uint32_t> tree;
	tree.resize(chains.size());
	for (uint32_t c = 0; c < chains.size(); c++) {
		Chain &chain = chains[c];
		chain.parent_chain = -1;
		chain.parent_joint = 0;
		tree[c] = c;
		if (chain.joints.is_empty()) {
			continue;
		}

		// parent chain은 앞 chain에서만 찾는다.
		const int root = chain.joints[0].bone;
		for (uint32_t other = 0; other < c && chain.parent_chain < 0; other++) {
			for (uint32_t j = 0; j < chains[other].joints.size(); j++) {
				if (chains[other].joints[j].bone != root) {
					continue;
				}

				chain.parent_chain = int(other);
				chain.parent_joint = j;
				chains[other].joints[j].branched = true;
				tree[c] = tree[other];
				break;
			}
		}
	}

	LocalVector<float> longest;
	longest.resize(chains.size());
	for (uint32_t c = 0; c < chains.size(); c++) {
		longest[c] = 0.0f;
	}

	for (uint32_t c = 0; c < chains.size(); c++) {
		Chain &chain = chains[c];
		if (chain.joints.is_empty()) {
			continue;
		}

		const float offset = chain.parent_chain >= 0 ? chains[chain.parent_chain].joints[chain.parent_joint].length_rate : 0.0f;
		for (Joint &joint : chain.joints) {
			joint.length_rate += offset;
		}

		longest[tree[c]] = MAX(longest[tree[c]], chain.joints[chain.joints.size() - 1].length_rate);
	}

	for (uint32_t c = 0; c < chains.size(); c++) {
		const float length = longest[tree[c]];
		for (Joint &joint : chains[c].joints) {
			joint.length_rate = length > 0.0f ? joint.length_rate / length : 0.0f;
		}
	}
}

// 여러 chain이 한 면처럼 움직이도록 같은 depth의 joint를 link로 연결한다 (root 제외)
// tip은 tip끼리만 연결한다.
void BoneClothSimulator3D::_build_links() {
	links.clear();
	LocalVector<uint32_t> linked;
	for (uint32_t c = 0; c < chains.size(); c++) {
		if (chains[c].parent_chain < 0 || chains[c].parent_joint == 0) {
			linked.push_back(c);
		}
	}

	const uint32_t count = linked.size();
	if (link_mode == LINK_MODE_NONE || count < 2) {
		return;
	}

	// Loop라도 chain이 2개면 마지막과 처음은 이미 이어져있다. 그래서 3개 이상일 때만 마지막과 처음을 잇는다.
	const uint32_t pair_count = (link_mode == LINK_MODE_LOOP && count >= 3) ? count : count - 1;
	for (uint32_t i = 0; i < pair_count; i++) {
		const uint32_t a = linked[i];
		const uint32_t b = linked[(i + 1) % count];
		const Chain &chain_a = chains[a];
		const Chain &chain_b = chains[b];
		const uint32_t depth_count = MIN(chain_a.joints.size(), chain_b.joints.size());
		for (uint32_t depth = 1; depth < depth_count; depth++) {
			if ((chain_a.joints[depth].bone < 0) != (chain_b.joints[depth].bone < 0)) {
				continue;
			}

			Link link;
			link.chain_a = a;
			link.chain_b = b;
			link.depth = depth;
			link.length = (chain_a.joints[depth].rest_location - chain_b.joints[depth].rest_location).length();
			links.push_back(link);
		}
	}
}

// sync에 쓸 rest 값을 구한다.
void BoneClothSimulator3D::_build_sync(Skeleton3D *p_skeleton) {
	rest_center = Vector3();
	int count = 0;
	for (const Chain &chain : chains) {
		if (!chain.joints.is_empty() && chain.parent_chain < 0) {
			rest_center += p_skeleton->get_bone_global_rest(chain.joints[0].bone).origin;
			count++;
		}
	}

	if (count > 0) {
		rest_center /= float(count);
	}

	for (Chain &chain : chains) {
		chain.sync_direction = Vector3();
		if (chain.joints.size() < 2) {
			continue;
		}

		Vector3 outward = p_skeleton->get_bone_global_rest(chain.end_bone).origin - rest_center;
		outward.y = 0.0f;
		chain.sync_direction = outward.normalized();
	}
}

// collision 목록에는 이 simulator의 자식 collision만 넣을 수 있다.
bool BoneClothSimulator3D::_is_child_collision(const NodePath &p_node_path) const {
	if (!is_inside_tree() || p_node_path.is_empty()) {
		return true;
	}

	const Node *node = get_node_or_null(p_node_path);
	return node && node->get_parent() == this && Object::cast_to<BoneClothCollision3D>(node);
}

// chain마다 collides를 채운다.
// 자식을 더하거나 빼면 바로 반영되도록 매 frame 다시 계산한다.
void BoneClothSimulator3D::_find_chain_collisions(const LocalVector<PosedCollision> &p_collisions) {
	for (Chain &chain : chains) {
		const bool all = chain.enable_all_child_collisions;
		chain.collides.resize(p_collisions.size());
		for (uint32_t c = 0; c < p_collisions.size(); c++) {
			chain.collides[c] = all;
		}

		for (const NodePath &node_path : all ? chain.exclude_collisions : chain.collisions) {
			const Node *node = get_node_or_null(node_path);
			for (uint32_t c = 0; c < p_collisions.size(); c++) {
				if (p_collisions[c].collision == node) {
					chain.collides[c] = !all;
				}
			}
		}
	}
}

// 이번 frame에 애니메이션(과 앞선 modifier)이 만든 bone 위치와 회전을 읽는다.
void BoneClothSimulator3D::_read_pose(Skeleton3D *p_skeleton, Chain &p_chain) {
	for (Joint &joint : p_chain.joints) {
		if (joint.bone >= 0) {
			const Transform3D pose = p_skeleton->get_bone_global_pose(joint.bone);
			joint.current_pose_location = pose.origin;
			joint.animated_location = pose.origin;
			joint.pose_basis = pose.basis;
		}
	}

	Joint &tip = p_chain.joints[p_chain.joints.size() - 1];
	if (tip.bone < 0) {
		const Joint &end = p_chain.joints[p_chain.joints.size() - 2];
		tip.current_pose_location = end.current_pose_location + end.pose_basis.orthonormalized().xform(p_chain.end_axis) * (p_chain.end_bone_length * units_per_meter);
		tip.animated_location = tip.current_pose_location;
	}
}

// curve에서 p_rate 위치의 값을 읽는다. curve가 없으면 1.0f를 반환한다.
// inspector에서 갓 만든 curve는 point가 없어 0으로 읽힌다. 그대로 곱하면 inspector 값과 상관없이 0이 되므로, 이때도 1.0f를 반환한다.
static float sample_curve(const Ref<Curve> &p_curve, float p_rate) {
	return p_curve.is_valid() && p_curve->get_point_count() > 0 ? p_curve->sample_baked(p_rate) : 1.0f;
}

void BoneClothSimulator3D::_update_joint_settings(Chain &p_chain) {
	for (Joint &joint : p_chain.joints) {
		joint.drag = CLAMP(drag * sample_curve(drag_damping_curve, joint.length_rate), 0.0f, 1.0f);
		joint.stiffness = CLAMP(stiffness * sample_curve(stiffness_damping_curve, joint.length_rate), 0.0f, 1.0f);
		joint.radius = MAX(radius * sample_curve(radius_damping_curve, joint.length_rate), 0.0f) * units_per_meter;
		joint.limit_angle = MAX(limit_angle * sample_curve(limit_angle_damping_curve, joint.length_rate), 0.0f);
		joint.gravity = gravity * sample_curve(gravity_damping_curve, joint.length_rate) * units_per_meter;
		joint.sync = MAX(sync * sample_curve(sync_damping_curve, joint.length_rate), 0.0f);
		joint.spacing = CLAMP(link_spacing * sample_curve(link_spacing_damping_curve, joint.length_rate), 0.0f, 1.0f);
	}
}

void BoneClothSimulator3D::_read_center(Skeleton3D *p_skeleton) {
	Vector3 location_sum;
	Quaternion rotation_sum(0, 0, 0, 0);
	Quaternion first_rotation;
	int count = 0;
	for (const Chain &chain : chains) {
		if (chain.joints.is_empty() || chain.parent_chain >= 0) {
			continue;
		}

		const Joint &root = chain.joints[0];
		Quaternion rotation = (root.pose_basis.orthonormalized() * p_skeleton->get_bone_global_rest(root.bone).basis.orthonormalized().inverse()).get_rotation_quaternion();
		if (count == 0) {
			first_rotation = rotation;
		} else if (first_rotation.dot(rotation) < 0.0f) {
			rotation = -rotation;
		}
		location_sum += root.current_pose_location;
		rotation_sum += rotation;
		count++;
	}

	if (count > 0) {
		frame_center = location_sum / float(count);
		frame_center_rotation = rotation_sum.normalized();
	}
}

void BoneClothSimulator3D::_update_local_shares(const Vector3 &p_center, const Quaternion &p_center_rotation, const Vector3 &p_move, const Quaternion &p_turn) {
	local_move_share = 0.0f;
	local_turn_share = 0.0f;

	if (local_movement_speed_limit > 0.0f) {
		const float speed = (p_center - (p_turn.xform(step_center) + p_move)).length() / float(STEP) / units_per_meter;
		if (speed > local_movement_speed_limit) {
			local_move_share = 1.0f - local_movement_speed_limit / speed;
		}
	}

	if (local_rotation_speed_limit > 0.0f) {
		const float turn_speed = (p_turn * step_center_rotation).normalized().angle_to(p_center_rotation) / float(STEP);
		if (turn_speed > local_rotation_speed_limit) {
			local_turn_share = 1.0f - local_rotation_speed_limit / turn_speed;
		}
	}
}

void BoneClothSimulator3D::_apply_sync(Skeleton3D *p_skeleton) {
	for (Chain &chain : chains) {
		if (chain.sync_bone < 0 || chain.joints.size() < 2) {
			continue;
		}

		const Vector3 location = frame_center_rotation.xform_inv(p_skeleton->get_bone_global_pose(chain.sync_bone).origin - frame_center);
		const Vector3 rest_location = p_skeleton->get_bone_global_rest(chain.sync_bone).origin - rest_center;
		const float along = (location - rest_location).dot(chain.sync_direction);
		if (along <= 0.0f) {
			continue;
		}

		const Vector3 offset = frame_center_rotation.xform(chain.sync_direction * along);
		for (uint32_t i = 1; i < chain.joints.size(); i++) {
			Joint &joint = chain.joints[i];
			const Joint &parent = chain.joints[i - 1];
			const Vector3 direction = joint.current_pose_location + offset * joint.sync - parent.current_pose_location;
			if (!direction.is_zero_approx()) {
				joint.current_pose_location = parent.current_pose_location + direction.normalized() * (joint.animated_location - parent.animated_location).length();
			}
		}
	}
}

// step 1번의 처리 순서.
void BoneClothSimulator3D::_step(const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn, const LocalVector<PosedCollision> &p_collisions) {
	// root는 몸에 붙은 joint라 물리 계산 없이 pose를 따라간다.
	// root_spring_stiffness가 있으면 root도 흔들린다(_move_root).
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty() && chain.parent_chain < 0) {
			Joint &root = chain.joints[0];
			if (root_spring_stiffness > 0.0f) {
				_move_root(chain, p_move, p_turn);
				continue;
			}

			root.prev_location = root.location;
			root.location = root.pose_location;
		}
	}

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_simulate(chain, p_gravity_direction, p_move, p_turn);
	}

	_solve_links();
	_collide(p_collisions, &Joint::location);
	_solve_links();

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_restore_limits_and_lengths(chain);
	}

	_collide(p_collisions, &Joint::location);
	_solve_links();

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_restore_lengths(chain, &Joint::location, &Joint::pose_location);
	}
}

// bone 길이만 다시 맞춘다(각도 제한은 빼고).
void BoneClothSimulator3D::_restore_lengths(Chain &p_chain, Vector3 Joint::*p_field, Vector3 Joint::*p_pose_field) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.*p_pose_field - parent.*p_pose_field;
		const Vector3 offset = joint.*p_field - parent.*p_field;
		joint.*p_field = parent.*p_field + (offset.is_zero_approx() ? pose_vector : offset.normalized() * pose_vector.length());
	}
}

// 파생 chain의 root를 parent joint의 현재 위치로 옮긴다.
void BoneClothSimulator3D::_follow_parent_chain(Chain &p_chain) {
	if (p_chain.parent_chain < 0 || p_chain.joints.is_empty()) {
		return;
	}

	Joint &root = p_chain.joints[0];
	root.prev_location = root.location;
	root.location = chains[p_chain.parent_chain].joints[p_chain.parent_joint].location;
}

// joint 하나를 step 한 번만큼 움직인다.
// 이전 움직임을 이어 가고(drag로 줄이고 gravity를 더한다), 캐릭터 움직임을 반영한다.
void BoneClothSimulator3D::_move_joint(Joint &r_joint, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn) {
	const float dt = float(STEP);

	// 중심(root들)이 이번 step에 움직이고 회전한 양 중 share만큼을 joint에 그대로 옮긴다 (share는 두 몫을 합친 것).
	const float depth_share = depth_inertia * (1.0f - r_joint.length_rate * r_joint.length_rate);
	const float share = local_move_share + (1.0f - local_move_share) * depth_share;
	const float turn_share = local_turn_share + (1.0f - local_turn_share) * depth_share;
	if (share > 0.0f || turn_share > 0.0f) {
		const Vector3 move = center_move * share;
		const Quaternion turn = Quaternion().slerp(center_turn, turn_share);
		r_joint.location = step_center + turn.xform(r_joint.location - step_center) + move;
		r_joint.prev_location = step_center + turn.xform(r_joint.prev_location - step_center) + move;
	}

	Vector3 velocity = (r_joint.location - r_joint.prev_location) / dt;

	// 땅에 대한 joint 속도를 joint_speed_limit으로 제한한다.
	if (joint_speed_limit > 0.0f) {
		const Vector3 carried = (prev_step_move + prev_step_turn.xform(r_joint.prev_location) - r_joint.prev_location) / dt * (1.0f - share);
		velocity = (velocity - carried).limit_length(joint_speed_limit * units_per_meter) + carried;
	}

	r_joint.prev_location = r_joint.location;
	velocity *= 1.0f - r_joint.drag;
	velocity += p_gravity_direction * r_joint.gravity * dt;
	r_joint.location += velocity * dt;

	// 캐릭터가 등속도로 달리는 동안에는 천이 뒤로 밀리지 않게 한다.
	r_joint.location += (p_move - prev_step_move) * (1.0f - share);
	r_joint.location += (p_turn.xform(r_joint.prev_location) - prev_step_turn.xform(r_joint.prev_location)) * (1.0f - turn_share);
}

void BoneClothSimulator3D::_move_root(Chain &r_chain, const Vector3 &p_move, const Quaternion &p_turn) {
	Joint &root = r_chain.joints[0];
	_move_joint(root, Vector3(), p_move, p_turn);

	Vector3 offset = ((root.location - root.pose_location) * (1.0f - root_spring_stiffness)).limit_length(root_spring_limit * units_per_meter);

	// joint가 root 하나뿐이면 bone 방향이 없어서 root_spring_limit만 쓴다.
	if (r_chain.joints.size() > 1) {
		const Vector3 forward = (r_chain.joints[1].pose_location - root.pose_location).normalized();
		const float along = offset.dot(forward);
		const float forward_limit = root_spring_forward_limit * units_per_meter;
		if (along > forward_limit) {
			offset -= forward * (along - forward_limit);
		}
	}
	root.location = root.pose_location + offset;
}

// root 아래 joint를 움직인다(_move_joint). 그리고 stiffness만큼 pose 쪽으로 당긴다.
void BoneClothSimulator3D::_simulate(Chain &p_chain, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		_move_joint(joint, p_gravity_direction, p_move, p_turn);

		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * joint.stiffness;
	}
}

// XPBD: 위치를 직접 업데이트하여 거리를 맞추는 계산법
// link와 rest의 거리의 차이를 구하고, 그 차이의 절반씩 두 joint를 서로 밀고 당긴다.
void BoneClothSimulator3D::_solve_links() {
	_space_links();
	if (link_stiffness <= 0.0f) {
		return;
	}

	const float compliance = LINK_COMPLIANCE / float(STEP * STEP);

	for (const Link &link : links) {
		Joint &joint_a = chains[link.chain_a].joints[link.depth];
		Joint &joint_b = chains[link.chain_b].joints[link.depth];
		const Vector3 delta = joint_b.location - joint_a.location;
		const float distance = delta.length();
		if (distance <= 0.0f) {
			continue;
		}

		const float delta_lambda = (distance - link.length) / (2.0f + compliance) * link_stiffness;
		const Vector3 correction = delta / distance * delta_lambda;
		joint_a.location += correction;
		joint_b.location -= correction;
	}
}

// 한 joint를 민 양들을 하나로 합친다.
static Vector3 combine_pushes(const LocalVector<Vector3> &p_pushes) {
	Vector3 direction_sum;
	int count = 0;
	for (const Vector3 &push : p_pushes) {
		if (!push.is_zero_approx()) {
			direction_sum += push.normalized();
			count++;
		}
	}

	if (count == 0 || direction_sum.is_zero_approx()) {
		return Vector3();
	}

	const float agreement = MIN((direction_sum / float(count)).length(), 1.0f);
	const Vector3 direction = direction_sum.normalized();
	float magnitude = 0.0f;
	for (const Vector3 &push : p_pushes) {
		magnitude = MAX(magnitude, push.dot(direction));
	}

	return direction * magnitude * agreement;
}

// collision에 밀린 joint는 그 collision 쪽으로는 옮기지 않는다. 몸 표면을 따라 미끄러지는 몫만 남긴다.
static Vector3 slide_on_contact(const LocalVector<Vector3> &p_pushes, const Vector3 &p_move) {
	const Vector3 push = combine_pushes(p_pushes);
	if (push.is_zero_approx()) {
		return p_move;
	}

	const Vector3 normal = push.normalized();
	return p_move - normal * MIN(p_move.dot(normal), 0.0f);
}

// 이웃한 두 가닥이 두 joint의 반지름 합보다 가까워지면 떼어 놓는다. 멀어지는 것은 그대로 둔다.
/*
	예: 긴 머리, 달리기.
	가닥들이 같이 뒤로 날리면서 끝이 한 점으로 모인다(붓처럼). 실제 머리카락은 가닥끼리 밀어내서 덩어리 사이에 틈이 남는다.
	어깨 위에 누운 쪽(chain의 위쪽)은 몸이 모양을 잡으므로, 보통 curve로 끝 쪽에만 준다.
*/
void BoneClothSimulator3D::_space_links() {
	if (link_spacing <= 0.0f) {
		return;
	}

	for (const Link &link : links) {
		Joint &joint_a = chains[link.chain_a].joints[link.depth];
		Joint &joint_b = chains[link.chain_b].joints[link.depth];
		const float strength = (joint_a.spacing + joint_b.spacing) * 0.5f;
		const float spacing = joint_a.radius + joint_b.radius;
		const Vector3 delta = joint_b.location - joint_a.location;
		const float distance = delta.length();
		if (strength <= 0.0f || distance <= 0.0f || distance >= spacing) {
			continue;
		}

		// 반지름 합에 막 들어왔을 때는 약하게, 더 가까워질수록 세게 민다. 한 번에 밀면 가닥이 떨린다.
		const float ramp = MIN((1.0f - distance / spacing) / LINK_SPACING_RAMP, 1.0f);
		const Vector3 correction = delta / distance * ((distance - spacing) * ramp * strength * 0.5f);
		joint_a.location += slide_on_contact(joint_a.collision_pushes, correction);
		joint_b.location += slide_on_contact(joint_b.collision_pushes, -correction);
	}
}

// 선분 하나(r_a ~ r_b)를 collision 밖으로 밀 양을 구해서, 두 joint의 collision_pushes에 더한다(여기서 옮기지는 않는다).
// collision마다 같은 시작 위치에서 민다. 합치는 것은 combine_pushes.
//
//     손 -->  |치마|  <-- 허벅지
//
// 위처럼 두 collision이 반대로 밀면 두 방향이 서로 지워져 선분은 거의 움직이지 않는다.
void BoneClothSimulator3D::_collide_segment(Joint &r_a, Joint &r_b, float p_radius, const LocalVector<PosedCollision> &p_collisions, const LocalVector<bool> &p_collides_a, const LocalVector<bool> &p_collides_b, Vector3 Joint::*p_field, bool p_split) {
	const Vector3 &location_a = r_a.*p_field;
	const Vector3 &location_b = r_b.*p_field;
	for (uint32_t c = 0; c < p_collisions.size(); c++) {
		if (!p_collides_a[c] || !p_collides_b[c]) {
			continue;
		}

		Vector3 a = location_a;
		Vector3 b = location_b;
		p_collisions[c].collision->collide_segment(a, b, p_radius, p_collisions[c].mirror);
		if (a == location_a && b == location_b) {
			continue;
		}

		// p_split(link_split을 켠 link): 양 끝 joint가 이 collision의 반대편에 있으면 link를 밀지 않는다.
		/*
			예: 긴 머리, 어깨 collision.
			앞으로 넘어온 가닥과 등 뒤 가닥을 잇는 link가 어깨를 가로지른다. 이 link를 밀면 두 가닥이 어깨 위로 들리고,
			각도 제한이 다시 끌어내린다. 매 step 밀고 당겨서 가닥이 떨린다.
			밀지 않으면 두 가닥이 어깨 앞뒤로 갈라진다. 가닥 자체(bone)는 그대로 collision에 밀린다.
		*/
		if (p_split && p_collisions[c].collision->separates(location_a, location_b, p_collisions[c].mirror)) {
			continue;
		}

		// rest_overlap을 끄면 rest 자세에서 겹친 만큼도 봐주지 않고, 다른 곳처럼 반지름만큼 민다.
		const float rest_depth = rest_overlap ? p_collisions[c].collision->get_rest_depth(r_a.rest_location, r_b.rest_location, p_radius, p_collisions[c].mirror) : 0.0f;
		if (rest_depth > 0.0f) {
			a = location_a;
			b = location_b;
			p_collisions[c].collision->collide_segment(a, b, p_radius - rest_depth, p_collisions[c].mirror);
		}

		const Vector3 moved = (a - location_a) + (b - location_b);
		if (moved.is_zero_approx()) {
			continue;
		}

		r_a.collision_pushes.push_back(a - location_a);
		r_b.collision_pushes.push_back(b - location_b);
	}
}

// collision이 천을 몸 밖으로 민다. root는 옮기지 않는다(pose나 parent chain을 따라간다).
// p_field: 미는 위치(location 또는 display_location).
void BoneClothSimulator3D::_collide(const LocalVector<PosedCollision> &p_collisions, Vector3 Joint::*p_field) {
	for (Chain &chain : chains) {
		for (Joint &joint : chain.joints) {
			joint.collision_pushes.clear();
		}
	}

	for (Chain &chain : chains) {
		for (uint32_t i = 1; i < chain.joints.size(); i++) {
			Joint &parent = chain.joints[i - 1];
			Joint &joint = chain.joints[i];
			_collide_segment(parent, joint, (parent.radius + joint.radius) * 0.5f, p_collisions, chain.collides, chain.collides, p_field, false);
		}
	}

	for (const Link &link : links) {
		Chain &chain_a = chains[link.chain_a];
		Chain &chain_b = chains[link.chain_b];
		Joint &joint_a = chain_a.joints[link.depth];
		Joint &joint_b = chain_b.joints[link.depth];
		_collide_segment(joint_a, joint_b, (joint_a.radius + joint_b.radius) * 0.5f, p_collisions, chain_a.collides, chain_b.collides, p_field, link_split);
	}

	for (Chain &chain : chains) {
		for (uint32_t i = 1; i < chain.joints.size(); i++) {
			Joint &joint = chain.joints[i];
			const Vector3 push = combine_pushes(joint.collision_pushes);
			if (p_field == &Joint::location && !push.is_zero_approx()) {
				const Vector3 normal = push.normalized();
				Vector3 velocity = joint.location - joint.prev_location;
				const float into = velocity.dot(normal);
				if (into < 0.0f) {
					velocity -= normal * into;
				}
				joint.location += push;
				joint.prev_location = joint.location - velocity;
			} else {
				joint.*p_field += push;
			}
		}
	}
}

// joint가 pose 방향에서 p_limit 각도보다 많이 기울면, 넘친 각도에 p_stiffness를 곱한 만큼 되돌린다.
// p_stiffness가 1이면 딱 p_limit까지 돌린다. 1보다 작으면 넘친 채로 남고, 다음 step들에 걸쳐 조금씩 돌아온다.
/*
	예: 긴 머리, 고개를 옆 위로 들 때.
	애니메이션의 머리카락 방향이 어깨 속을 향한다. 각도 제한이 한 번에 p_limit까지 당기면 joint가 어깨 안으로 크게 들어가고,
	collision이 그것을 밀어낸다. 한 step에 당기는 거리가 collision보다 커지면 joint가 어깨를 넘어가서 반대편으로 밀려난다(튄다).
	조금씩 되돌리면 joint가 어깨 위를 미끄러져 넘어간다.
*/
static Vector3 limit_to_cone(const Vector3 &p_offset, const Vector3 &p_pose_vector, float p_limit, float p_stiffness) {
	const Vector3 pose_direction = p_pose_vector.normalized();
	const Vector3 direction = p_offset.normalized();
	const Vector3 axis = pose_direction.cross(direction);
	const float angle = Math::atan2(axis.length(), pose_direction.dot(direction));
	if (angle <= p_limit) {
		return p_offset;
	}

	// offset이 pose 방향과 정반대면 외적이 0이 되어 회전축을 구할 수 없다. 이때는 pose 방향에 수직인 아무 축으로 돌린다.
	// rig마다 bone 축 방향이 달라서 특정 bone을 축으로 쓰지 않는다.
	const Vector3 turn_axis = axis.is_zero_approx() ? pose_direction.get_any_perpendicular() : axis.normalized();
	return p_offset.rotated(turn_axis, (p_limit - angle) * p_stiffness);
}

// link와 collision이 joint를 밀면 bone이 늘어나거나 각도 제한(limit_to_cone)을 넘을 수 있다.
// 그래서 step 마지막에 bone 길이와 각도를 다시 맞춘다.
void BoneClothSimulator3D::_restore_limits_and_lengths(Chain &p_chain) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.pose_location - parent.pose_location;

		// inspector의 limit_angle이 0이면 각도 제한을 비활성화한다.
		// curve 때문에 0이 된 joint는 비활성화하지 않고 pose 방향에 고정한다.
		Vector3 offset = joint.location - parent.location;
		if (limit_angle > 0.0f) {
			offset = limit_to_cone(offset, pose_vector, joint.limit_angle, limit_angle_stiffness);
		}

		joint.location = parent.location + offset.normalized() * pose_vector.length();
	}
}

// step이 frame보다 드물 때(슬로 모션, 60Hz보다 빠른 모니터) joint가 멈췄다 튀지 않고 매 frame 움직이게 한다.
void BoneClothSimulator3D::_update_display(Chain &p_chain, float p_follow) {
	// root spring이 root를 pose에서 옮긴 만큼(location - pose_location)을 이번 frame의 pose에 더한다. 스프링이 없으면 0.
	Joint &root = p_chain.joints[0];
	root.display_location = p_chain.parent_chain >= 0 ? chains[p_chain.parent_chain].joints[p_chain.parent_joint].display_location : root.current_pose_location + (root.location - root.pose_location);

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Vector3 next_location = joint.location + (joint.location - joint.prev_location);
		joint.display_location = joint.display_location.lerp(next_location, p_follow);
	}
}

// frame의 마지막 단계. joint 위치를 bone 회전으로 바꿔 skeleton에 넣는다(mesh는 bone 회전으로 움직인다).
void BoneClothSimulator3D::_write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain) {
	Quaternion turn; // 위 bone들이 pose보다 더 회전한 양. 처음은 회전 없음.
	for (uint32_t i = p_chain.parent_chain >= 0 ? 2 : 1; i < p_chain.joints.size(); i++) {
		const Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.animated_location - parent.animated_location;
		const Vector3 display_vector = joint.display_location - parent.display_location;
		turn = parent.branched ? Quaternion() : Quaternion(turn.xform(pose_vector), display_vector) * turn;
		p_skeleton->set_bone_global_pose(parent.bone, Transform3D(Basis(turn) * parent.pose_basis, parent.display_location));
	}
}

void BoneClothSimulator3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("apply_preset", "preset"), &BoneClothSimulator3D::apply_preset);
	ClassDB::bind_method(D_METHOD("make_preset"), &BoneClothSimulator3D::make_preset);

	ClassDB::bind_method(D_METHOD("set_chain_count", "count"), &BoneClothSimulator3D::set_chain_count);
	ClassDB::bind_method(D_METHOD("get_chain_count"), &BoneClothSimulator3D::get_chain_count);
	ClassDB::bind_method(D_METHOD("set_root_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_root_bone_name);
	ClassDB::bind_method(D_METHOD("get_root_bone_name", "index"), &BoneClothSimulator3D::get_root_bone_name);
	ClassDB::bind_method(D_METHOD("set_root_bone", "index", "bone"), &BoneClothSimulator3D::set_root_bone);
	ClassDB::bind_method(D_METHOD("get_root_bone", "index"), &BoneClothSimulator3D::get_root_bone);
	ClassDB::bind_method(D_METHOD("set_end_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_end_bone_name);
	ClassDB::bind_method(D_METHOD("get_end_bone_name", "index"), &BoneClothSimulator3D::get_end_bone_name);
	ClassDB::bind_method(D_METHOD("set_end_bone", "index", "bone"), &BoneClothSimulator3D::set_end_bone);
	ClassDB::bind_method(D_METHOD("get_end_bone", "index"), &BoneClothSimulator3D::get_end_bone);
	ClassDB::bind_method(D_METHOD("set_extend_end_bone", "index", "enabled"), &BoneClothSimulator3D::set_extend_end_bone);
	ClassDB::bind_method(D_METHOD("is_end_bone_extended", "index"), &BoneClothSimulator3D::is_end_bone_extended);
	ClassDB::bind_method(D_METHOD("set_end_bone_direction", "index", "bone_direction"), &BoneClothSimulator3D::set_end_bone_direction);
	ClassDB::bind_method(D_METHOD("get_end_bone_direction", "index"), &BoneClothSimulator3D::get_end_bone_direction);
	ClassDB::bind_method(D_METHOD("set_end_bone_length", "index", "length"), &BoneClothSimulator3D::set_end_bone_length);
	ClassDB::bind_method(D_METHOD("get_end_bone_length", "index"), &BoneClothSimulator3D::get_end_bone_length);

	ClassDB::bind_method(D_METHOD("set_sync_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_sync_bone_name);
	ClassDB::bind_method(D_METHOD("get_sync_bone_name", "index"), &BoneClothSimulator3D::get_sync_bone_name);
	ClassDB::bind_method(D_METHOD("set_sync_bone", "index", "bone"), &BoneClothSimulator3D::set_sync_bone);
	ClassDB::bind_method(D_METHOD("get_sync_bone", "index"), &BoneClothSimulator3D::get_sync_bone);

	ClassDB::bind_method(D_METHOD("set_enable_all_child_collisions", "index", "enabled"), &BoneClothSimulator3D::set_enable_all_child_collisions);
	ClassDB::bind_method(D_METHOD("are_all_child_collisions_enabled", "index"), &BoneClothSimulator3D::are_all_child_collisions_enabled);
	ClassDB::bind_method(D_METHOD("set_exclude_collision_path", "index", "collision", "node_path"), &BoneClothSimulator3D::set_exclude_collision_path);
	ClassDB::bind_method(D_METHOD("get_exclude_collision_path", "index", "collision"), &BoneClothSimulator3D::get_exclude_collision_path);
	ClassDB::bind_method(D_METHOD("set_exclude_collision_count", "index", "count"), &BoneClothSimulator3D::set_exclude_collision_count);
	ClassDB::bind_method(D_METHOD("get_exclude_collision_count", "index"), &BoneClothSimulator3D::get_exclude_collision_count);
	ClassDB::bind_method(D_METHOD("clear_exclude_collisions", "index"), &BoneClothSimulator3D::clear_exclude_collisions);
	ClassDB::bind_method(D_METHOD("set_collision_path", "index", "collision", "node_path"), &BoneClothSimulator3D::set_collision_path);
	ClassDB::bind_method(D_METHOD("get_collision_path", "index", "collision"), &BoneClothSimulator3D::get_collision_path);
	ClassDB::bind_method(D_METHOD("set_collision_count", "index", "count"), &BoneClothSimulator3D::set_collision_count);
	ClassDB::bind_method(D_METHOD("get_collision_count", "index"), &BoneClothSimulator3D::get_collision_count);
	ClassDB::bind_method(D_METHOD("clear_collisions", "index"), &BoneClothSimulator3D::clear_collisions);

	ClassDB::bind_method(D_METHOD("set_link_mode", "mode"), &BoneClothSimulator3D::set_link_mode);
	ClassDB::bind_method(D_METHOD("get_link_mode"), &BoneClothSimulator3D::get_link_mode);
	ClassDB::bind_method(D_METHOD("set_link_stiffness", "stiffness"), &BoneClothSimulator3D::set_link_stiffness);
	ClassDB::bind_method(D_METHOD("get_link_stiffness"), &BoneClothSimulator3D::get_link_stiffness);
	ClassDB::bind_method(D_METHOD("set_link_split_enabled", "enabled"), &BoneClothSimulator3D::set_link_split_enabled);
	ClassDB::bind_method(D_METHOD("is_link_split_enabled"), &BoneClothSimulator3D::is_link_split_enabled);
	ClassDB::bind_method(D_METHOD("set_link_spacing", "spacing"), &BoneClothSimulator3D::set_link_spacing);
	ClassDB::bind_method(D_METHOD("get_link_spacing"), &BoneClothSimulator3D::get_link_spacing);
	ClassDB::bind_method(D_METHOD("set_link_spacing_damping_curve", "curve"), &BoneClothSimulator3D::set_link_spacing_damping_curve);
	ClassDB::bind_method(D_METHOD("get_link_spacing_damping_curve"), &BoneClothSimulator3D::get_link_spacing_damping_curve);

	ClassDB::bind_method(D_METHOD("set_drag", "drag"), &BoneClothSimulator3D::set_drag);
	ClassDB::bind_method(D_METHOD("get_drag"), &BoneClothSimulator3D::get_drag);
	ClassDB::bind_method(D_METHOD("set_drag_damping_curve", "curve"), &BoneClothSimulator3D::set_drag_damping_curve);
	ClassDB::bind_method(D_METHOD("get_drag_damping_curve"), &BoneClothSimulator3D::get_drag_damping_curve);

	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothSimulator3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothSimulator3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_stiffness_damping_curve", "curve"), &BoneClothSimulator3D::set_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("get_stiffness_damping_curve"), &BoneClothSimulator3D::get_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothSimulator3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothSimulator3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius_damping_curve", "curve"), &BoneClothSimulator3D::set_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("get_radius_damping_curve"), &BoneClothSimulator3D::get_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("set_rest_overlap_enabled", "enabled"), &BoneClothSimulator3D::set_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("is_rest_overlap_enabled"), &BoneClothSimulator3D::is_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothSimulator3D::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothSimulator3D::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_limit_angle_damping_curve", "curve"), &BoneClothSimulator3D::set_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("get_limit_angle_damping_curve"), &BoneClothSimulator3D::get_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("set_limit_angle_stiffness", "stiffness"), &BoneClothSimulator3D::set_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("get_limit_angle_stiffness"), &BoneClothSimulator3D::get_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity_damping_curve", "curve"), &BoneClothSimulator3D::set_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("get_gravity_damping_curve"), &BoneClothSimulator3D::get_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity_direction", "direction"), &BoneClothSimulator3D::set_gravity_direction);
	ClassDB::bind_method(D_METHOD("get_gravity_direction"), &BoneClothSimulator3D::get_gravity_direction);
	ClassDB::bind_method(D_METHOD("set_sync", "sync"), &BoneClothSimulator3D::set_sync);
	ClassDB::bind_method(D_METHOD("get_sync"), &BoneClothSimulator3D::get_sync);
	ClassDB::bind_method(D_METHOD("set_sync_damping_curve", "curve"), &BoneClothSimulator3D::set_sync_damping_curve);
	ClassDB::bind_method(D_METHOD("get_sync_damping_curve"), &BoneClothSimulator3D::get_sync_damping_curve);

	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothSimulator3D::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothSimulator3D::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia_smoothing", "smoothing"), &BoneClothSimulator3D::set_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("get_inertia_smoothing"), &BoneClothSimulator3D::get_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("set_depth_inertia", "depth_inertia"), &BoneClothSimulator3D::set_depth_inertia);
	ClassDB::bind_method(D_METHOD("get_depth_inertia"), &BoneClothSimulator3D::get_depth_inertia);
	ClassDB::bind_method(D_METHOD("set_local_movement_speed_limit", "limit"), &BoneClothSimulator3D::set_local_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_local_movement_speed_limit"), &BoneClothSimulator3D::get_local_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_local_rotation_speed_limit", "limit"), &BoneClothSimulator3D::set_local_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_local_rotation_speed_limit"), &BoneClothSimulator3D::get_local_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_movement_speed_limit", "limit"), &BoneClothSimulator3D::set_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_movement_speed_limit"), &BoneClothSimulator3D::get_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_rotation_speed_limit", "limit"), &BoneClothSimulator3D::set_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_rotation_speed_limit"), &BoneClothSimulator3D::get_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_joint_speed_limit", "limit"), &BoneClothSimulator3D::set_joint_speed_limit);
	ClassDB::bind_method(D_METHOD("get_joint_speed_limit"), &BoneClothSimulator3D::get_joint_speed_limit);
	ClassDB::bind_method(D_METHOD("set_teleport_distance", "distance"), &BoneClothSimulator3D::set_teleport_distance);
	ClassDB::bind_method(D_METHOD("get_teleport_distance"), &BoneClothSimulator3D::get_teleport_distance);
	ClassDB::bind_method(D_METHOD("set_teleport_angle", "angle"), &BoneClothSimulator3D::set_teleport_angle);
	ClassDB::bind_method(D_METHOD("get_teleport_angle"), &BoneClothSimulator3D::get_teleport_angle);
	ClassDB::bind_method(D_METHOD("set_root_spring_stiffness", "stiffness"), &BoneClothSimulator3D::set_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("get_root_spring_stiffness"), &BoneClothSimulator3D::get_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("set_root_spring_limit", "limit"), &BoneClothSimulator3D::set_root_spring_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_limit"), &BoneClothSimulator3D::get_root_spring_limit);
	ClassDB::bind_method(D_METHOD("set_root_spring_forward_limit", "limit"), &BoneClothSimulator3D::set_root_spring_forward_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_forward_limit"), &BoneClothSimulator3D::get_root_spring_forward_limit);

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), "set_link_mode", "get_link_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "link_split"), "set_link_split_enabled", "is_link_split_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_spacing/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_spacing", "get_link_spacing");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "link_spacing/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_link_spacing_damping_curve", "get_link_spacing_damping_curve");

	// chain 위치마다 값을 다르게 주기 위해 값마다 curve가 있다(<name>/value, <name>/damping_curve).
	// stiffness는 step마다 joint를 pose 쪽으로 당기는 비율이다(0에서 1).
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_damping_curve", "get_drag_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_stiffness_damping_curve", "get_stiffness_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_radius_damping_curve", "get_radius_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "radius/rest_overlap"), "set_rest_overlap_enabled", "is_rest_overlap_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_limit_angle_damping_curve", "get_limit_angle_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_limit_angle_stiffness", "get_limit_angle_stiffness");

	// gravity/value는 가속도(m/s^2)다. step마다 속도에 value * dt를 더한다.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_gravity_damping_curve", "get_gravity_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), "set_gravity_direction", "get_gravity_direction");

	// sync/value는 chain의 sync bone이 움직인 양 중 pose를 옮기는 비율이다. sync bone이 없는 chain에는 쓰지 않는다.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sync/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_sync", "get_sync");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "sync/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_sync_damping_curve", "get_sync_damping_curve");

	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia_smoothing", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia_smoothing", "get_inertia_smoothing");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_depth_inertia", "get_depth_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_local_movement_speed_limit", "get_local_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_local_rotation_speed_limit", "get_local_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_speed_limit", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m/s"), "set_joint_speed_limit", "get_joint_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");

	// root_spring_stiffness는 step마다 root를 pose 쪽으로 당기는 비율이다(0에서 1, 0이면 root는 pose에 고정).
	ADD_GROUP("Root Spring", "root_spring_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_root_spring_stiffness", "get_root_spring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_limit", "get_root_spring_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_forward_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_forward_limit", "get_root_spring_forward_limit");
	ADD_GROUP("", "");

	// inspector에서 chains/N/... property를 Chains 목록 하나로 묶는다(항목 추가, 삭제 버튼).
	// godot-cpp에는 ADD_ARRAY_COUNT가 없어서 ARRAY usage와 "label,prefix"로 대신한다.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), "set_chain_count", "get_chain_count");

	BIND_ENUM_CONSTANT(LINK_MODE_NONE);
	BIND_ENUM_CONSTANT(LINK_MODE_SEQUENTIAL);
	BIND_ENUM_CONSTANT(LINK_MODE_LOOP);
}

// 설정 밖에서 생기는 변화에도 configuration warning을 다시 확인한다.
// 자식 collision을 더하거나 빼거나 옮길 때, 그리고 skeleton이나 그 위 node의 scale이 바뀔 때다.
// transform 알림은 scale 경고에만 쓴다. 경고는 에디터에서만 보이므로 에디터에서만 켠다(기본은 꺼져 있다).
void BoneClothSimulator3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint()) {
				set_notify_transform(true);
			}
		} break;
		case NOTIFICATION_CHILD_ORDER_CHANGED:
		case NOTIFICATION_TRANSFORM_CHANGED: {
			update_configuration_warnings();
		} break;
	}
}

// chain별 property(chains/N/...)를 넣고 읽는다 (N은 chain 번호).
bool BoneClothSimulator3D::_set(const StringName &p_name, const Variant &p_value) {
	const String path = p_name;
	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone_name") {
		set_root_bone_name(which, p_value);
	} else if (what == "root_bone") {
		set_root_bone(which, p_value);
	} else if (what == "end_bone_name") {
		set_end_bone_name(which, p_value);
	} else if (what == "end_bone") {
		const String option = path.get_slice("/", 3);
		if (option.is_empty()) {
			set_end_bone(which, p_value);
		} else if (option == "direction") {
			set_end_bone_direction(which, BoneDirection(int(p_value)));
		} else if (option == "length") {
			set_end_bone_length(which, p_value);
		} else {
			return false;
		}
	} else if (what == "extend_end_bone") {
		set_extend_end_bone(which, p_value);
	} else if (what == "sync_bone_name") {
		set_sync_bone_name(which, p_value);
	} else if (what == "sync_bone") {
		set_sync_bone(which, p_value);
	} else if (what == "enable_all_child_collisions") {
		set_enable_all_child_collisions(which, p_value);
	} else if (what == "exclude_collision_count") {
		set_exclude_collision_count(which, p_value);
	} else if (what == "exclude_collisions") {
		set_exclude_collision_path(which, path.get_slice("/", 3).to_int(), p_value);
	} else if (what == "collision_count") {
		set_collision_count(which, p_value);
	} else if (what == "collisions") {
		set_collision_path(which, path.get_slice("/", 3).to_int(), p_value);
	} else {
		return false;
	}

	return true;
}

bool BoneClothSimulator3D::_get(const StringName &p_name, Variant &r_ret) const {
	const String path = p_name;
	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone_name") {
		r_ret = get_root_bone_name(which);
	} else if (what == "root_bone") {
		r_ret = get_root_bone(which);
	} else if (what == "end_bone_name") {
		r_ret = get_end_bone_name(which);
	} else if (what == "end_bone") {
		const String option = path.get_slice("/", 3);
		if (option.is_empty()) {
			r_ret = get_end_bone(which);
		} else if (option == "direction") {
			r_ret = int(get_end_bone_direction(which));
		} else if (option == "length") {
			r_ret = get_end_bone_length(which);
		} else {
			return false;
		}
	} else if (what == "extend_end_bone") {
		r_ret = is_end_bone_extended(which);
	} else if (what == "sync_bone_name") {
		r_ret = get_sync_bone_name(which);
	} else if (what == "sync_bone") {
		r_ret = get_sync_bone(which);
	} else if (what == "enable_all_child_collisions") {
		r_ret = are_all_child_collisions_enabled(which);
	} else if (what == "exclude_collision_count") {
		r_ret = get_exclude_collision_count(which);
	} else if (what == "exclude_collisions") {
		r_ret = get_exclude_collision_path(which, path.get_slice("/", 3).to_int());
	} else if (what == "collision_count") {
		r_ret = get_collision_count(which);
	} else if (what == "collisions") {
		r_ret = get_collision_path(which, path.get_slice("/", 3).to_int());
	} else {
		return false;
	}

	return true;
}

// bone 이름을 직접 치지 않도록 skeleton bone 목록을 drop-down으로 보여 준다.
// bone index는 내부적으로 저장만 하고 inspector에는 보이지 않는다.
void BoneClothSimulator3D::_get_property_list(List<PropertyInfo> *p_list) const {
	Skeleton3D *skeleton = get_skeleton();
	const PropertyHint hint = skeleton ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE;
	const String bone_names = skeleton ? String(skeleton->get_concatenated_bone_names()) : String();

	for (uint32_t i = 0; i < chains.size(); i++) {
		const String path = "chains/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("root_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("root_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("end_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("end_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));

		// 안쓰는 값은 숨긴다.
		const bool has_end_bone = chains[i].end_bone != -1;
		const uint32_t extend_usage = has_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t tip_usage = has_end_bone && chains[i].extend_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + String("extend_end_bone"), PROPERTY_HINT_NONE, "", extend_usage));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("end_bone/direction"), PROPERTY_HINT_ENUM, "+X,-X,+Y,-Y,+Z,-Z,FromParent", tip_usage));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + String("end_bone/length"), PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", tip_usage));
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("sync_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("sync_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));

		const Chain &chain = chains[i];
		const bool all = chain.enable_all_child_collisions;
		const uint32_t exclude_usage = all ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t collision_usage = all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + String("enable_all_child_collisions")));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("exclude_collision_count"), PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY : PROPERTY_USAGE_NONE, String("Exclude Collisions,") + path + String("exclude_collisions/")));
		for (uint32_t j = 0; j < chain.exclude_collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + String("exclude_collisions/") + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", exclude_usage));
		}
		p_list->push_back(PropertyInfo(Variant::INT, path + String("collision_count"), PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, String("Collisions,") + path + String("collisions/")));
		for (uint32_t j = 0; j < chain.collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + String("collisions/") + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", collision_usage));
		}
	}
}

// inspector에 노출할 지 말 지 여부를 여기서 최종적으로 정한다.
void BoneClothSimulator3D::_validate_property(PropertyInfo &p_property) const {
	if (link_mode == LINK_MODE_NONE && (p_property.name == StringName("link_stiffness") || p_property.name == StringName("link_split") || String(p_property.name).begins_with("link_spacing/"))) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

// preset 값을 이름별로 이 node에 복사한다 (chain, link mode, collision은 preset에 없다).
void BoneClothSimulator3D::apply_preset(const Ref<BoneClothPreset> &p_preset) {
	ERR_FAIL_COND_MSG(p_preset.is_null(), "No preset to apply.");
	for (const char *name : BoneClothPreset::VALUES) {
		set(name, BoneClothPreset::copy_value(p_preset->get(name)));
	}
}

// 이 node의 값으로 새 preset을 만든다.
Ref<BoneClothPreset> BoneClothSimulator3D::make_preset() const {
	Ref<BoneClothPreset> made;
	made.instantiate();
	for (const char *name : BoneClothPreset::VALUES) {
		made->set(name, BoneClothPreset::copy_value(get(name)));
	}
	return made;
}

void BoneClothSimulator3D::set_chain_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	chains.resize(p_count);
	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_chain_count() const {
	return chains.size();
}

// bone 이름과 index는 짝이다. 이름을 넣으면 skeleton에서 index를 찾고, index를 넣으면 이름을 찾는다.
void BoneClothSimulator3D::set_root_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].root_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		// 스크립트에서 이름을 잘못 넣어도, 에러임을 알 수 있게 한 번 출력한다.
		// 에디터에서는 configuration warning도 뜬다.
		const int found = skeleton->find_bone(p_bone_name);
		if (found < 0 && !p_bone_name.is_empty()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": root bone not found: '" + p_bone_name + "'.");
		}

		set_root_bone(p_index, found);
	}
}

String BoneClothSimulator3D::get_root_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].root_bone_name;
}

// -1은 "bone 없음"이다.
// 이름은 남기고, chain을 만들 때 "Root bone not found"를 출력한다.
void BoneClothSimulator3D::set_root_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.root_bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && p_bone >= 0) {
		if (p_bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": root bone index " + itos(p_bone) + " is out of range.");
			chain.root_bone = -1;
		} else {
			chain.root_bone_name = skeleton->get_bone_name(p_bone);
		}
	}

	joints_dirty = true;
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_root_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].root_bone;
}

void BoneClothSimulator3D::set_end_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].end_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(p_bone_name);
		if (found < 0 && !p_bone_name.is_empty()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": end bone not found: '" + p_bone_name + "'.");
		}

		set_end_bone(p_index, found);
	}
}

String BoneClothSimulator3D::get_end_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].end_bone_name;
}

void BoneClothSimulator3D::set_end_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.end_bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && p_bone >= 0) {
		if (p_bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": end bone index " + itos(p_bone) + " is out of range.");
			chain.end_bone = -1;
		} else {
			chain.end_bone_name = skeleton->get_bone_name(p_bone);
		}
	}

	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_end_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].end_bone;
}

void BoneClothSimulator3D::set_extend_end_bone(int p_index, bool p_enabled) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].extend_end_bone = p_enabled;
	joints_dirty = true;
	notify_property_list_changed();
}

bool BoneClothSimulator3D::is_end_bone_extended(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	return chains[p_index].extend_end_bone;
}

void BoneClothSimulator3D::set_end_bone_direction(int p_index, BoneDirection p_bone_direction) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_INDEX(int(p_bone_direction), int(BONE_DIRECTION_FROM_PARENT) + 1);
	chains[p_index].end_bone_direction = p_bone_direction;
	joints_dirty = true;
}

SkeletonModifier3D::BoneDirection BoneClothSimulator3D::get_end_bone_direction(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), BONE_DIRECTION_FROM_PARENT);
	return chains[p_index].end_bone_direction;
}

void BoneClothSimulator3D::set_end_bone_length(int p_index, float p_length) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].end_bone_length = p_length;
	joints_dirty = true;
}

float BoneClothSimulator3D::get_end_bone_length(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0.0f);
	return chains[p_index].end_bone_length;
}

// sync bone은 비워 둘 수 있다(-1). 그 chain은 sync로 옮기지 않는다.
void BoneClothSimulator3D::set_sync_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].sync_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(p_bone_name);
		if (found < 0 && !p_bone_name.is_empty()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": sync bone not found: '" + p_bone_name + "'.");
		}

		set_sync_bone(p_index, found);
	}
}

String BoneClothSimulator3D::get_sync_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].sync_bone_name;
}

void BoneClothSimulator3D::set_sync_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.sync_bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && p_bone >= 0) {
		if (p_bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Chain " + itos(p_index) + ": sync bone index " + itos(p_bone) + " is out of range.");
			chain.sync_bone = -1;
		} else {
			chain.sync_bone_name = skeleton->get_bone_name(p_bone);
		}
	}

	update_configuration_warnings();
}

int BoneClothSimulator3D::get_sync_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].sync_bone;
}

void BoneClothSimulator3D::set_enable_all_child_collisions(int p_index, bool p_enabled) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].enable_all_child_collisions = p_enabled;
	notify_property_list_changed();
	update_configuration_warnings();
}

bool BoneClothSimulator3D::are_all_child_collisions_enabled(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	return chains[p_index].enable_all_child_collisions;
}

void BoneClothSimulator3D::set_exclude_collision_count(int p_index, int p_count) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(!chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the exclude collision list is used only while enable_all_child_collisions is on.");
	ERR_FAIL_COND(p_count < 0);
	chains[p_index].exclude_collisions.resize(p_count);
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_exclude_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].exclude_collisions.size();
}

void BoneClothSimulator3D::set_exclude_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(!chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the exclude collision list is used only while enable_all_child_collisions is on.");
	ERR_FAIL_INDEX(p_collision, int(chains[p_index].exclude_collisions.size()));
	ERR_FAIL_COND_EDMSG(!_is_child_collision(p_node_path), "Collision must be a BoneClothCollision3D child of this BoneClothSimulator3D.");
	chains[p_index].exclude_collisions[p_collision] = p_node_path;
	update_configuration_warnings();
}

NodePath BoneClothSimulator3D::get_exclude_collision_path(int p_index, int p_collision) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), NodePath());
	ERR_FAIL_INDEX_V(p_collision, int(chains[p_index].exclude_collisions.size()), NodePath());
	return chains[p_index].exclude_collisions[p_collision];
}

void BoneClothSimulator3D::clear_exclude_collisions(int p_index) {
	set_exclude_collision_count(p_index, 0);
}

void BoneClothSimulator3D::set_collision_count(int p_index, int p_count) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the collision list is used only while enable_all_child_collisions is off.");
	ERR_FAIL_COND(p_count < 0);
	chains[p_index].collisions.resize(p_count);
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].collisions.size();
}

void BoneClothSimulator3D::set_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the collision list is used only while enable_all_child_collisions is off.");
	ERR_FAIL_INDEX(p_collision, int(chains[p_index].collisions.size()));
	ERR_FAIL_COND_EDMSG(!_is_child_collision(p_node_path), "Collision must be a BoneClothCollision3D child of this BoneClothSimulator3D.");
	chains[p_index].collisions[p_collision] = p_node_path;
	update_configuration_warnings();
}

NodePath BoneClothSimulator3D::get_collision_path(int p_index, int p_collision) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), NodePath());
	ERR_FAIL_INDEX_V(p_collision, int(chains[p_index].collisions.size()), NodePath());
	return chains[p_index].collisions[p_collision];
}

void BoneClothSimulator3D::clear_collisions(int p_index) {
	set_collision_count(p_index, 0);
}

void BoneClothSimulator3D::set_link_mode(LinkMode p_mode) {
	link_mode = p_mode;
	joints_dirty = true;
	notify_property_list_changed();
}

BoneClothSimulator3D::LinkMode BoneClothSimulator3D::get_link_mode() const {
	return link_mode;
}

void BoneClothSimulator3D::set_link_stiffness(float p_stiffness) {
	link_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_link_stiffness() const {
	return link_stiffness;
}

void BoneClothSimulator3D::set_link_split_enabled(bool p_enabled) {
	link_split = p_enabled;
}

bool BoneClothSimulator3D::is_link_split_enabled() const {
	return link_split;
}

void BoneClothSimulator3D::set_link_spacing(float p_spacing) {
	link_spacing = p_spacing;
}

float BoneClothSimulator3D::get_link_spacing() const {
	return link_spacing;
}

void BoneClothSimulator3D::set_link_spacing_damping_curve(const Ref<Curve> &p_curve) {
	link_spacing_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_link_spacing_damping_curve() const {
	return link_spacing_damping_curve;
}

void BoneClothSimulator3D::set_drag(float p_drag) {
	drag = p_drag;
}

float BoneClothSimulator3D::get_drag() const {
	return drag;
}

void BoneClothSimulator3D::set_drag_damping_curve(const Ref<Curve> &p_curve) {
	drag_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_drag_damping_curve() const {
	return drag_damping_curve;
}

void BoneClothSimulator3D::set_stiffness(float p_stiffness) {
	stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_stiffness() const {
	return stiffness;
}

void BoneClothSimulator3D::set_stiffness_damping_curve(const Ref<Curve> &p_curve) {
	stiffness_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_stiffness_damping_curve() const {
	return stiffness_damping_curve;
}

void BoneClothSimulator3D::set_radius(float p_radius) {
	radius = p_radius;
}

float BoneClothSimulator3D::get_radius() const {
	return radius;
}

void BoneClothSimulator3D::set_radius_damping_curve(const Ref<Curve> &p_curve) {
	radius_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_radius_damping_curve() const {
	return radius_damping_curve;
}

void BoneClothSimulator3D::set_rest_overlap_enabled(bool p_enabled) {
	rest_overlap = p_enabled;
}

bool BoneClothSimulator3D::is_rest_overlap_enabled() const {
	return rest_overlap;
}

void BoneClothSimulator3D::set_limit_angle(float p_angle) {
	limit_angle = p_angle;
}

float BoneClothSimulator3D::get_limit_angle() const {
	return limit_angle;
}

void BoneClothSimulator3D::set_limit_angle_damping_curve(const Ref<Curve> &p_curve) {
	limit_angle_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_limit_angle_damping_curve() const {
	return limit_angle_damping_curve;
}

void BoneClothSimulator3D::set_limit_angle_stiffness(float p_stiffness) {
	limit_angle_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_limit_angle_stiffness() const {
	return limit_angle_stiffness;
}

void BoneClothSimulator3D::set_gravity(float p_gravity) {
	gravity = p_gravity;
}

float BoneClothSimulator3D::get_gravity() const {
	return gravity;
}

void BoneClothSimulator3D::set_gravity_damping_curve(const Ref<Curve> &p_curve) {
	gravity_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_gravity_damping_curve() const {
	return gravity_damping_curve;
}

void BoneClothSimulator3D::set_gravity_direction(const Vector3 &p_direction) {
	ERR_FAIL_COND(p_direction.is_zero_approx());
	gravity_direction = p_direction;
}

Vector3 BoneClothSimulator3D::get_gravity_direction() const {
	return gravity_direction;
}

void BoneClothSimulator3D::set_sync(float p_sync) {
	sync = p_sync;
}

float BoneClothSimulator3D::get_sync() const {
	return sync;
}

void BoneClothSimulator3D::set_sync_damping_curve(const Ref<Curve> &p_curve) {
	sync_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_sync_damping_curve() const {
	return sync_damping_curve;
}

void BoneClothSimulator3D::set_inertia(float p_inertia) {
	inertia = p_inertia;
}

float BoneClothSimulator3D::get_inertia() const {
	return inertia;
}

void BoneClothSimulator3D::set_inertia_smoothing(float p_smoothing) {
	inertia_smoothing = p_smoothing;
}

float BoneClothSimulator3D::get_inertia_smoothing() const {
	return inertia_smoothing;
}

void BoneClothSimulator3D::set_depth_inertia(float p_depth_inertia) {
	depth_inertia = p_depth_inertia;
}

float BoneClothSimulator3D::get_depth_inertia() const {
	return depth_inertia;
}

void BoneClothSimulator3D::set_local_movement_speed_limit(float p_limit) {
	local_movement_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_local_movement_speed_limit() const {
	return local_movement_speed_limit;
}

void BoneClothSimulator3D::set_local_rotation_speed_limit(float p_limit) {
	local_rotation_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_local_rotation_speed_limit() const {
	return local_rotation_speed_limit;
}

void BoneClothSimulator3D::set_joint_speed_limit(float p_limit) {
	joint_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_joint_speed_limit() const {
	return joint_speed_limit;
}

void BoneClothSimulator3D::set_movement_speed_limit(float p_limit) {
	movement_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_movement_speed_limit() const {
	return movement_speed_limit;
}

void BoneClothSimulator3D::set_rotation_speed_limit(float p_limit) {
	rotation_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_rotation_speed_limit() const {
	return rotation_speed_limit;
}

void BoneClothSimulator3D::set_teleport_distance(float p_distance) {
	teleport_distance = p_distance;
}

float BoneClothSimulator3D::get_teleport_distance() const {
	return teleport_distance;
}

void BoneClothSimulator3D::set_teleport_angle(float p_angle) {
	teleport_angle = p_angle;
}

float BoneClothSimulator3D::get_teleport_angle() const {
	return teleport_angle;
}

void BoneClothSimulator3D::set_root_spring_stiffness(float p_stiffness) {
	root_spring_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_root_spring_stiffness() const {
	return root_spring_stiffness;
}

void BoneClothSimulator3D::set_root_spring_limit(float p_limit) {
	root_spring_limit = p_limit;
}

float BoneClothSimulator3D::get_root_spring_limit() const {
	return root_spring_limit;
}

void BoneClothSimulator3D::set_root_spring_forward_limit(float p_limit) {
	root_spring_forward_limit = p_limit;
}

float BoneClothSimulator3D::get_root_spring_forward_limit() const {
	return root_spring_forward_limit;
}

void BoneClothSimulator3D::reset() {
	needs_reset = true;
}

// 자식 collision도 크기 값(m)을 같은 배율로 skeleton 단위로 바꾼다.
float BoneClothSimulator3D::get_units_per_meter() const {
	return units_per_meter;
}

// p_bone이 p_root에서 p_end까지의 bone 경로 위에 있는지 확인한다(p_root, p_end 포함).
// p_end가 p_root 아래에 없으면 경로가 없으므로 false다.
static bool is_on_bone_path(const Skeleton3D *p_skeleton, int p_root, int p_end, int p_bone) {
	bool found = false;
	for (int bone = p_end; bone >= 0; bone = p_skeleton->get_bone_parent(bone)) {
		found = found || bone == p_bone;
		if (bone == p_root) {
			return found;
		}
	}

	return false;
}

// 잘못된 설정을 Scene 트리의 노란 삼각형으로 보여 준다(에디터 전용). 잘못된 chain은 움직이지 않는다.
// skeleton이 없을 때의 경고는 SkeletonModifier3D가 따로 띄운다.
PackedStringArray BoneClothSimulator3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	if (chains.is_empty()) {
		warnings.push_back("No chains. Add a chain under Chains and pick its root bone and end bone.");
	}

	// 천은 skeleton 공간에서 계산한다. 축마다 scale이 다르면 같은 캐릭터를 scale 1로 둘 때와 다르게 움직인다.
	// 균일한 scale은 경고하지 않는다(_get_units_per_meter가 길이 값을 맞춘다).
	const Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const Vector3 scale = skeleton->get_global_transform().basis.get_scale();
		if (!(Math::is_zero_approx(scale.x - scale.y) && Math::is_zero_approx(scale.y - scale.z))) {
			warnings.push_back(vformat("The Skeleton3D has a non-uniform scale %s, so the cloth moves differently. Make the scale of the Skeleton3D and its parents uniform (the same on all axes).", scale));
		}
	}

	for (uint32_t c = 0; c < chains.size(); c++) {
		const Chain &chain = chains[c];
		// 한 chain의 문제는 모아서 한 줄로 알린다.
		PackedStringArray problems;
		if (skeleton) {
			// 막 더한 chain처럼 둘 다 비어 있으면 한 줄로 알린다.
			const bool no_root = chain.root_bone_name.is_empty();
			const bool no_end = chain.end_bone_name.is_empty();
			if (no_root || no_end) {
				problems.push_back(vformat("pick %s", no_root && no_end ? "a root bone and an end bone" : (no_root ? "a root bone" : "an end bone")));
			}
			if (!no_root && chain.root_bone < 0) {
				problems.push_back(vformat("root bone '%s' is not in the skeleton", chain.root_bone_name));
			}
			if (!no_end && chain.end_bone < 0) {
				problems.push_back(vformat("end bone '%s' is not in the skeleton", chain.end_bone_name));
			}

			if (!chain.sync_bone_name.is_empty() && chain.sync_bone < 0) {
				problems.push_back(vformat("sync bone '%s' is not in the skeleton", chain.sync_bone_name));
			}

			const bool has_path = is_on_bone_path(skeleton, chain.root_bone, chain.end_bone, chain.root_bone);
			if (chain.root_bone >= 0 && chain.end_bone >= 0 && !has_path) {
				problems.push_back(vformat("end bone '%s' is neither root bone '%s' nor under it", chain.end_bone_name, chain.root_bone_name));
			}

			// root bone이 뒤 chain의 중간에 있고 앞 chain 어디에도 없으면 경고한다(_build_branches는 앞 chain에서만 찾는다).
			// root bone이 같은 chain끼리는 순서와 상관없다.
			bool has_trunk = false;
			for (uint32_t other = 0; other < c && has_path; other++) {
				has_trunk = has_trunk || is_on_bone_path(skeleton, chains[other].root_bone, chains[other].end_bone, chain.root_bone);
			}

			for (uint32_t other = c + 1; other < chains.size() && has_path && !has_trunk; other++) {
				const Chain &later = chains[other];
				if (later.root_bone != chain.root_bone && is_on_bone_path(skeleton, later.root_bone, later.end_bone, chain.root_bone)) {
					problems.push_back(vformat("root bone '%s' is on chain %d, so move chain %d above this chain", chain.root_bone_name, other, other));
					break;
				}
			}
		}

		// 쓰고 있는 목록만 확인한다(안 쓰는 목록은 저장되지 않는다).
		const bool all = chain.enable_all_child_collisions;
		const LocalVector<NodePath> &node_paths = all ? chain.exclude_collisions : chain.collisions;
		for (const NodePath &node_path : node_paths) {
			if (!_is_child_collision(node_path)) {
				problems.push_back(vformat("%s '%s' is not a BoneClothCollision3D child of this node", all ? "exclude collision" : "collision", node_path));
			}
		}

		if (!problems.is_empty()) {
			warnings.push_back(vformat("Chain %d: %s.", c, String("; ").join(problems)));
		}
	}

	return warnings;
}

// skeleton의 bone 순서가 바뀌어도 chain이 같은 bone을 찾도록, bone 이름으로 찾는다.
// SkeletonModifier3D가 skeleton을 찾았을 때와 skeleton이 수정됐을 때 부른다.
void BoneClothSimulator3D::_validate_bone_names() {
	for (uint32_t i = 0; i < chains.size(); i++) {
		const String root_name = chains[i].root_bone_name;
		if (!root_name.is_empty()) {
			set_root_bone_name(i, root_name);
		} else if (chains[i].root_bone != -1) {
			set_root_bone(i, chains[i].root_bone);
		}

		const String end_name = chains[i].end_bone_name;
		if (!end_name.is_empty()) {
			set_end_bone_name(i, end_name);
		} else if (chains[i].end_bone != -1) {
			set_end_bone(i, chains[i].end_bone);
		}

		const String sync_name = chains[i].sync_bone_name;
		if (!sync_name.is_empty()) {
			set_sync_bone_name(i, sync_name);
		} else if (chains[i].sync_bone != -1) {
			set_sync_bone(i, chains[i].sync_bone);
		}
	}
}

// 길이 값(m)을 skeleton 단위로 바꾸는 배율을 구한다.
// 기준은 skeleton이 들어 있는 씬(skeleton의 owner, 보통 import한 모델) 안에서 skeleton이 가진 scale이다.
// 모델 안의 scale은 단위를 맞추려고 있는 것이고, 모델에 준 scale은 캐릭터의 크기를 바꾸려고 준 것이기 때문이다.
/*
	Stage
	 +-- Model (skeleton의 owner, Stage에서 scale 2)
		  +-- Armature (scale 0.01, cm rig)
			   +-- Skeleton3D
					+-- BoneClothSimulator3D

	위 그림이면 모델 안의 scale이 0.01이므로 배율은 100이다(radius 0.03 m -> skeleton 단위 3).
	Model의 scale 2는 넣지 않는다. 그래서 천도 같이 2배가 되어, 크기를 키운 복사본처럼 움직인다(gravity 포함).
	simulator를 모델 씬에 넣든, Stage에서 모델 아래에 넣든 같다.
*/
// owner가 없으면(스크립트로 만든 skeleton) world scale을 쓴다.
float BoneClothSimulator3D::_get_units_per_meter(const Skeleton3D *p_skeleton) const {
	const Node *owner = p_skeleton->get_owner();
	Basis basis;
	for (const Node3D *node = p_skeleton; node && node != owner; node = Object::cast_to<Node3D>(node->get_parent())) {
		basis = node->get_basis() * basis;
	}

	return 1.0f / MAX(basis.get_uniform_scale(), 1e-6f);
}

void BoneClothSimulator3D::_process_modification_with_delta(double p_delta) {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	// Monitors 탭의 Bone Cloth > Time 값이다.
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	_rebuild_if_changed(skeleton);
	_read_frame(skeleton);

	const Transform3D skeleton_transform = skeleton->get_global_transform_interpolated();
	if (needs_reset) {
		_reset(skeleton_transform);
	}

	LocalVector<PosedCollision> collisions;
	_gather_collisions(collisions);
	_find_chain_collisions(collisions);

	if (p_delta > 0.0) {
		FrameMotion motion = _measure_motion(p_delta, skeleton_transform);
		_run_steps(motion, collisions);
		_settle_display(p_delta, motion, collisions);
	}

	_end_frame(skeleton);

	uint64_t joint_count = 0;
	for (const Chain &chain : chains) {
		joint_count += chain.joints.size();
	}

	monitor_joints.sum.fetch_add(joint_count, std::memory_order_relaxed);
	monitor_time.sum.fetch_add(std::chrono::nanoseconds(std::chrono::steady_clock::now() - start).count(), std::memory_order_relaxed);
}

void BoneClothSimulator3D::_rebuild_if_changed(Skeleton3D *p_skeleton) {
	// owner가 없으면 캐릭터가 돌 때 float 오차가 생기므로, 근사값으로 비교한다.
	const float units = _get_units_per_meter(p_skeleton);
	if (!Math::is_equal_approx(units, units_per_meter)) {
		units_per_meter = units;
		joints_dirty = true;
	}

	if (joints_dirty) {
		joints_dirty = false;
		needs_reset = true;
		for (Chain &chain : chains) {
			if (!_build_joints(p_skeleton, chain)) {
				chain.joints.clear();
			}
		}
		_build_branches();
		_build_links();
		_build_sync(p_skeleton);
	}
}

void BoneClothSimulator3D::_read_frame(Skeleton3D *p_skeleton) {
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty()) {
			_read_pose(p_skeleton, chain);
			_update_joint_settings(chain);
		}
	}

	_read_center(p_skeleton);
	_apply_sync(p_skeleton);
}

// 처음 돌 때, chain을 다시 만든 뒤, reset()을 부른 뒤에 부른다.
void BoneClothSimulator3D::_reset(const Transform3D &p_skeleton_transform) {
	needs_reset = false;
	for (Chain &chain : chains) {
		for (Joint &joint : chain.joints) {
			joint.location = joint.current_pose_location;
			joint.prev_location = joint.current_pose_location;
			joint.pose_location = joint.current_pose_location;
			joint.prev_pose_location = joint.current_pose_location;
			joint.display_location = joint.current_pose_location;
		}
	}

	step_time = 0.0;
	prev_skeleton_transform = p_skeleton_transform;
	prev_step_move = Vector3();
	prev_step_turn = Quaternion();
	smoothing_velocity = Vector3();
	prev_frame_center = frame_center;
	prev_frame_center_rotation = frame_center_rotation;
	step_center = frame_center;
	step_center_rotation = frame_center_rotation;
}

// mirror 쪽은 목록 뒤에 하나 더 넣는다.
void BoneClothSimulator3D::_gather_collisions(LocalVector<PosedCollision> &r_collisions) const {
	for (int i = 0; i < get_child_count(); i++) {
		BoneClothCollision3D *collision = Object::cast_to<BoneClothCollision3D>(get_child(i));
		if (collision) {
			collision->sync_pose();
			r_collisions.push_back({ collision, false });
		}
	}

	const uint32_t collision_count = r_collisions.size();
	for (uint32_t i = 0; i < collision_count; i++) {
		if (r_collisions[i].collision->has_mirror()) {
			r_collisions.push_back({ r_collisions[i].collision, true });
		}
	}
}

BoneClothSimulator3D::FrameMotion BoneClothSimulator3D::_measure_motion(double p_delta, const Transform3D &p_skeleton_transform) {
	FrameMotion motion;
	motion.skeleton_transform = p_skeleton_transform;

	const Transform3D move = p_skeleton_transform.affine_inverse() * prev_skeleton_transform;
	motion.move_location = move.origin;
	motion.move_rotation = move.basis.get_rotation_quaternion();

	// 순간이동이면 지난 step의 움직임을 다시 써서 천이 튀지 않게 한다.
	motion.teleported = motion.move_location.length() > teleport_distance * units_per_meter || Quaternion().angle_to(motion.move_rotation) > teleport_angle;

	motion.elapsed = step_time + p_delta;
	step_time = MIN(motion.elapsed, MAX_STEPS * STEP);
	motion.dropped = motion.elapsed - step_time;
	motion.step_count = int(step_time / STEP);
	step_time -= motion.step_count * STEP;

	// inertia가 1이면 이동을 모두 받아 휘날리고, 0이면 skeleton을 그대로 따른다.
	const float share = float(STEP / motion.elapsed);
	const float speed_limit = movement_speed_limit * units_per_meter;
	motion.step_move = (motion.move_location * share * inertia).limit_length(speed_limit * float(STEP));
	motion.step_turn = Quaternion().slerp(motion.move_rotation, share * inertia);
	const float max_turn = rotation_speed_limit * float(STEP);
	if (motion.step_turn.get_angle() > max_turn) {
		motion.step_turn = Quaternion(motion.step_turn.get_axis(), max_turn);
	}

	if (motion.teleported) {
		motion.step_move = prev_step_move;
		motion.step_turn = prev_step_turn;
	}

	// 급출발과 급정지에서 천이 튀지 않게 step마다 조금씩 바꾼다.
	motion.smoothing = inertia_smoothing >= 1e-6f && !motion.teleported;
	motion.smoothing_rate = CLAMP(Math::pow(1.0f - inertia_smoothing, 3.0f) * 0.99f + 0.01f, 0.0f, 1.0f);
	motion.frame_velocity = p_skeleton_transform.basis.xform((-motion.move_location / float(motion.elapsed)).limit_length(speed_limit));
	motion.to_skeleton = p_skeleton_transform.basis.inverse();
	// 크기는 joint.gravity에 이미 들어 있어서 방향만 바꾼다.
	motion.gravity_direction = motion.to_skeleton.xform(gravity_direction).normalized();
	return motion;
}

void BoneClothSimulator3D::_run_steps(FrameMotion &r_motion, const LocalVector<PosedCollision> &p_collisions) {
	const float speed_limit = movement_speed_limit * units_per_meter;
	for (int step = 0; step < r_motion.step_count; step++) {
		if (r_motion.smoothing) {
			smoothing_velocity = smoothing_velocity.lerp(r_motion.frame_velocity, r_motion.smoothing_rate);
			r_motion.step_move = (r_motion.to_skeleton.xform(-smoothing_velocity) * float(STEP) * inertia).limit_length(speed_limit * float(STEP));
		}

		const float weight = float(step + 1) / float(r_motion.step_count);
		for (Chain &chain : chains) {
			for (Joint &joint : chain.joints) {
				joint.pose_location = joint.prev_pose_location.lerp(joint.current_pose_location, weight);
			}
		}

		// 중심도 pose처럼 보간한다.
		const Vector3 center = prev_frame_center.lerp(frame_center, weight);
		const Quaternion center_rotation = prev_frame_center_rotation.slerp(frame_center_rotation, weight).normalized();
		center_move = center - step_center;
		center_turn = (center_rotation * step_center_rotation.inverse()).normalized();
		_update_local_shares(center, center_rotation, r_motion.step_move, r_motion.step_turn);

		_step(r_motion.gravity_direction, r_motion.step_move, r_motion.step_turn, p_collisions);

		step_center = center;
		step_center_rotation = center_rotation;

		prev_step_move = r_motion.step_move;
		prev_step_turn = r_motion.step_turn;
	}
}

// 화면에 그릴 위치를 예측 위치 쪽으로 옮기고 collision과 길이를 다시 맞춘다.
void BoneClothSimulator3D::_settle_display(double p_delta, const FrameMotion &p_motion, const LocalVector<PosedCollision> &p_collisions) {
	const double spent = p_motion.teleported ? 1.0 : (p_motion.step_count * STEP + p_motion.dropped) / p_motion.elapsed;
	prev_skeleton_transform = prev_skeleton_transform.interpolate_with(p_motion.skeleton_transform, float(spent));

	const float follow = float(p_delta / (STEP - step_time + p_delta));
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty()) {
			_update_display(chain, follow);
		}
	}

	_collide(p_collisions, &Joint::display_location);
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty()) {
			_restore_lengths(chain, &Joint::display_location, &Joint::current_pose_location);
		}
	}
}

void BoneClothSimulator3D::_end_frame(Skeleton3D *p_skeleton) {
	prev_frame_center = frame_center;
	prev_frame_center_rotation = frame_center_rotation;

	for (Chain &chain : chains) {
		for (Joint &joint : chain.joints) {
			joint.prev_pose_location = joint.current_pose_location;
		}

		_write_rotations(p_skeleton, chain);
	}

	// joint가 매 frame 움직이므로 gizmo도 매 frame 다시 그린다.
	update_gizmos();
}

// 탭에는 Bone Cloth 아래에 Time과 Joints로 나온다(에디터가 이름의 _를 띄우고, 단어의 첫 글자를 대문자로 바꾼다).
void BoneClothSimulator3D::add_monitors() {
	Performance *performance = Performance::get_singleton();
	performance->add_custom_monitor("bone_cloth/time", callable_mp_static(&get_monitor_time), Array(), Performance::MONITOR_TYPE_TIME);
	performance->add_custom_monitor("bone_cloth/joints", callable_mp_static(&get_monitor_joints));
}

void BoneClothSimulator3D::remove_monitors() {
	Performance *performance = Performance::get_singleton();
	performance->remove_custom_monitor("bone_cloth/time");
	performance->remove_custom_monitor("bone_cloth/joints");
}

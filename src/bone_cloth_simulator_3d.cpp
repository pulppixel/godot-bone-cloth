#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

// frame rate가 달라도 천이 자연스럽게 움직이도록, 1/60초짜리 step으로 계산한다.
// step은 한 frame에 4번까지 돈다. 느린 frame에서 연산이 계속 돌지 않게 하려는 것이다 (초과한 시간은 버린다).
// drag, stiffness, link_stiffness는 step 1번 기준 값이다.
static constexpr double STEP = 1.0 / 60.0;
static constexpr int MAX_STEPS = 4;

// link의 부드러움 정도 (사실상 거의 늘어나지 않는다. 세기는 link_stiffness로 정할 수 있다).
static constexpr float LINK_COMPLIANCE = 1e-9f;

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

		// parent chain은 앞 chain에서만 찾는다. 파생 chain은 이번 step에 먼저 움직인 parent joint를 따라가야 한다.
		// 순서가 반대면 configuration warning이 알려 준다.
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
void BoneClothSimulator3D::_build_links(Skeleton3D *p_skeleton) {
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

	auto rest_location = [&](const Chain &p_chain, uint32_t p_depth) {
		const Joint &joint = p_chain.joints[p_depth];
		if (joint.bone >= 0) {
			return p_skeleton->get_bone_global_rest(joint.bone).origin;
		}

		const Transform3D end = p_skeleton->get_bone_global_rest(p_chain.joints[p_depth - 1].bone);
		return end.origin + end.basis.orthonormalized().xform(p_chain.end_axis) * (p_chain.end_bone_length * units_per_meter);
	};

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
			link.length = (rest_location(chain_a, depth) - rest_location(chain_b, depth)).length();
			links.push_back(link);
		}
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

// 이번 frame에 애니메이션(과 앞선 modifier)이 만든 bone 위치와 회전을 읽는다. tip은 end bone에서 계산한다.
void BoneClothSimulator3D::_read_pose(Skeleton3D *p_skeleton, Chain &p_chain) {
	for (Joint &joint : p_chain.joints) {
		if (joint.bone >= 0) {
			const Transform3D pose = p_skeleton->get_bone_global_pose(joint.bone);
			joint.current_pose_location = pose.origin;
			joint.pose_basis = pose.basis;
		}
	}

	Joint &tip = p_chain.joints[p_chain.joints.size() - 1];
	if (tip.bone < 0) {
		const Joint &end = p_chain.joints[p_chain.joints.size() - 2];
		tip.current_pose_location = end.current_pose_location + end.pose_basis.orthonormalized().xform(p_chain.end_axis) * (p_chain.end_bone_length * units_per_meter);
	}
}

// curve에서 p_rate 위치의 값을 읽는다. curve가 없으면 1.0f를 반환한다.
// inspector에서 갓 만든 curve는 point가 없어 0으로 읽힌다. 그대로 곱하면 inspector 값과 상관없이 0이 되므로, 이때도 1.0f를 반환한다.
static float sample_curve(const Ref<Curve> &p_curve, float p_rate) {
	return p_curve.is_valid() && p_curve->get_point_count() > 0 ? p_curve->sample_baked(p_rate) : 1.0f;
}

// chain 위치마다 값을 다르게 주기 위해, joint의 length_rate 위치에서 읽은 curve 값을 곱한다.
// 실행 중에 값이나 curve를 바꾸면 바로 반영되도록 매 frame 다시 계산한다.
// radius와 gravity는 m 단위 값이라 units_per_meter를 곱해 skeleton 단위로 바꾼다.
void BoneClothSimulator3D::_update_joint_settings(Chain &p_chain) {
	for (Joint &joint : p_chain.joints) {
		joint.drag = CLAMP(drag * sample_curve(drag_damping_curve, joint.length_rate), 0.0f, 1.0f);
		joint.stiffness = CLAMP(stiffness * sample_curve(stiffness_damping_curve, joint.length_rate), 0.0f, 1.0f);
		joint.radius = MAX(radius * sample_curve(radius_damping_curve, joint.length_rate), 0.0f) * units_per_meter;
		joint.limit_angle = MAX(limit_angle * sample_curve(limit_angle_damping_curve, joint.length_rate), 0.0f);
		joint.gravity = gravity * sample_curve(gravity_damping_curve, joint.length_rate) * units_per_meter;
	}
}

// 모든 chain root의 중심과 회전을 읽는다. 파생 chain의 root는 parent joint를 따라가는 점이라 넣지 않는다.
// 회전은 root마다 rest에서 회전한 양(pose 회전 * rest 회전의 역)의 평균이다.
// root bone들은 서로 다른 방향을 향하지만 rest에서 돈 양은 골반이 회전한 양과 거의 같아서, 평균해도 서로 지워지지 않는다.
// 쿼터니언은 q와 -q가 같은 회전이라, 첫 root의 값과 반대쪽을 향하는 값은 뒤집어서 더한다.
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

// step 1번의 처리 순서.
void BoneClothSimulator3D::_step(const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn, const LocalVector<PosedCollision> &p_collisions) {
	// root는 몸에 붙은 joint라 물리 계산 없이 pose를 따라간다. 파생된 chain의 root는 _follow_parent_chain에서 옮긴다.
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty() && chain.parent_chain < 0) {
			Joint &root = chain.joints[0];
			root.prev_location = root.location;
			root.location = root.pose_location;
		}
	}

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_simulate(chain, p_gravity_direction, p_move, p_turn);
	}

	_solve_links();
	for (Chain &chain : chains) {
		_collide(chain, p_collisions);
	}

	_collide_links(p_collisions);
	_solve_links();

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_restore_limits_and_lengths(chain);
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

// root 아래 joint를 움직인다.
// 이전 움직임을 이어 가고(drag로 줄이고 gravity를 더한다), 캐릭터 움직임을 반영하고, stiffness만큼 pose 쪽으로 당긴다.
void BoneClothSimulator3D::_simulate(Chain &p_chain, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn) {
	const float dt = float(STEP);

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];

		// 중심(root들)이 이번 step에 움직이고 회전항 양 중 share만큼을 joint에 그대로 옮긴다.
		// share는 root 쪽에서 depth_inertia이고, 끝으로 갈수록 줄어 끝에서 0이다. 허리는 골반을 따라가고 끝단만 흔들린다.
		const float share = depth_inertia * (1.0f - joint.length_rate * joint.length_rate);
		if (share > 0.0f) {
			const Vector3 move = center_move * share;
			const Quaternion turn = Quaternion().slerp(center_turn, share);
			joint.location = step_center + turn.xform(joint.location - step_center) + move;
			joint.prev_location = step_center + turn.xform(joint.prev_location - step_center) + move;
		}

		Vector3 velocity = (joint.location - joint.prev_location) / dt;

		// 캐릭터가 움직이는 속도는 그대로 두고, 몸을 기준으로 천이 흔들리는 속도만 joint_speed_limit으로 제한한다.
		if (joint_speed_limit > 0.0f) {
			const Vector3 carried = (prev_step_move + prev_step_turn.xform(joint.prev_location) - joint.prev_location) / dt;
			velocity = (velocity - carried).limit_length(joint_speed_limit * units_per_meter) + carried;
		}

		joint.prev_location = joint.location;
		velocity *= 1.0f - joint.drag;
		velocity += p_gravity_direction * joint.gravity * dt;
		joint.location += velocity * dt;

		// 캐릭터가 등속도로 달리는 동안에는 천이 뒤로 밀리지 않게 한다.
		// 출발, 정지, 방향 전환처럼 skeleton 움직임이 지난 step과 달라진 만큼만 joint 위치에 더한다.
		// 이 중 share만큼은 중심과 함께 옮긴 것으로 보고 뺀다(root 쪽은 출발, 정지 때도 덜 흔들린다).
		joint.location += (p_move - prev_step_move) * (1.0f - share);
		joint.location += (p_turn.xform(joint.prev_location) - prev_step_turn.xform(joint.prev_location)) * (1.0f - share);

		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * joint.stiffness;
	}
}

// XPBD: 위치를 직접 업데이트하여 거리를 맞추는 계산법
// link와 rest의 거리의 차이를 구하고, 그 차이의 절반씩 두 joint를 서로 밀고 당긴다.
void BoneClothSimulator3D::_solve_links() {
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

// joint마다, 이 chain의 collides가 true인 collision으로만 밀어낸다 (root 제외).
// 그다음 bone(위 joint와 아래 joint를 잇는 선분)마다 같은 collision으로 밀어낸다.
// joint 사이가 멀면 joint만 밀어서는 그 사이로 몸이 빠져나온다(예: 땋은 머리 bone 가운데가 가슴에 묻힌다).
// 위 joint가 root면 아래 joint만 움직인다(root는 pose나 parent chain을 따라간다).
void BoneClothSimulator3D::_collide(Chain &p_chain, const LocalVector<PosedCollision> &p_collisions) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		for (uint32_t c = 0; c < p_collisions.size(); c++) {
			if (p_chain.collides[c]) {
				joint.location = p_collisions[c].collision->collide(joint.location, joint.radius, p_collisions[c].mirror);
			}
		}
	}

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &parent = p_chain.joints[i - 1];
		Joint &joint = p_chain.joints[i];
		const float bone_radius = (parent.radius + joint.radius) * 0.5f;
		for (uint32_t c = 0; c < p_collisions.size(); c++) {
			if (p_chain.collides[c]) {
				Vector3 parent_location = parent.location;
				p_collisions[c].collision->collide_segment(parent_location, joint.location, bone_radius, p_collisions[c].mirror);
				if (i > 1) {
					parent.location = parent_location;
				}
			}
		}
	}
}

// 치마의 joint 두 개가 벌어져 있을 때, 두 joint가 닿지 않는 사이의 천에서는 다리가 뚫고 나올 수 있다.
// 그래서 두 joint를 잇는 선을 다리 밖으로 밀어 준다.
// 두 chain의 collides가 모두 true인 collision만 쓴다(한쪽 chain에서 뺀 collision이 link를 거쳐 그 chain을 밀지 않게).
void BoneClothSimulator3D::_collide_links(const LocalVector<PosedCollision> &p_collisions) {
	for (const Link &link : links) {
		Chain &chain_a = chains[link.chain_a];
		Chain &chain_b = chains[link.chain_b];
		Joint &joint_a = chain_a.joints[link.depth];
		Joint &joint_b = chain_b.joints[link.depth];
		const float link_radius = (joint_a.radius + joint_b.radius) * 0.5f;
		for (uint32_t c = 0; c < p_collisions.size(); c++) {
			if (chain_a.collides[c] && chain_b.collides[c]) {
				p_collisions[c].collision->collide_segment(joint_a.location, joint_b.location, link_radius, p_collisions[c].mirror);
			}
		}
	}
}

// joint가 pose 방향에서 p_limit 각도보다 많이 기울면, 딱 p_limit까지만 기울도록 한다.
static Vector3 limit_to_cone(const Vector3 &p_offset, const Vector3 &p_pose_vector, float p_limit) {
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
	return p_offset.rotated(turn_axis, p_limit - angle);
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
			offset = limit_to_cone(offset, pose_vector, joint.limit_angle);
		}

		joint.location = parent.location + offset.normalized() * pose_vector.length();
	}
}

// step이 frame보다 드물 때(슬로 모션, 60Hz보다 빠른 모니터) joint가 멈췄다 튀지 않고 매 frame 움직이게 한다.
// 지난 step의 이동으로 다음 step 위치를 예측하고, 이번 frame 시간 비율만큼 그쪽으로 옮긴다.
// 예측은 실제와 다를 수 있으므로, 화면 위치에도 root부터 각도 제한과 bone 길이를 다시 맞춘다.
void BoneClothSimulator3D::_update_display(Chain &p_chain, float p_follow) {
	Joint &root = p_chain.joints[0];
	root.display_location = p_chain.parent_chain >= 0 ? chains[p_chain.parent_chain].joints[p_chain.parent_joint].display_location : root.current_pose_location;

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 next_location = joint.location + (joint.location - joint.prev_location);
		joint.display_location = joint.display_location.lerp(next_location, p_follow);

		const Vector3 pose_vector = joint.current_pose_location - parent.current_pose_location;
		Vector3 offset = joint.display_location - parent.display_location;
		if (limit_angle > 0.0f) {
			offset = limit_to_cone(offset, pose_vector, joint.limit_angle);
		}

		joint.display_location = parent.display_location + offset.normalized() * pose_vector.length();
	}
}

// frame의 마지막 단계. joint 위치를 bone 회전으로 바꿔 skeleton에 넣는다(mesh는 bone 회전으로 움직인다).
// bone마다 위 bone이 돈 만큼 먼저 돌고, 남은 각만큼 더 돌아서 아래 joint를 향한다.
// pose 방향에서 바로 돌리면, 머리카락이 머리 위로 넘어갈 때(180도 근처) bone이 frame마다 비틀린다.
// root부터 차례로 넣어야 한다(set_bone_global_pose가 부모 bone의 pose를 쓴다).
void BoneClothSimulator3D::_write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain) {
	Quaternion turn; // 위 bone들이 pose보다 더 회전한 양. 처음은 회전 없음.
	for (uint32_t i = p_chain.parent_chain >= 0 ? 2 : 1; i < p_chain.joints.size(); i++) {
		const Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.current_pose_location - parent.current_pose_location;
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
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothSimulator3D::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothSimulator3D::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_limit_angle_damping_curve", "curve"), &BoneClothSimulator3D::set_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("get_limit_angle_damping_curve"), &BoneClothSimulator3D::get_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity_damping_curve", "curve"), &BoneClothSimulator3D::set_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("get_gravity_damping_curve"), &BoneClothSimulator3D::get_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity_direction", "direction"), &BoneClothSimulator3D::set_gravity_direction);
	ClassDB::bind_method(D_METHOD("get_gravity_direction"), &BoneClothSimulator3D::get_gravity_direction);

	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothSimulator3D::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothSimulator3D::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia_smoothing", "smoothing"), &BoneClothSimulator3D::set_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("get_inertia_smoothing"), &BoneClothSimulator3D::get_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("set_depth_inertia", "depth_inertia"), &BoneClothSimulator3D::set_depth_inertia);
	ClassDB::bind_method(D_METHOD("get_depth_inertia"), &BoneClothSimulator3D::get_depth_inertia);
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

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), "set_link_mode", "get_link_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");

	// chain 위치마다 값을 다르게 주기 위해 값마다 curve가 있다(<name>/value, <name>/damping_curve).
	// stiffness는 step마다 joint를 pose 쪽으로 당기는 비율이다(0에서 1).
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_damping_curve", "get_drag_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_stiffness_damping_curve", "get_stiffness_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_radius_damping_curve", "get_radius_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_limit_angle_damping_curve", "get_limit_angle_damping_curve");

	// gravity/value는 가속도(m/s^2)다. step마다 속도에 value * dt를 더한다.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_gravity_damping_curve", "get_gravity_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), "set_gravity_direction", "get_gravity_direction");

	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia_smoothing", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia_smoothing", "get_inertia_smoothing");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_depth_inertia", "get_depth_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_speed_limit", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m/s"), "set_joint_speed_limit", "get_joint_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");
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
	const String bone_names = skeleton ? skeleton->get_concatenated_bone_names() : String();

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
	if (link_mode == LINK_MODE_NONE && p_property.name == StringName("link_stiffness")) {
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

	// 씬 안의 scale이 바뀌면 길이 값을 다시 바꿔야 하므로 chain을 다시 만든다.
	// owner가 없으면 캐릭터가 돌 때 float 오차가 생기므로, 근사값으로 비교한다.
	const float units = _get_units_per_meter(skeleton);
	if (!Math::is_equal_approx(units, units_per_meter)) {
		units_per_meter = units;
		joints_dirty = true;
	}

	// 설정이 바뀌면 chain을 다시 만든다.
	if (joints_dirty) {
		joints_dirty = false;
		needs_reset = true;
		for (Chain &chain : chains) {
			if (!_build_joints(skeleton, chain)) {
				chain.joints.clear();
			}
		}
		_build_branches();
		_build_links(skeleton);
	}

	for (Chain &chain : chains) {
		if (!chain.joints.is_empty()) {
			_read_pose(skeleton, chain);
			_update_joint_settings(chain);
		}
	}

	_read_center(skeleton);

	// 이번 frame에 화면에 그려지는 skeleton transform (physics tick 사이 frame에서도 보간된 값이다).
	const Transform3D skeleton_transform = skeleton->get_global_transform_interpolated();

	if (needs_reset) {
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
		prev_skeleton_transform = skeleton_transform;
		prev_step_move = Vector3();
		prev_step_turn = Quaternion();
		smoothing_velocity = Vector3();
		prev_frame_center = frame_center;
		prev_frame_center_rotation = frame_center_rotation;
		step_center = frame_center;
		step_center_rotation = frame_center_rotation;
	}

	LocalVector<PosedCollision> collisions;
	for (int i = 0; i < get_child_count(); i++) {
		BoneClothCollision3D *collision = Object::cast_to<BoneClothCollision3D>(get_child(i));
		if (collision) {
			collision->sync_pose();
			collisions.push_back({ collision, false });
		}
	}

	const uint32_t collision_count = collisions.size();
	for (uint32_t i = 0; i < collision_count; i++) {
		if (collisions[i].collision->has_mirror()) {
			collisions.push_back({ collisions[i].collision, true });
		}
	}

	_find_chain_collisions(collisions);

	if (p_delta > 0.0) {
		// 지난 step 이후 skeleton이 움직이고 돈 양(현재 skeleton 기준).
		const Transform3D move = skeleton_transform.affine_inverse() * prev_skeleton_transform;
		const Vector3 move_location = move.origin;
		const Quaternion move_rotation = move.basis.get_rotation_quaternion();

		// 순간이동 때 천이 크게 튀지 않게 한다.
		// 한 frame에 teleport_distance보다 멀리 가거나 teleport_angle보다 많이 돌면,
		// skeleton이 지난 step과 같은 양만큼 움직인 것으로 계산한다.
		// 돈 각도는 angle_to로 잰다.
		const bool teleported = move_location.length() > teleport_distance * units_per_meter || Quaternion().angle_to(move_rotation) > teleport_angle;

		const double elapsed = step_time + p_delta;
		step_time = MIN(elapsed, MAX_STEPS * STEP);
		const double dropped = elapsed - step_time;
		const int step_count = int(step_time / STEP);
		step_time -= step_count * STEP;

		// 캐릭터가 움직일 때 천을 얼마나 뒤로 휘날릴지 정한다.
		// inertia가 1이면 이동을 모두 반영하여 뒤로 휘날리고, 0이면 skeleton과 동일하게 움직인다.
		const float share = float(STEP / elapsed);
		const float speed_limit = movement_speed_limit * units_per_meter;
		Vector3 step_move = (move_location * share * inertia).limit_length(speed_limit * float(STEP));
		Quaternion step_turn = Quaternion().slerp(move_rotation, share * inertia);
		const float max_turn = rotation_speed_limit * float(STEP);
		if (step_turn.get_angle() > max_turn) {
			step_turn = Quaternion(step_turn.get_axis(), max_turn);
		}

		if (teleported) {
			step_move = prev_step_move;
			step_turn = prev_step_turn;
		}

		// 급출발, 급정지 때 천이 갑자기 튀지 않게 한다
		// frame rate와 상관없이 같은 시간이 걸리도록 step마다 바꾼다.
		const bool smoothing = inertia_smoothing >= 1e-6f && !teleported;
		const float smoothing_rate = CLAMP(Math::pow(1.0f - inertia_smoothing, 3.0f) * 0.99f + 0.01f, 0.0f, 1.0f);
		const Vector3 frame_velocity = skeleton_transform.basis.xform((-move_location / float(elapsed)).limit_length(speed_limit));
		const Basis to_skeleton = skeleton_transform.basis.inverse();
		// 방향만 skeleton 공간으로 바꾼다. 크기는 joint.gravity에 이미 skeleton 단위로 들어 있다.
		const Vector3 skeleton_gravity_direction = to_skeleton.xform(gravity_direction).normalized();

		for (int step = 0; step < step_count; step++) {
			if (smoothing) {
				smoothing_velocity = smoothing_velocity.lerp(frame_velocity, smoothing_rate);
				step_move = (to_skeleton.xform(-smoothing_velocity) * float(STEP) * inertia).limit_length(speed_limit * float(STEP));
			}

			const float weight = float(step + 1) / float(step_count);
			for (Chain &chain : chains) {
				for (Joint &joint : chain.joints) {
					joint.pose_location = joint.prev_pose_location.lerp(joint.current_pose_location, weight);
				}
			}

			// 중심도 pose처럼 보간하고, 지난 step부터 움직이고 회전한 양을 구한다.
			const Vector3 center = prev_frame_center.lerp(frame_center, weight);
			const Quaternion center_rotation = prev_frame_center_rotation.slerp(frame_center_rotation, weight).normalized();
			center_move = center - step_center;
			center_turn = (center_rotation * step_center_rotation.inverse()).normalized();

			_step(skeleton_gravity_direction, step_move, step_turn, collisions);

			step_center = center;
			step_center_rotation = center_rotation;

			prev_step_move = step_move;
			prev_step_turn = step_turn;
		}

		// 남은 이동은 다음 step으로 넘긴다.
		const double spent = teleported ? 1.0 : (step_count * STEP + dropped) / elapsed;
		prev_skeleton_transform = prev_skeleton_transform.interpolate_with(skeleton_transform, float(spent));

		// 화면 위치를 예측 위치 쪽으로 얼마나 옮길지 계산.
		// 지난 frame부터 다음 step까지 걸리는 시간 중 이번 frame이 쓴 비율이다.
		const float follow = float(p_delta / (STEP - step_time + p_delta));
		for (Chain &chain : chains) {
			if (!chain.joints.is_empty()) {
				_update_display(chain, follow);
			}
		}
	}

	prev_frame_center = frame_center;
	prev_frame_center_rotation = frame_center_rotation;

	for (Chain &chain : chains) {
		for (Joint &joint : chain.joints) {
			joint.prev_pose_location = joint.current_pose_location;
		}

		_write_rotations(skeleton, chain);
	}

	// joint가 매 frame 움직이므로 gizmo도 매 frame 다시 그린다.
	update_gizmos();
}

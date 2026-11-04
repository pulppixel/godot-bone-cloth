#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

#include <atomic>
#include <chrono>

using namespace godot;

static constexpr double STEP = 1.0 / 60.0;
static constexpr int MAX_STEPS = 4;

static constexpr float LINK_COMPLIANCE = 1e-9f;

static constexpr float LINK_SPACING_RAMP = 0.3f;

struct MonitorSum {
	std::atomic<uint64_t> sum = 0;
	uint64_t read_frame = 0;
	double average = 0.0;

	double read() {
		const uint64_t frame = Engine::get_singleton()->get_process_frames();
		if (frame != read_frame) {
			average = double(sum.exchange(0)) / double(frame - read_frame);
			read_frame = frame;
		}
		return average;
	}
};

static MonitorSum monitor_time;
static MonitorSum monitor_joints;

static double get_monitor_time() {
	return monitor_time.read() / 1e9;
}

static int64_t get_monitor_joints() {
	return int64_t(Math::round(monitor_joints.read()));
}

static Vector3 end_bone_axis(Skeleton3D *p_skeleton, int p_end_bone, SkeletonModifier3D::BoneDirection p_direction) {
	if (p_direction != SkeletonModifier3D::BONE_DIRECTION_FROM_PARENT) {
		static const Vector3 axes[] = { Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0), Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(0, 0, -1) };
		return axes[p_direction];
	}

	const Transform3D rest = p_skeleton->get_bone_rest(p_end_bone);
	return rest.basis.xform_inv(rest.origin).normalized();
}

bool BoneClothSimulator3D::_build_joints(Skeleton3D *p_skeleton, Chain &p_chain) {
	p_chain.joints.clear();
	const int root = p_chain.root_bone;
	const int end = p_chain.end_bone;

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

bool BoneClothSimulator3D::_is_child_collision(const NodePath &p_node_path) const {
	if (!is_inside_tree() || p_node_path.is_empty()) {
		return true;
	}

	const Node *node = get_node_or_null(p_node_path);
	return node && node->get_parent() == this && Object::cast_to<BoneClothCollision3D>(node);
}

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

// inspector에서 막 만든 Curve는 점이 없어 0을 돌려주므로, 이때도 1을 쓴다.
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

void BoneClothSimulator3D::_step(const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn, const LocalVector<PosedCollision> &p_collisions) {
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

void BoneClothSimulator3D::_restore_lengths(Chain &p_chain, Vector3 Joint::*p_field, Vector3 Joint::*p_pose_field) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.*p_pose_field - parent.*p_pose_field;
		const Vector3 offset = joint.*p_field - parent.*p_field;
		joint.*p_field = parent.*p_field + (offset.is_zero_approx() ? pose_vector : offset.normalized() * pose_vector.length());
	}
}

void BoneClothSimulator3D::_follow_parent_chain(Chain &p_chain) {
	if (p_chain.parent_chain < 0 || p_chain.joints.is_empty()) {
		return;
	}

	Joint &root = p_chain.joints[0];
	root.prev_location = root.location;
	root.location = chains[p_chain.parent_chain].joints[p_chain.parent_joint].location;
}

void BoneClothSimulator3D::_move_joint(Joint &r_joint, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn) {
	const float dt = float(STEP);

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

	if (joint_speed_limit > 0.0f) {
		const Vector3 carried = (prev_step_move + prev_step_turn.xform(r_joint.prev_location) - r_joint.prev_location) / dt * (1.0f - share);
		velocity = (velocity - carried).limit_length(joint_speed_limit * units_per_meter) + carried;
	}

	r_joint.prev_location = r_joint.location;
	velocity *= 1.0f - r_joint.drag;
	velocity += p_gravity_direction * r_joint.gravity * dt;
	r_joint.location += velocity * dt;

	r_joint.location += (p_move - prev_step_move) * (1.0f - share);
	r_joint.location += (p_turn.xform(r_joint.prev_location) - prev_step_turn.xform(r_joint.prev_location)) * (1.0f - turn_share);
}

void BoneClothSimulator3D::_move_root(Chain &r_chain, const Vector3 &p_move, const Quaternion &p_turn) {
	Joint &root = r_chain.joints[0];
	_move_joint(root, Vector3(), p_move, p_turn);

	Vector3 offset = ((root.location - root.pose_location) * (1.0f - root_spring_stiffness)).limit_length(root_spring_limit * units_per_meter);

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

void BoneClothSimulator3D::_simulate(Chain &p_chain, const Vector3 &p_gravity_direction, const Vector3 &p_move, const Quaternion &p_turn) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		_move_joint(joint, p_gravity_direction, p_move, p_turn);

		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * joint.stiffness;
	}
}

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

static Vector3 slide_on_contact(const LocalVector<Vector3> &p_pushes, const Vector3 &p_move) {
	const Vector3 push = combine_pushes(p_pushes);
	if (push.is_zero_approx()) {
		return p_move;
	}

	const Vector3 normal = push.normalized();
	return p_move - normal * MIN(p_move.dot(normal), 0.0f);
}

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

		const float ramp = MIN((1.0f - distance / spacing) / LINK_SPACING_RAMP, 1.0f);
		const Vector3 correction = delta / distance * ((distance - spacing) * ramp * strength * 0.5f);
		joint_a.location += slide_on_contact(joint_a.collision_pushes, correction);
		joint_b.location += slide_on_contact(joint_b.collision_pushes, -correction);
	}
}

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

		if (p_split && p_collisions[c].collision->separates(location_a, location_b, p_collisions[c].mirror)) {
			continue;
		}

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

static Vector3 limit_to_cone(const Vector3 &p_offset, const Vector3 &p_pose_vector, float p_limit, float p_stiffness) {
	const Vector3 pose_direction = p_pose_vector.normalized();
	const Vector3 direction = p_offset.normalized();
	const Vector3 axis = pose_direction.cross(direction);
	const float angle = Math::atan2(axis.length(), pose_direction.dot(direction));
	if (angle <= p_limit) {
		return p_offset;
	}

	const Vector3 turn_axis = axis.is_zero_approx() ? pose_direction.get_any_perpendicular() : axis.normalized();
	return p_offset.rotated(turn_axis, (p_limit - angle) * p_stiffness);
}

void BoneClothSimulator3D::_restore_limits_and_lengths(Chain &p_chain) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.pose_location - parent.pose_location;

		Vector3 offset = joint.location - parent.location;
		if (limit_angle > 0.0f) {
			offset = limit_to_cone(offset, pose_vector, joint.limit_angle, limit_angle_stiffness);
		}

		joint.location = parent.location + offset.normalized() * pose_vector.length();
	}
}

void BoneClothSimulator3D::_update_display(Chain &p_chain, float p_follow) {
	Joint &root = p_chain.joints[0];
	root.display_location = p_chain.parent_chain >= 0 ? chains[p_chain.parent_chain].joints[p_chain.parent_joint].display_location : root.current_pose_location + (root.location - root.pose_location);

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Vector3 next_location = joint.location + (joint.location - joint.prev_location);
		joint.display_location = joint.display_location.lerp(next_location, p_follow);
	}
}

void BoneClothSimulator3D::_write_rotations(Skeleton3D *p_skeleton, const Chain &p_chain) {
	Quaternion turn;
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

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_gravity_damping_curve", "get_gravity_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), "set_gravity_direction", "get_gravity_direction");

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

	ADD_GROUP("Root Spring", "root_spring_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_root_spring_stiffness", "get_root_spring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_limit", "get_root_spring_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_forward_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_forward_limit", "get_root_spring_forward_limit");
	ADD_GROUP("", "");

	// godot-cpp에는 ADD_ARRAY_COUNT가 없어서 PROPERTY_USAGE_ARRAY로 대신한다.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), "set_chain_count", "get_chain_count");

	BIND_ENUM_CONSTANT(LINK_MODE_NONE);
	BIND_ENUM_CONSTANT(LINK_MODE_SEQUENTIAL);
	BIND_ENUM_CONSTANT(LINK_MODE_LOOP);
}

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

void BoneClothSimulator3D::_validate_property(PropertyInfo &p_property) const {
	if (link_mode == LINK_MODE_NONE && (p_property.name == StringName("link_stiffness") || p_property.name == StringName("link_split") || String(p_property.name).begins_with("link_spacing/"))) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void BoneClothSimulator3D::apply_preset(const Ref<BoneClothPreset> &p_preset) {
	ERR_FAIL_COND_MSG(p_preset.is_null(), "No preset to apply.");
	for (const char *name : BoneClothPreset::VALUES) {
		set(name, BoneClothPreset::copy_value(p_preset->get(name)));
	}
}

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

void BoneClothSimulator3D::set_root_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].root_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
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

float BoneClothSimulator3D::get_units_per_meter() const {
	return units_per_meter;
}

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

PackedStringArray BoneClothSimulator3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	if (chains.is_empty()) {
		warnings.push_back("No chains. Add a chain under Chains and pick its root bone and end bone.");
	}

	const Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const Vector3 scale = skeleton->get_global_transform().basis.get_scale();
		if (!(Math::is_zero_approx(scale.x - scale.y) && Math::is_zero_approx(scale.y - scale.z))) {
			warnings.push_back(vformat("The Skeleton3D has a non-uniform scale %s, so the cloth moves differently. Make the scale of the Skeleton3D and its parents uniform (the same on all axes).", scale));
		}
	}

	for (uint32_t c = 0; c < chains.size(); c++) {
		const Chain &chain = chains[c];
		PackedStringArray problems;
		if (skeleton) {
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

// 모델을 키우면 천도 같이 커져야 하므로, 모델 안쪽의 scale만 단위로 쓴다.
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

	motion.teleported = motion.move_location.length() > teleport_distance * units_per_meter || Quaternion().angle_to(motion.move_rotation) > teleport_angle;

	motion.elapsed = step_time + p_delta;
	step_time = MIN(motion.elapsed, MAX_STEPS * STEP);
	motion.dropped = motion.elapsed - step_time;
	motion.step_count = int(step_time / STEP);
	step_time -= motion.step_count * STEP;

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

	motion.smoothing = inertia_smoothing >= 1e-6f && !motion.teleported;
	motion.smoothing_rate = CLAMP(Math::pow(1.0f - inertia_smoothing, 3.0f) * 0.99f + 0.01f, 0.0f, 1.0f);
	motion.frame_velocity = p_skeleton_transform.basis.xform((-motion.move_location / float(motion.elapsed)).limit_length(speed_limit));
	motion.to_skeleton = p_skeleton_transform.basis.inverse();
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

	update_gizmos();
}

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

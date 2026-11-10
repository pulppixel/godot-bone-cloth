#include "bone_cloth_simulator_3d.h"
#include "bone_cloth_wind_3d.h"

using namespace godot;

static constexpr double STEP = 1.0 / 60.0;
static constexpr int MAX_STEPS = 4;

static constexpr float LINK_COMPLIANCE = 1e-9f;
static constexpr float LINK_SPACING_RAMP = 0.3f;

static constexpr float WIND_BASE_SPEED = 7.5f;
static constexpr float WIND_NOISE_RATE = 2.3132f;
static constexpr float WIND_NOISE_BLEND = 0.7f;
static constexpr float WIND_TURN = Math::deg_to_rad(45.0f);
static constexpr double WIND_ROOT_SPREAD = 0.24;
static constexpr double WIND_TIP_DELAY = Math::deg_to_rad(120.0);
static constexpr float WIND_YAW_SHARE = 0.38f;
static constexpr double WIND_SWELL_SECONDS = 8.0;
static constexpr float WIND_SWELL_MIN = 0.6f;
static constexpr float WIND_CONTACT_DECAY = 0.6f;

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

// inspector에서 새로 만든 Curve는 점이 없어서 0을 돌려주므로, 곱하지 않고 1을 쓴다.
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
		joint.wind = wind_influence * sample_curve(wind_influence_damping_curve, joint.length_rate) * units_per_meter;
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

void BoneClothSimulator3D::_update_local_shares(StepMotion &r_step, const Vector3 &p_center, const Quaternion &p_center_rotation) const {
	if (local_movement_speed_limit > 0.0f) {
		const float speed = (p_center - (r_step.turn.xform(step_center) + r_step.move)).length() / float(STEP) / units_per_meter;
		if (speed > local_movement_speed_limit) {
			r_step.move_share = 1.0f - local_movement_speed_limit / speed;
		}
	}

	if (local_rotation_speed_limit > 0.0f) {
		const float turn_speed = (r_step.turn * step_center_rotation).normalized().angle_to(p_center_rotation) / float(STEP);
		if (turn_speed > local_rotation_speed_limit) {
			r_step.turn_share = 1.0f - local_rotation_speed_limit / turn_speed;
		}
	}
}

Quaternion BoneClothSimulator3D::_get_sync_swing(Skeleton3D *p_skeleton, int p_bone) const {
	const Quaternion rest = p_skeleton->get_bone_rest(p_bone).basis.get_rotation_quaternion();
	const Quaternion turn = p_skeleton->get_bone_pose_rotation(p_bone) * rest.inverse();
	const Vector3 axis = rest.xform(Vector3(0, 1, 0));
	const Vector3 along = axis * Vector3(turn.x, turn.y, turn.z).dot(axis);
	Quaternion twist(along.x, along.y, along.z, turn.w);
	twist = twist.length_squared() > 0.0f ? twist.normalized() : Quaternion();

	const int parent = p_skeleton->get_bone_parent(p_bone);
	const Quaternion parent_rotation = parent >= 0 ? p_skeleton->get_bone_global_pose(parent).basis.get_rotation_quaternion() : Quaternion();
	return parent_rotation * (turn * twist.inverse()) * parent_rotation.inverse();
}

void BoneClothSimulator3D::_apply_sync(Skeleton3D *p_skeleton) {
	if (sync_bones.is_empty()) {
		return;
	}

	LocalVector<Quaternion> swings;
	for (const SyncBone &sync_bone : sync_bones) {
		swings.push_back(sync_bone.bone >= 0 ? _get_sync_swing(p_skeleton, sync_bone.bone) : Quaternion());
	}

	LocalVector<Quaternion> turns;
	LocalVector<Vector3> pivots;
	turns.resize(chains.size());
	pivots.resize(chains.size());
	for (uint32_t c = 0; c < chains.size(); c++) {
		Chain &chain = chains[c];
		if (chain.joints.is_empty()) {
			continue;
		}

		if (chain.parent_chain >= 0) {
			turns[c] = turns[chain.parent_chain];
			pivots[c] = pivots[chain.parent_chain];
		} else {
			Quaternion turn;
			for (uint32_t s = 0; s < chain.sync_shares.size(); s++) {
				turn = turn * Quaternion().slerp(swings[s], sync * chain.sync_shares[s]);
			}
			turns[c] = turn;
			pivots[c] = chain.joints[0].current_pose_location;
		}

		for (Joint &joint : chain.joints) {
			joint.current_pose_location = pivots[c] + turns[c].xform(joint.current_pose_location - pivots[c]);
			joint.animated_location = pivots[c] + turns[c].xform(joint.animated_location - pivots[c]);
			joint.pose_basis = Basis(turns[c]) * joint.pose_basis;
		}
	}
}

void BoneClothSimulator3D::_step(const StepMotion &p_step, const LocalVector<PosedCollision> &p_collisions) {
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty() && chain.parent_chain < 0) {
			Joint &root = chain.joints[0];
			if (root_spring_stiffness > 0.0f) {
				_move_root(chain, p_step);
				continue;
			}

			root.prev_location = root.location;
			root.location = root.pose_location;
		}
	}

	for (Chain &chain : chains) {
		_follow_parent_chain(chain);
		_simulate(chain, p_step);
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

void BoneClothSimulator3D::_move_joint(Joint &r_joint, const StepMotion &p_step, const Vector3 &p_force) {
	const float dt = float(STEP);

	const float depth_share = depth_inertia * (1.0f - r_joint.length_rate * r_joint.length_rate);
	const float share = p_step.move_share + (1.0f - p_step.move_share) * depth_share;
	const float turn_share = p_step.turn_share + (1.0f - p_step.turn_share) * depth_share;
	if (share > 0.0f || turn_share > 0.0f) {
		const Vector3 move = p_step.center_move * share;
		const Quaternion turn = Quaternion().slerp(p_step.center_turn, turn_share);
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
	velocity += p_force * dt;
	r_joint.location += velocity * dt;

	r_joint.location += (p_step.move - prev_step_move) * (1.0f - share);
	r_joint.location += (p_step.turn.xform(r_joint.prev_location) - prev_step_turn.xform(r_joint.prev_location)) * (1.0f - turn_share);
}

void BoneClothSimulator3D::_move_root(Chain &r_chain, const StepMotion &p_step) {
	Joint &root = r_chain.joints[0];
	_move_joint(root, p_step, Vector3());

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

void BoneClothSimulator3D::_simulate(Chain &p_chain, const StepMotion &p_step) {
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint &joint = p_chain.joints[i];
		const Joint &parent = p_chain.joints[i - 1];
		_move_joint(joint, p_step, p_step.gravity_direction * joint.gravity + _get_wind_force(joint, p_chain, p_step) * joint.wind);
		joint.contact *= WIND_CONTACT_DECAY;
		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * joint.stiffness;
	}
}

static double wind_noise(double p_u, double p_seed) {
	const double cell = Math::floor(p_u);
	const double t = p_u - cell;
	const double smooth = t * t * (3.0 - 2.0 * t);
	const double a = Math::fposmod(Math::sin((cell + p_seed) * 12.9898) * 43758.5453, 1.0);
	const double b = Math::fposmod(Math::sin((cell + 1.0 + p_seed) * 12.9898) * 43758.5453, 1.0);
	return Math::lerp(a, b, smooth) * 2.0 - 1.0;
}

Vector3 BoneClothSimulator3D::_get_wind_force(const Joint &p_joint, const Chain &p_chain, const StepMotion &p_step) const {
	if (p_joint.wind == 0.0f || p_step.wind_speed < 0.01f) {
		return Vector3();
	}

	const double phase = wind_seed + double(p_chain.wind_index) * WIND_ROOT_SPREAD * (1.0 - wind_sync);
	const double sine = Math::sin(phase + wind_time * 10.0 - p_joint.length_rate * WIND_TIP_DELAY);
	const double noise = wind_noise(phase + wind_time * WIND_NOISE_RATE, 1.7);
	const float wave = float(Math::lerp(sine, noise, double(WIND_NOISE_BLEND)));

	const float turbulence = p_step.wind_turbulence * wind_turbulence;
	const float pitch = wave * WIND_TURN * turbulence;
	const float yaw = wave * WIND_TURN * WIND_YAW_SHARE * turbulence;
	const Vector3 direction = p_step.wind_forward * (Math::cos(pitch) * Math::cos(yaw)) + p_step.wind_up * Math::sin(pitch) + p_step.wind_side * (Math::cos(pitch) * Math::sin(yaw));

	const float lull = MIN((wave + 1.0f) * 0.5f * CLAMP(1.0f - p_step.wind_speed / WIND_BASE_SPEED, 0.0f, 1.0f) * turbulence, 1.0f);
	Vector3 force = direction * (p_step.wind_speed * (1.0f - lull) * p_step.wind_swell);
	const float into = force.dot(p_joint.contact_normal);
	if (into < 0.0f) {
		force -= p_joint.contact_normal * (into * p_joint.contact);
	}
	return force;
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
				joint.contact = 1.0f;
				joint.contact_normal = normal;
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

void BoneClothSimulator3D::_read_frame(Skeleton3D *p_skeleton) {
	for (Chain &chain : chains) {
		if (!chain.joints.is_empty()) {
			_read_pose(p_skeleton, chain);
			_update_joint_settings(chain);
		}
	}

	_apply_sync(p_skeleton);
	_read_center(p_skeleton);
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
	const BoneClothWind3D::Total wind = BoneClothWind3D::get_total_wind(this);
	motion.wind_speed = wind.force.length();
	motion.wind_turbulence = wind.turbulence;
	motion.wind_rate = MIN((0.2f + motion.wind_speed / WIND_BASE_SPEED * 0.5f) * wind.frequency * wind_frequency, 1.5f);
	if (motion.wind_speed >= 0.01f) {
		const Vector3 forward = wind.force / motion.wind_speed;
		const Vector3 cross = forward.cross(Vector3(0, 1, 0));
		const Vector3 side = cross.is_zero_approx() ? forward.get_any_perpendicular() : cross.normalized();
		const Vector3 up = side.cross(forward);
		motion.wind_forward = motion.to_skeleton.xform(forward).normalized();
		motion.wind_up = motion.to_skeleton.xform(up).normalized();
		motion.wind_side = motion.to_skeleton.xform(side).normalized();
	}

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
		StepMotion step_motion;
		step_motion.gravity_direction = r_motion.gravity_direction;
		step_motion.wind_forward = r_motion.wind_forward;
		step_motion.wind_up = r_motion.wind_up;
		step_motion.wind_side = r_motion.wind_side;
		step_motion.wind_speed = r_motion.wind_speed;
		step_motion.wind_turbulence = r_motion.wind_turbulence;
		wind_time += r_motion.wind_rate * STEP;
		wind_seconds += STEP;
		step_motion.wind_swell = Math::lerp(WIND_SWELL_MIN, 1.0f, float(wind_noise(wind_seconds / WIND_SWELL_SECONDS, wind_swell_seed)) * 0.5f + 0.5f);
		step_motion.move = r_motion.step_move;
		step_motion.turn = r_motion.step_turn;
		step_motion.center_move = center - step_center;
		step_motion.center_turn = (center_rotation * step_center_rotation.inverse()).normalized();
		_update_local_shares(step_motion, center, center_rotation);

		_step(step_motion, p_collisions);

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

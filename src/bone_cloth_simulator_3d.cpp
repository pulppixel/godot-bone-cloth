#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>


using namespace godot;

// KawaiiPhysics' fixed substeps: 60 a second, at most 4 a frame, time beyond that dropped.
// Every per step value (damping, stiffness, link_stiffness) is per 1/60 s.
static constexpr double STEP = 1.0 / 60.0;
static constexpr int MAX_STEPS = 4;

// KawaiiPhysics' default link compliance, Leather.
static constexpr float LINK_COMPLIANCE = 1e-9f;

// The chain from root_bone down to end_bone, plus a tip past the end bone when end_bone_length is above 0.
// Without the tip the end bone has no point below it and cannot turn.
bool BoneClothSimulator3D::_build_joints(Skeleton3D* p_skeleton, Chain& p_chain)
{
	p_chain.joints.clear();
	const int root = p_skeleton->find_bone(p_chain.root_bone);
	const int end = p_skeleton->find_bone(p_chain.end_bone);
	ERR_FAIL_COND_V_MSG(root < 0, false, "Root bone not found: '" + p_chain.root_bone + "'.");
	ERR_FAIL_COND_V_MSG(end < 0, false, "End bone not found: '" + p_chain.end_bone + "'.");

	LocalVector<int> bones;
	for (int bone = end; bone != root; bone = p_skeleton->get_bone_parent(bone)) {
		ERR_FAIL_COND_V_MSG(bone < 0, false, "End bone '" + p_chain.end_bone + "' is not under root bone '" + p_chain.root_bone + "'.");
		bones.push_back(bone);
	}

	bones.push_back(root);
	bones.reverse();

	for (int bone : bones) {
		Joint joint;
		joint.bone = bone;
		p_chain.joints.push_back(joint);
	}

	if (end_bone_length > 0.0f) {
		p_chain.joints.push_back(Joint());
	}

	const Transform3D rest = p_skeleton->get_bone_rest(end);
	p_chain.end_axis = rest.basis.xform_inv(rest.origin).normalized();

	return true;
}

// Links between neighbouring chains in the order they are listed, at every depth below the roots, as KawaiiPhysics' ring helper pairs skirt columns.
// A tip links only to a tip, as bAutoAddChildDummyBoneConstraint does. The length is the rest distance.
void BoneClothSimulator3D::_build_links(Skeleton3D* p_skeleton)
{
	links.clear();
	const uint32_t count = chains.size();
	if (link_mode == LINK_MODE_NONE || count < 2) {
		return;
	}

	auto rest_location = [&](const Chain& p_chain, uint32_t p_depth) {
		const Joint& joint = p_chain.joints[p_depth];
		if (joint.bone >= 0) {
			return p_skeleton->get_bone_global_rest(joint.bone).origin;
		}

		const Transform3D end = p_skeleton->get_bone_global_rest(p_chain.joints[p_depth - 1].bone);
		return end.origin + end.basis.orthonormalized().xform(p_chain.end_axis) * end_bone_length;
		};

	// The last chain links back to the first only with three or more, or the two would be linked twice.
	const uint32_t pair_count = (link_mode == LINK_MODE_LOOP && count >= 3) ? count : count - 1;
	for (uint32_t a = 0; a < pair_count; a++) {
		const uint32_t b = (a + 1) % count;
		const Chain& chain_a = chains[a];
		const Chain& chain_b = chains[b];
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

// Where the animation, and any modifier before this one, put the chain this frame.
void BoneClothSimulator3D::_read_pose(Skeleton3D* p_skeleton, Chain& p_chain)
{
	for (Joint& joint : p_chain.joints) {
		if (joint.bone >= 0) {
			const Transform3D pose = p_skeleton->get_bone_global_pose(joint.bone);
			joint.current_pose_location = pose.origin;
			joint.pose_basis = pose.basis;
		}
	}

	Joint& tip = p_chain.joints[p_chain.joints.size() - 1];
	if (tip.bone < 0) {
		const Joint& end = p_chain.joints[p_chain.joints.size() - 2];
		tip.current_pose_location = end.current_pose_location + end.pose_basis.orthonormalized().xform(p_chain.end_axis) * end_bone_length;
	}
}

// One step of KawaiiPhysics' SimulateOnce, in its order.
void BoneClothSimulator3D::_step(const Vector3& p_gravity, const Vector3& p_move, const Quaternion& p_turn, const LocalVector<BoneClothCapsule3D*>& p_capsules)
{
	// The roots are kinematic: they follow the pose.
	for (Chain& chain : chains) {
		if (!chain.joints.is_empty()) {
			Joint& root = chain.joints[0];
			root.prev_location = root.location;
			root.location = root.pose_location;
		}
	}

	for (Chain& chain : chains) {
		_simulate(chain, p_gravity, p_move, p_turn);
	}

	// KawaiiPhysics solves the links once before the collision and once after it.
	_solve_links();
	for (Chain& chain : chains) {
		_collide(chain, p_capsules);
	}

	_collide_links(p_capsules);
	_solve_links();

	for (Chain& chain : chains) {
		_restore_limits_and_lengths(chain);
	}
}

// KawaiiPhysics' Simulate for every joint below the root: Verlet with damping and gravity, the character's own motion, then a pull toward the pose.
// p_move and p_turn are the skeleton's motion this step as the cloth feels it, seen from where the skeleton is now.
void BoneClothSimulator3D::_simulate(Chain& p_chain, const Vector3& p_gravity, const Vector3& p_move, const Quaternion& p_turn)
{
	const float dt = float(STEP);

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint& joint = p_chain.joints[i];
		const Joint& parent = p_chain.joints[i - 1];

		Vector3 velocity = (joint.location - joint.prev_location) / dt;
		joint.prev_location = joint.location;
		velocity *= 1.0f - damping;
		velocity += p_gravity * dt;
		joint.location += velocity * dt;

		// Only a change in the skeleton's motion moves the cloth against it: keeping a speed or a turn rate carries the cloth along, as Magica Cloth 2's world inertia does.
		// KawaiiPhysics' world damping adds a share of the motion itself every step, a drag that lifts a skirt while she runs at a steady speed.
		joint.location += p_move - prev_step_move;
		joint.location += p_turn.xform(joint.prev_location) - prev_step_turn.xform(joint.prev_location);

		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * stiffness;
	}
}

// One XPBD pass over the links with equal masses.
// KawaiiPhysics resets lambda before each pass and runs one iteration, so its lambda term is always zero and left out here.
// link_stiffness scales the correction: 1 is KawaiiPhysics' link, 0 keeps no distance and leaves the links to the collision.
void BoneClothSimulator3D::_solve_links()
{
	if (link_stiffness <= 0.0f) {
		return;
	}

	const float compliance = LINK_COMPLIANCE / float(STEP * STEP);

	for (const Link& link : links) {
		Joint& joint_a = chains[link.chain_a].joints[link.depth];
		Joint& joint_b = chains[link.chain_b].joints[link.depth];
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

// Every joint below the root against every capsule, as KawaiiPhysics' collision pass.
// The roots follow the pose and are never pushed.
void BoneClothSimulator3D::_collide(Chain& p_chain, const LocalVector<BoneClothCapsule3D*>& p_capsules)
{
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint& joint = p_chain.joints[i];
		for (const BoneClothCapsule3D* capsule : p_capsules) {
			joint.location = capsule->collide(joint.location, radius);
		}
	}
}

// Every link against every capsule as a line, where KawaiiPhysics feeds its bridge points' collision back to the two ends.
void BoneClothSimulator3D::_collide_links(const LocalVector<BoneClothCapsule3D*>& p_capsules)
{
	for (const Link& link : links) {
		Joint& joint_a = chains[link.chain_a].joints[link.depth];
		Joint& joint_b = chains[link.chain_b].joints[link.depth];
		for (const BoneClothCapsule3D* capsule : p_capsules) {
			capsule->collide_segment(joint_a.location, joint_b.location, radius);
		}
	}
}

// KawaiiPhysics' RestoreBoneLengthsAndLimits, last in the step so the limit and the length win over the links and the collision.
// The angle limit is a cone of limit_angle around the bone's pose direction: a joint leaning further out is turned back onto the cone (AdjustByAngleLimit).
// Then each joint goes back to its pose distance from the parent.
void BoneClothSimulator3D::_restore_limits_and_lengths(Chain& p_chain)
{
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		Joint& joint = p_chain.joints[i];
		const Joint& parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.pose_location - parent.pose_location;

		if (limit_angle > 0.0f) {
			const Vector3 offset = joint.location - parent.location;
			const Vector3 pose_direction = pose_vector.normalized();
			const Vector3 direction = offset.normalized();
			const Vector3 axis = pose_direction.cross(direction);
			const float angle = Math::atan2(axis.length(), pose_direction.dot(direction));
			if (angle > limit_angle) {
				// Pointing straight back the cross product vanishes and any axis across the pose direction turns it back.
				// KawaiiPhysics takes the parent's X axis there, but which bone axis lies across the bone differs between rigs.
				const Vector3 turn_axis = axis.is_zero_approx() ? pose_direction.get_any_perpendicular() : axis.normalized();
				joint.location = parent.location + offset.rotated(turn_axis, limit_angle - angle);
			}
		}

		joint.location = parent.location + (joint.location - parent.location).normalized() * pose_vector.length();
	}
}

// Each bone turns by the arc from its pose direction to its simulated direction.
// Rotation only: the lengths stay the pose's. Root first, because set_bone_global_pose works out the local pose from the parent's global pose.
// The root keeps its pose location, as KawaiiPhysics writes only the bones below it: in a frame with no step its simulated location is a frame old.
void BoneClothSimulator3D::_write_rotations(Skeleton3D* p_skeleton, const Chain& p_chain)
{
	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		const Joint& joint = p_chain.joints[i];
		const Joint& parent = p_chain.joints[i - 1];
		const Vector3 pose_vector = joint.current_pose_location - parent.current_pose_location;
		const Vector3 sim_vector = joint.location - parent.location;
		const Basis basis = Basis(Quaternion(pose_vector, sim_vector)) * parent.pose_basis;
		const Vector3 origin = i == 1 ? parent.current_pose_location : parent.location;
		p_skeleton->set_bone_global_pose(parent.bone, Transform3D(basis, origin));
	}
}

void BoneClothSimulator3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_chain_count", "count"), &BoneClothSimulator3D::set_chain_count);
	ClassDB::bind_method(D_METHOD("get_chain_count"), &BoneClothSimulator3D::get_chain_count);
	ClassDB::bind_method(D_METHOD("set_link_mode", "mode"), &BoneClothSimulator3D::set_link_mode);
	ClassDB::bind_method(D_METHOD("get_link_mode"), &BoneClothSimulator3D::get_link_mode);
	ClassDB::bind_method(D_METHOD("set_link_stiffness", "stiffness"), &BoneClothSimulator3D::set_link_stiffness);
	ClassDB::bind_method(D_METHOD("get_link_stiffness"), &BoneClothSimulator3D::get_link_stiffness);

	ClassDB::bind_method(D_METHOD("set_end_bone_length", "length"), &BoneClothSimulator3D::set_end_bone_length);
	ClassDB::bind_method(D_METHOD("get_end_bone_length"), &BoneClothSimulator3D::get_end_bone_length);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &BoneClothSimulator3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &BoneClothSimulator3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothSimulator3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothSimulator3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothSimulator3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothSimulator3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothSimulator3D::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothSimulator3D::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothSimulator3D::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothSimulator3D::get_inertia);
	ClassDB::bind_method(D_METHOD("set_movement_speed_limit", "limit"), &BoneClothSimulator3D::set_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_movement_speed_limit"), &BoneClothSimulator3D::get_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_rotation_speed_limit", "limit"), &BoneClothSimulator3D::set_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_rotation_speed_limit"), &BoneClothSimulator3D::get_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_teleport_distance", "distance"), &BoneClothSimulator3D::set_teleport_distance);
	ClassDB::bind_method(D_METHOD("get_teleport_distance"), &BoneClothSimulator3D::get_teleport_distance);
	ClassDB::bind_method(D_METHOD("set_teleport_angle", "angle"), &BoneClothSimulator3D::set_teleport_angle);
	ClassDB::bind_method(D_METHOD("get_teleport_angle"), &BoneClothSimulator3D::get_teleport_angle);

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), "set_link_mode", "get_link_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "end_bone_length", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_end_bone_length", "get_end_bone_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity", PROPERTY_HINT_NONE, "suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");
	ADD_GROUP("", "");

	// The engine's ADD_ARRAY_COUNT, which godot-cpp does not have: the
	// array usage and "label,prefix" make the inspector group chains/N/... under one Chains list.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), "set_chain_count", "get_chain_count");

	BIND_ENUM_CONSTANT(LINK_MODE_NONE);
	BIND_ENUM_CONSTANT(LINK_MODE_SEQUENTIAL);
	BIND_ENUM_CONSTANT(LINK_MODE_LOOP);
}

// chains/N/root_bone and chains/N/end_bone, as SpringBoneSimulator3D's settings/N/...
bool BoneClothSimulator3D::_set(const StringName& p_name, const Variant& p_value)
{
	const String path = p_name;
	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone") {
		chains[which].root_bone = p_value;
	}
	else if (what == "end_bone") {
		chains[which].end_bone = p_value;
	}
	else {
		return false;
	}

	joints_dirty = true;
	return true;
}

bool BoneClothSimulator3D::_get(const StringName& p_name, Variant& r_ret) const
{
	const String path = p_name;
	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone") {
		r_ret = chains[which].root_bone;
	}
	else if (what == "end_bone") {
		r_ret = chains[which].end_bone;
	}
	else {
		return false;
	}

	return true;
}

// The bone names as a drop-down of the skeleton's bones, as BoneAttachment3D does.
void BoneClothSimulator3D::_get_property_list(List<PropertyInfo>* p_list) const
{
	Skeleton3D* skeleton = get_skeleton();
	const PropertyHint hint = skeleton ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE;
	const String bone_names = skeleton ? skeleton->get_concatenated_bone_names() : String();

	for (uint32_t i = 0; i < chains.size(); i++) {
		const String path = "chains/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("root_bone"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("end_bone"), hint, bone_names));
	}
}

void BoneClothSimulator3D::set_chain_count(int p_count)
{
	ERR_FAIL_COND(p_count < 0);
	chains.resize(p_count);
	joints_dirty = true;
	notify_property_list_changed();
}

int BoneClothSimulator3D::get_chain_count() const
{
	return chains.size();
}

void BoneClothSimulator3D::set_link_mode(LinkMode p_mode)
{
	link_mode = p_mode;
	joints_dirty = true;
}

BoneClothSimulator3D::LinkMode BoneClothSimulator3D::get_link_mode() const
{
	return link_mode;
}

void BoneClothSimulator3D::set_link_stiffness(float p_stiffness)
{
	link_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_link_stiffness() const
{
	return link_stiffness;
}

void BoneClothSimulator3D::set_end_bone_length(float p_length)
{
	end_bone_length = p_length;
	joints_dirty = true;
}

float BoneClothSimulator3D::get_end_bone_length() const
{
	return end_bone_length;
}

void BoneClothSimulator3D::set_damping(float p_damping)
{
	damping = p_damping;
}

float BoneClothSimulator3D::get_damping() const
{
	return damping;
}

void BoneClothSimulator3D::set_stiffness(float p_stiffness)
{
	stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_stiffness() const
{
	return stiffness;
}

void BoneClothSimulator3D::set_radius(float p_radius)
{
	radius = p_radius;
}

float BoneClothSimulator3D::get_radius() const
{
	return radius;
}

void BoneClothSimulator3D::set_limit_angle(float p_angle)
{
	limit_angle = p_angle;
}

float BoneClothSimulator3D::get_limit_angle() const
{
	return limit_angle;
}

void BoneClothSimulator3D::set_gravity(const Vector3& p_gravity)
{
	gravity = p_gravity;
}

Vector3 BoneClothSimulator3D::get_gravity() const
{
	return gravity;
}

void BoneClothSimulator3D::set_inertia(float p_inertia)
{
	inertia = p_inertia;
}

float BoneClothSimulator3D::get_inertia() const
{
	return inertia;
}

void BoneClothSimulator3D::set_movement_speed_limit(float p_limit)
{
	movement_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_movement_speed_limit() const
{
	return movement_speed_limit;
}

void BoneClothSimulator3D::set_rotation_speed_limit(float p_limit)
{
	rotation_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_rotation_speed_limit() const
{
	return rotation_speed_limit;
}

void BoneClothSimulator3D::set_teleport_distance(float p_distance)
{
	teleport_distance = p_distance;
}

float BoneClothSimulator3D::get_teleport_distance() const
{
	return teleport_distance;
}

void BoneClothSimulator3D::set_teleport_angle(float p_angle)
{
	teleport_angle = p_angle;
}

float BoneClothSimulator3D::get_teleport_angle() const
{
	return teleport_angle;
}

void BoneClothSimulator3D::reset()
{
	needs_reset = true;
}

void BoneClothSimulator3D::_process_modification_with_delta(double p_delta)
{
	Skeleton3D* skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	// A chain that fails to build prints its error once and stays empty until a setting changes; the others still run.
	if (joints_dirty) {
		joints_dirty = false;
		needs_reset = true;
		for (Chain& chain : chains) {
			if (!_build_joints(skeleton, chain)) {
				chain.joints.clear();
			}
		}
		_build_links(skeleton);
	}

	for (Chain& chain : chains) {
		if (!chain.joints.is_empty()) {
			_read_pose(skeleton, chain);
		}
	}

	// Where the skeleton is drawn this frame, also between physics ticks.
	const Transform3D skeleton_transform = skeleton->get_global_transform_interpolated();

	if (needs_reset) {
		needs_reset = false;
		for (Chain& chain : chains) {
			for (Joint& joint : chain.joints) {
				joint.location = joint.current_pose_location;
				joint.prev_location = joint.current_pose_location;
				joint.pose_location = joint.current_pose_location;
				joint.prev_pose_location = joint.current_pose_location;
			}
		}

		step_time = 0.0;
		prev_skeleton_transform = skeleton_transform;
		prev_step_move = Vector3();
		prev_step_turn = Quaternion();
	}

	// The capsules follow their bones before the steps, as SpringBoneSimulator3D syncs its collisions; also while paused, so the editor shows them in place.
	LocalVector<BoneClothCapsule3D*> capsules;
	for (int i = 0; i < get_child_count(); i++) {
		BoneClothCapsule3D* capsule = Object::cast_to<BoneClothCapsule3D>(get_child(i));
		if (capsule) {
			capsule->sync_pose();
			capsules.push_back(capsule);
		}
	}

	if (p_delta > 0.0) {
		// The skeleton's move since the last step, seen from where it is now (KawaiiPhysics' UpdateSkelCompMove).
		const Transform3D move = skeleton_transform.affine_inverse() * prev_skeleton_transform;
		const Vector3 move_location = move.origin;
		const Quaternion move_rotation = move.basis.get_rotation_quaternion();

		// Magica Cloth 2's teleport check in its Keep mode: after a jump over teleport_distance or a turn over teleport_angle in one frame, the steps go on as if the last step's motion had continued.
		const bool teleported = skeleton_transform.origin.distance_to(prev_skeleton_transform.origin) > teleport_distance || move_rotation.get_angle() > teleport_angle;

		// Fixed steps (SimulateModifyBones): the frame's time is added to what was left over and spent in whole steps.
		const double elapsed = step_time + p_delta;
		step_time = MIN(elapsed, MAX_STEPS * STEP);
		const double dropped = elapsed - step_time;
		const int step_count = int(step_time / STEP);
		step_time -= step_count * STEP;

		// Each step takes its share of the move, by time, and feels the inertia share of it up to the speed limits; the rest carries the cloth along.
		const float share = float(STEP / elapsed);
		Vector3 step_move = (move_location * share * inertia).limit_length(movement_speed_limit * float(STEP));
		Quaternion step_turn = Quaternion().slerp(move_rotation, share * inertia);
		const float max_turn = rotation_speed_limit * float(STEP);
		if (step_turn.get_angle() > max_turn) {
			step_turn = Quaternion(step_turn.get_axis(), max_turn);
		}

		if (teleported) {
			step_move = prev_step_move;
			step_turn = prev_step_turn;
		}

		const Vector3 skeleton_gravity = skeleton_transform.basis.inverse().xform(gravity);

		for (int step = 0; step < step_count; step++) {
			// The pose targets move from last frame's to this frame's across the steps.
			const float weight = float(step + 1) / float(step_count);
			for (Chain& chain : chains) {
				for (Joint& joint : chain.joints) {
					joint.pose_location = joint.prev_pose_location.lerp(joint.current_pose_location, weight);
				}
			}

			_step(skeleton_gravity, step_move, step_turn, capsules);
			prev_step_move = step_move;
			prev_step_turn = step_turn;
		}

		// The move not yet spent waits for the next step; a teleport and dropped time are thrown away (KawaiiPhysics' AdvancePreSkelCompTransform).
		const double spent = teleported ? 1.0 : (step_count * STEP + dropped) / elapsed;
		prev_skeleton_transform = prev_skeleton_transform.interpolate_with(skeleton_transform, float(spent));
	}

	for (Chain& chain : chains) {
		for (Joint& joint : chain.joints) {
			joint.prev_pose_location = joint.current_pose_location;
		}

		_write_rotations(skeleton, chain);
	}

}

#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>


using namespace godot;

// KawaiiPhysics' TargetFramerate (AnimNode_KawaiiPhysics.h)
static constexpr double TARGET_FPS = 60.0;

// The chain from root_bone down to end_bone, plus a tip past the end bone when end_bone_length is above 0.
// Without the tip the end bone has no point below it and cannot turn.
bool BoneClothSimulator3D::_build_joints(Skeleton3D* p_skeleton)
{
	joints.clear();
	const int root = p_skeleton->find_bone(root_bone);
	const int end = p_skeleton->find_bone(end_bone);
	ERR_FAIL_COND_V_MSG(root < 0, false, "Root bone not found: '" + root_bone + "'.");
	ERR_FAIL_COND_V_MSG(end < 0, false, "End bone not found: '" + end_bone + "'.");

	LocalVector<int> chain;
	for (int bone = end; bone != root; bone = p_skeleton->get_bone_parent(bone)) {
		ERR_FAIL_COND_V_MSG(bone < 0, false, "End bone '" + end_bone + "' is not under root bone '" + root_bone + "'.");
		chain.push_back(bone);
	}

	chain.push_back(root);
	chain.reverse();

	for (int bone : chain) {
		Joint joint;
		joint.bone = bone;
		joints.push_back(joint);
	}

	if (end_bone_length > 0.0f) {
		joints.push_back(Joint());
	}

	const Transform3D rest = p_skeleton->get_bone_rest(end);
	end_axis = rest.basis.xform_inv(rest.origin).normalized();

	return true;
}

// Where the animation, and any modifier before this one, put the chain this frame.
void BoneClothSimulator3D::_read_pose(Skeleton3D* p_skeleton)
{
	for (Joint& joint : joints) {
		if (joint.bone >= 0) {
			const Transform3D pose = p_skeleton->get_bone_global_pose(joint.bone);
			joint.pose_location = pose.origin;
			joint.pose_basis = pose.basis;
		}
	}

	Joint& tip = joints[joints.size() - 1];
	if (tip.bone < 0) {
		const Joint& end = joints[joints.size() - 2];
		tip.pose_location = end.pose_location + end.pose_basis.orthonormalized().xform(end_axis) * end_bone_length;
	}
}

// One step of KawaiiPhysics' Simulate for every joint below the root: Verlet with damping and gravity, then a pull toward the pose.
void BoneClothSimulator3D::_simulate(double p_delta, const Vector3& p_gravity)
{
	// Stiffness is per step at TARGET_FPS; the power makes the pull the same at any frame rate
	const float pull = 1.0f - Math::pow(1.0f - stiffness, float(TARGET_FPS * p_delta));
	const float dt = float(p_delta);

	for (uint32_t i = 1; i < joints.size(); i++) {
		Joint& joint = joints[i];
		const Joint& parent = joints[i - 1];

		Vector3 velocity = (joint.location - joint.prev_location) / float(delta_old);
		joint.prev_location = joint.location;
		velocity *= 1.0f - damping;
		velocity += p_gravity * dt;
		joint.location += velocity * dt;

		const Vector3 target = parent.location + (joint.pose_location - parent.pose_location);
		joint.location += (target - joint.location) * pull;
	}

	delta_old = p_delta;
}

// Each joint back to its pose distance from the parent.
void BoneClothSimulator3D::_restore_lengths()
{
	for (uint32_t i = 1; i < joints.size(); i++) {
		Joint& joint = joints[i];
		const Joint& parent = joints[i - 1];
		const float length = (joint.pose_location - parent.pose_location).length();
		joint.location = parent.location + (joint.location - parent.location).normalized() * length;
	}
}

// Each bone turns by the arc from its pose direction to its simulated direction.
// Rotation only: the lengths stay the pose's. Root first, because set_bone_global_pose works out the local pose from the parent's global pose.
void BoneClothSimulator3D::_write_rotations(Skeleton3D* p_skeleton)
{
	for (uint32_t i = 1; i < joints.size(); i++) {
		const Joint& joint = joints[i];
		const Joint& parent = joints[i - 1];
		const Vector3 pose_vector = joint.pose_location - parent.pose_location;
		const Vector3 sim_vector = joint.location - parent.location;
		const Basis basis = Basis(Quaternion(pose_vector, sim_vector)) * parent.pose_basis;
		p_skeleton->set_bone_global_pose(parent.bone, Transform3D(basis, parent.location));
	}
}


void BoneClothSimulator3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_root_bone", "bone"), &BoneClothSimulator3D::set_root_bone);
	ClassDB::bind_method(D_METHOD("get_root_bone"), &BoneClothSimulator3D::get_root_bone);
	ClassDB::bind_method(D_METHOD("set_end_bone", "bone"), &BoneClothSimulator3D::set_end_bone);
	ClassDB::bind_method(D_METHOD("get_end_bone"), &BoneClothSimulator3D::get_end_bone);
	ClassDB::bind_method(D_METHOD("set_end_bone_length", "length"), &BoneClothSimulator3D::set_end_bone_length);
	ClassDB::bind_method(D_METHOD("get_end_bone_length"), &BoneClothSimulator3D::get_end_bone_length);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &BoneClothSimulator3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &BoneClothSimulator3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothSimulator3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothSimulator3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "root_bone"), "set_root_bone", "get_root_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "end_bone"), "set_end_bone", "get_end_bone");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "end_bone_length", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_end_bone_length", "get_end_bone_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity", PROPERTY_HINT_NONE, "suffix:m/s^2"), "set_gravity", "get_gravity");
}

// The bone names as a drop-down of the skeleton's bones, as BoneAttachment3D does.
void BoneClothSimulator3D::_validate_property(PropertyInfo& p_property) const
{
	if (p_property.name != StringName("root_bone") && p_property.name != StringName("end_bone")) {
		return;
	}

	Skeleton3D* skeleton = get_skeleton();
	if (skeleton) {
		p_property.hint = PROPERTY_HINT_ENUM;
		p_property.hint_string = skeleton->get_concatenated_bone_names();
	}
}

void BoneClothSimulator3D::set_root_bone(const String& p_bone)
{
	root_bone = p_bone;
	joints_dirty = true;
}

String BoneClothSimulator3D::get_root_bone() const
{
	return root_bone;
}

void BoneClothSimulator3D::set_end_bone(const String& p_bone)
{
	end_bone = p_bone;
	joints_dirty = true;
}

String BoneClothSimulator3D::get_end_bone() const
{
	return end_bone;
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

void BoneClothSimulator3D::set_gravity(const Vector3& p_gravity)
{
	gravity = p_gravity;
}

Vector3 BoneClothSimulator3D::get_gravity() const
{
	return gravity;
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

	// A failed build prints its error once and leaves the chain empty until a bone setting changes.
	if (joints_dirty) {
		joints_dirty = false;
		needs_reset = true;
		if (!_build_joints(skeleton)) {
			joints.clear();
		}
	}

	if (joints.is_empty()) {
		return;
	}

	_read_pose(skeleton);

	if (needs_reset) {
		needs_reset = false;
		for (Joint& joint : joints) {
			joint.location = joint.pose_location;
			joint.prev_location = joint.pose_location;
		}

		delta_old = 1.0 / TARGET_FPS;
	}

	// The root is kinematic: it follows the pose.
	Joint& root = joints[0];
	root.prev_location = root.location;
	root.location = root.pose_location;

	if (p_delta > 0.0) {
		const Vector3 skeleton_gravity = skeleton->get_global_transform().basis.inverse().xform(gravity);
		_simulate(p_delta, skeleton_gravity);
	}

	_restore_lengths();
	_write_rotations(skeleton);
}

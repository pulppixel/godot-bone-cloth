#include "bone_cloth_simulator_3d.h"

using namespace godot;

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

static bool same_paths(const LocalVector<NodePath> &p_a, const LocalVector<NodePath> &p_b) {
	if (p_a.size() != p_b.size()) {
		return false;
	}
	for (uint32_t i = 0; i < p_a.size(); i++) {
		if (p_a[i] != p_b[i]) {
			return false;
		}
	}
	return true;
}

void BoneClothSimulator3D::_build_same_chains() {
	const float tolerance = 0.0001f * units_per_meter;
	for (uint32_t c = 0; c < chains.size(); c++) {
		Chain &chain = chains[c];
		chain.same_as = -1;
		if (chain.joints.is_empty() || chain.parent_chain >= 0) {
			continue;
		}

		for (uint32_t other = 0; other < c && chain.same_as < 0; other++) {
			const Chain &first = chains[other];
			if (first.same_as >= 0 || first.parent_chain >= 0 || first.joints.size() != chain.joints.size() || first.enable_all_child_collisions != chain.enable_all_child_collisions || !same_paths(first.exclude_collisions, chain.exclude_collisions) || !same_paths(first.collisions, chain.collisions)) {
				continue;
			}

			bool same = true;
			for (uint32_t j = 0; j < chain.joints.size() && same; j++) {
				same = (first.joints[j].bone < 0) == (chain.joints[j].bone < 0) && first.joints[j].rest_location.distance_to(chain.joints[j].rest_location) <= tolerance;
			}
			if (same) {
				chain.same_as = int(other);
			}
		}
	}
}

void BoneClothSimulator3D::_sort_around(LocalVector<uint32_t> &r_chains) const {
	struct Around {
		float angle = 0.0f;
		uint32_t chain = 0;

		bool operator<(const Around &p_other) const {
			return angle < p_other.angle;
		}
	};

	Vector3 center;
	Vector3 axis;
	LocalVector<Around> around;
	for (const uint32_t c : r_chains) {
		const LocalVector<Joint> &joints = chains[c].joints;
		if (!joints.is_empty()) {
			center += joints[0].rest_location;
			axis += joints[joints.size() - 1].rest_location - joints[0].rest_location;
			around.push_back({ 0.0f, c });
		}
	}
	if (around.is_empty() || axis.is_zero_approx()) {
		return;
	}

	center /= float(around.size());
	axis.normalize();
	const Vector3 side = axis.cross(Math::abs(axis.z) < 0.9f ? Vector3(0, 0, 1) : Vector3(1, 0, 0)).normalized();
	const Vector3 forward = side.cross(axis);
	for (Around &entry : around) {
		const LocalVector<Joint> &joints = chains[entry.chain].joints;
		const Vector3 offset = joints[MIN(1u, joints.size() - 1)].rest_location - center;
		entry.angle = Math::atan2(offset.dot(forward), offset.dot(side));
	}
	around.sort();

	r_chains.clear();
	for (const Around &entry : around) {
		r_chains.push_back(entry.chain);
	}
}

void BoneClothSimulator3D::_build_links() {
	links.clear();
	LocalVector<uint32_t> linked;
	for (uint32_t c = 0; c < chains.size(); c++) {
		if ((chains[c].parent_chain < 0 || chains[c].parent_joint == 0) && chains[c].same_as < 0) {
			linked.push_back(c);
		}
	}

	if (link_mode == LINK_MODE_LOOP) {
		_sort_around(linked);
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

void BoneClothSimulator3D::_build_wind(Skeleton3D *p_skeleton) {
	wind_seed = double(String(get_path()).hash() % 1000u) * 0.0419;
	wind_swell_seed = double(String(p_skeleton->get_path()).hash() % 1000u) * 0.0419;
	uint32_t joint_count = 0;
	for (Chain &chain : chains) {
		if (chain.parent_chain >= 0 && chain.parent_joint > 0) {
			chain.wind_index = chains[chain.parent_chain].wind_index;
			continue;
		}
		chain.wind_index = joint_count;
		joint_count += chain.joints.size();
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

	LocalVector<Vector3> bone_directions;
	for (const SyncBone &sync_bone : sync_bones) {
		Vector3 outward = sync_bone.bone >= 0 ? p_skeleton->get_bone_global_rest(sync_bone.bone).origin - rest_center : Vector3();
		outward.y = 0.0f;
		bone_directions.push_back(outward.normalized());
	}

	for (Chain &chain : chains) {
		chain.sync_shares.clear();
		if (chain.joints.size() < 2) {
			continue;
		}

		Vector3 outward = p_skeleton->get_bone_global_rest(chain.end_bone).origin - rest_center;
		outward.y = 0.0f;
		outward.normalize();
		for (const Vector3 &direction : bone_directions) {
			const float share = 0.5f + 0.5f * outward.dot(direction);
			chain.sync_shares.push_back(share * share);
		}
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
		_build_same_chains();
		_build_links();
		_build_wind(p_skeleton);
		_build_sync(p_skeleton);
	}
}

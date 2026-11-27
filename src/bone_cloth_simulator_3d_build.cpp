#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>

#include <algorithm>

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
		joint.display_location = joint.rest_location;
	}

	for (uint32_t i = 1; i < p_chain.joints.size(); i++) {
		const Vector3 direction = (p_chain.joints[i].rest_location - p_chain.joints[i - 1].rest_location).normalized();
		p_chain.joints[i].twist_axis = find_twist_axis(p_skeleton->get_bone_global_rest(p_chain.joints[i - 1].bone).basis, direction);
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
	longest.resize_initialized(chains.size());

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
			if (first.same_as >= 0 || first.parent_chain >= 0 || first.joints.size() != chain.joints.size() || first.enable_all_child_collisions != chain.enable_all_child_collisions || !same_paths(chain.enable_all_child_collisions ? first.exclude_collisions : first.collisions, chain.enable_all_child_collisions ? chain.exclude_collisions : chain.collisions)) {
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

static constexpr float BEND_TRIANGLE_MAX_ANGLE = Math::deg_to_rad(120.0f);
static constexpr float BEND_TRIANGLE_MIN_ANGLE = Math::deg_to_rad(10.0f);
static constexpr float BEND_MAX_ANGLE = Math::deg_to_rad(120.0f);
static constexpr float BEND_VOLUME_MIN_ANGLE = Math::deg_to_rad(90.0f);
static constexpr float BEND_VOLUME_MAX_ANGLE = Math::deg_to_rad(179.0f);

static uint64_t pack_edge(uint32_t p_a, uint32_t p_b) {
	return (uint64_t(MIN(p_a, p_b)) << 32) | uint64_t(MAX(p_a, p_b));
}

void BoneClothSimulator3D::_build_bends() {
	bend_joints.clear();
	bends.clear();

	LocalVector<int> first_joint;
	first_joint.resize(chains.size());
	for (uint32_t c = 0; c < chains.size(); c++) {
		first_joint[c] = -1;
	}
	for (const Link &link : links) {
		for (uint32_t c : { link.chain_a, link.chain_b }) {
			if (first_joint[c] >= 0) {
				continue;
			}
			first_joint[c] = int(bend_joints.size());
			for (uint32_t depth = 0; depth < chains[c].joints.size(); depth++) {
				bend_joints.push_back({ c, depth });
			}
		}
	}

	const uint32_t count = bend_joints.size();
	LocalVector<LocalVector<uint32_t>> neighbors;
	neighbors.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		if (bend_joints[i].depth > 0) {
			neighbors[i].push_back(i - 1);
			neighbors[i - 1].push_back(i);
		}
	}

	HashSet<uint64_t> linked_pairs;
	for (const Link &link : links) {
		if (linked_pairs.has(pack_edge(link.chain_a, link.chain_b))) {
			continue;
		}
		linked_pairs.insert(pack_edge(link.chain_a, link.chain_b));
		const uint32_t a = first_joint[link.chain_a];
		const uint32_t b = first_joint[link.chain_b];
		neighbors[a].push_back(b);
		neighbors[b].push_back(a);
	}
	for (const Link &link : links) {
		const uint32_t a = first_joint[link.chain_a] + link.depth;
		const uint32_t b = first_joint[link.chain_b] + link.depth;
		neighbors[a].push_back(b);
		neighbors[b].push_back(a);
	}

	const auto rest = [&](uint32_t p_index) -> const Vector3 & {
		return chains[bend_joints[p_index].chain].joints[bend_joints[p_index].depth].rest_location;
	};

	LocalVector<Vector3i> triangles;
	HashSet<Vector3i> found;
	for (uint32_t i = 0; i < count; i++) {
		const LocalVector<uint32_t> &around = neighbors[i];
		for (uint32_t j = 0; j + 1 < around.size(); j++) {
			for (uint32_t k = j + 1; k < around.size(); k++) {
				const uint32_t a = around[j];
				const uint32_t b = around[k];
				const Vector3 to_a = rest(a) - rest(i);
				const Vector3 to_b = rest(b) - rest(i);
				if (to_a.length() < 1e-6f || to_b.length() < 1e-6f || to_a.cross(to_b).length() < 1e-9f) {
					continue;
				}

				const float angle = to_a.angle_to(to_b);
				if (angle >= BEND_TRIANGLE_MAX_ANGLE || angle <= BEND_TRIANGLE_MIN_ANGLE) {
					continue;
				}

				const uint32_t chain = bend_joints[i].chain;
				if (bend_joints[a].chain != chain && bend_joints[b].chain != chain && bend_joints[a].chain != bend_joints[b].chain) {
					continue;
				}

				uint32_t sorted[3] = { i, a, b };
				std::sort(sorted, sorted + 3);
				const Vector3i triangle(sorted[0], sorted[1], sorted[2]);
				if (!found.has(triangle)) {
					found.insert(triangle);
					triangles.push_back(triangle);
				}
			}
		}
	}

	HashMap<uint64_t, LocalVector<uint32_t>> edge_triangles;
	LocalVector<uint64_t> edges;
	for (uint32_t t = 0; t < triangles.size(); t++) {
		const Vector3i &triangle = triangles[t];
		for (const uint64_t edge : { pack_edge(triangle.x, triangle.y), pack_edge(triangle.y, triangle.z), pack_edge(triangle.x, triangle.z) }) {
			if (!edge_triangles.has(edge)) {
				edges.push_back(edge);
			}
			edge_triangles[edge].push_back(t);
		}
	}

	HashSet<Vector4i> volumes;
	for (const uint64_t edge : edges) {
		const uint32_t edge_a = uint32_t(edge >> 32);
		const uint32_t edge_b = uint32_t(edge & 0xffffffffu);
		const LocalVector<uint32_t> &shared = edge_triangles[edge];
		for (uint32_t i = 0; i + 1 < shared.size(); i++) {
			for (uint32_t j = i + 1; j < shared.size(); j++) {
				const Vector3i &first = triangles[shared[i]];
				const Vector3i &second = triangles[shared[j]];
				const uint32_t opposite_a = uint32_t(first.x + first.y + first.z) - edge_a - edge_b;
				const uint32_t opposite_b = uint32_t(second.x + second.y + second.z) - edge_a - edge_b;
				const uint32_t vertices[4] = { opposite_a, opposite_b, edge_a, edge_b };
				if (bend_joints[opposite_a].depth == 0 && bend_joints[opposite_b].depth == 0 && bend_joints[edge_a].depth == 0 && bend_joints[edge_b].depth == 0) {
					continue;
				}

				const Vector3 points[4] = { rest(opposite_a), rest(opposite_b), rest(edge_a), rest(edge_b) };
				const float angle = Math::abs(get_bend_angle(points));

				Bend bend;
				for (int v = 0; v < 4; v++) {
					bend.joints[v] = vertices[v];
				}
				if (angle < BEND_MAX_ANGLE) {
					bends.push_back(bend);
				}

				if (angle >= BEND_VOLUME_MIN_ANGLE && angle <= BEND_VOLUME_MAX_ANGLE) {
					uint32_t sorted[4] = { opposite_a, opposite_b, edge_a, edge_b };
					std::sort(sorted, sorted + 4);
					const Vector4i key(sorted[0], sorted[1], sorted[2], sorted[3]);
					if (!volumes.has(key)) {
						volumes.insert(key);
						bend.volume = true;
						bends.push_back(bend);
					}
				}
			}
		}
	}

	bend_sums.resize(count);
	bend_counts.resize(count);
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
	for (Chain &chain : chains) {
		chain.sync_inherited.clear();
		if (chain.joints.size() < 2) {
			continue;
		}

		for (const SyncBone &sync_bone : sync_bones) {
			bool inherited = false;
			for (int bone = p_skeleton->get_bone_parent(chain.joints[0].bone); bone >= 0; bone = p_skeleton->get_bone_parent(bone)) {
				if (bone == sync_bone.bone) {
					inherited = true;
					break;
				}
			}
			chain.sync_inherited.push_back(inherited);
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
		_build_bends();
		_build_wind(p_skeleton);
		_build_sync(p_skeleton);
	}
}

#include "bone_cloth_simulator_3d.h"

#include "meshoptimizer/meshoptimizer.h"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/skin.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>

#include <algorithm>

using namespace godot;

static constexpr float WELD_DISTANCE = 0.0001f;
static constexpr float FIXED_WEIGHT = 0.5f;
static constexpr float EQUILATERAL_AREA = 0.4330127f;

bool BoneClothSimulator3D::_read_mesh(Skeleton3D *p_skeleton, MeshPoints &r_points) const {
	const MeshInstance3D *mesh_node = Object::cast_to<MeshInstance3D>(get_node_or_null(mesh_instance));
	const Ref<ArrayMesh> mesh = mesh_node ? mesh_node->get_mesh() : Ref<ArrayMesh>();
	if (mesh.is_null() || mesh_node->get_skin().is_null()) {
		return false;
	}

	const Ref<Skin> skin = mesh_node->get_skin();
	LocalVector<Transform3D> binds;
	for (int b = 0; b < skin->get_bind_count(); b++) {
		int bone = skin->get_bind_bone(b);
		if (bone < 0) {
			bone = p_skeleton->find_bone(skin->get_bind_name(b));
		}
		binds.push_back(bone < 0 ? Transform3D() : p_skeleton->get_bone_global_rest(bone) * skin->get_bind_pose(b));
	}

	HashSet<int> fixed;
	for (const String &name : mesh_fixed_bones) {
		fixed.insert(p_skeleton->find_bone(name));
	}
	LocalVector<bool> fixed_binds;
	for (int b = 0; b < skin->get_bind_count(); b++) {
		const int bone = skin->get_bind_bone(b) >= 0 ? skin->get_bind_bone(b) : p_skeleton->find_bone(skin->get_bind_name(b));
		fixed_binds.push_back(fixed.has(bone));
	}

	const float weld = WELD_DISTANCE * units_per_meter;
	HashMap<Vector3i, uint32_t> welded;
	for (int s = 0; s < mesh->get_surface_count(); s++) {
		if (mesh->surface_get_primitive_type(s) != Mesh::PRIMITIVE_TRIANGLES) {
			continue;
		}

		const Array arrays = mesh->surface_get_arrays(s);
		const PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
		const PackedInt32Array bones = arrays[Mesh::ARRAY_BONES];
		const PackedFloat32Array weights = arrays[Mesh::ARRAY_WEIGHTS];
		PackedInt32Array indices = arrays[Mesh::ARRAY_INDEX];
		if (vertices.is_empty() || bones.size() != weights.size() || bones.size() % vertices.size() != 0) {
			continue;
		}
		if (indices.is_empty()) {
			indices.resize(vertices.size());
			for (int64_t i = 0; i < vertices.size(); i++) {
				indices.set(i, int32_t(i));
			}
		}

		const int64_t per_vertex = bones.size() / vertices.size();
		LocalVector<uint32_t> to_point;
		to_point.resize(vertices.size());
		for (int64_t v = 0; v < vertices.size(); v++) {
			Vector3 position;
			float fixed_weight = 0.0f;
			for (int64_t k = 0; k < per_vertex; k++) {
				const int bind = bones[v * per_vertex + k];
				const float weight = weights[v * per_vertex + k];
				if (weight <= 0.0f || bind < 0 || bind >= int(binds.size())) {
					continue;
				}
				position += binds[bind].xform(vertices[v]) * weight;
				fixed_weight += fixed_binds[bind] ? weight : 0.0f;
			}

			const Vector3i key = Vector3i((position / weld).round());
			const uint32_t *found = welded.getptr(key);
			if (found) {
				to_point[v] = *found;
				continue;
			}
			to_point[v] = r_points.positions.size();
			welded.insert(key, to_point[v]);
			r_points.positions.push_back(position);
			r_points.fixed.push_back(fixed_weight > FIXED_WEIGHT);
		}

		for (int64_t i = 0; i + 2 < indices.size(); i += 3) {
			const uint32_t a = to_point[indices[i]];
			const uint32_t b = to_point[indices[i + 1]];
			const uint32_t c = to_point[indices[i + 2]];
			if (a != b && b != c && a != c) {
				r_points.indices.push_back(a);
				r_points.indices.push_back(b);
				r_points.indices.push_back(c);
			}
		}
	}

	return !r_points.indices.is_empty();
}

void BoneClothSimulator3D::_reduce_mesh(MeshPoints &r_points) const {
	float area = 0.0f;
	for (uint32_t i = 0; i < r_points.indices.size(); i += 3) {
		const Vector3 &a = r_points.positions[r_points.indices[i]];
		area += (r_points.positions[r_points.indices[i + 1]] - a).cross(r_points.positions[r_points.indices[i + 2]] - a).length() * 0.5f;
	}

	const float spacing = mesh_point_spacing * units_per_meter;
	const float triangle_area = spacing * spacing * EQUILATERAL_AREA;
	const size_t target = MIN(size_t(area / triangle_area) * 3, size_t(r_points.indices.size()));

	LocalVector<uint32_t> reduced;
	reduced.resize(r_points.indices.size());
	const size_t count = meshopt_simplify(reduced.ptr(), r_points.indices.ptr(), r_points.indices.size(), &r_points.positions[0].x, r_points.positions.size(), sizeof(Vector3), target, spacing, meshopt_SimplifyErrorAbsolute | meshopt_SimplifyRegularize, nullptr);

	LocalVector<int> remap;
	remap.resize(r_points.positions.size());
	for (int &index : remap) {
		index = -1;
	}

	MeshPoints compact;
	for (size_t i = 0; i < count; i++) {
		const uint32_t point = reduced[i];
		if (remap[point] < 0) {
			remap[point] = int(compact.positions.size());
			compact.positions.push_back(r_points.positions[point]);
			compact.fixed.push_back(r_points.fixed[point]);
		}
		compact.indices.push_back(uint32_t(remap[point]));
	}
	r_points = compact;
}

static uint32_t find_root(LocalVector<uint32_t> &r_roots, uint32_t p_index) {
	while (r_roots[p_index] != p_index) {
		r_roots[p_index] = r_roots[r_roots[p_index]];
		p_index = r_roots[p_index];
	}
	return p_index;
}

void BoneClothSimulator3D::_join_pieces(MeshPoints &r_points) const {
	const uint32_t count = r_points.positions.size();
	LocalVector<uint32_t> roots;
	roots.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		roots[i] = i;
	}
	for (uint32_t i = 0; i < r_points.indices.size(); i += 3) {
		for (int e = 1; e < 3; e++) {
			roots[find_root(roots, r_points.indices[i])] = find_root(roots, r_points.indices[i + e]);
		}
	}

	for (uint32_t i = 0; i < count; i++) {
		roots[i] = find_root(roots, i);
	}
	LocalVector<uint32_t> sizes;
	sizes.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		sizes[i] = 0;
	}
	for (uint32_t i = 0; i < count; i++) {
		sizes[roots[i]]++;
	}
	LocalVector<uint32_t> order;
	for (uint32_t i = 0; i < count; i++) {
		order.push_back(i);
	}
	std::sort(order.ptr(), order.ptr() + count, [&](uint32_t p_a, uint32_t p_b) {
		const uint32_t root_a = roots[p_a];
		const uint32_t root_b = roots[p_b];
		return sizes[root_a] != sizes[root_b] ? sizes[root_a] > sizes[root_b] : (root_a != root_b ? root_a < root_b : p_a < p_b);
	});

	const float radius = mesh_point_spacing * units_per_meter;
	LocalVector<uint32_t> into;
	into.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		into[i] = i;
	}
	LocalVector<uint32_t> kept;
	for (uint32_t start = 0; start < count;) {
		uint32_t end = start;
		while (end < count && roots[order[end]] == roots[order[start]]) {
			end++;
		}
		for (uint32_t i = start; i < end; i++) {
			const uint32_t point = order[i];
			float nearest = radius;
			for (const uint32_t other : kept) {
				const float distance = r_points.positions[point].distance_to(r_points.positions[other]);
				if (distance <= nearest) {
					nearest = distance;
					into[point] = other;
				}
			}
		}
		for (uint32_t i = start; i < end; i++) {
			const uint32_t point = order[i];
			if (into[point] == point) {
				kept.push_back(point);
			} else {
				r_points.fixed[into[point]] = r_points.fixed[into[point]] || r_points.fixed[point];
			}
		}
		start = end;
	}

	LocalVector<int> remap;
	remap.resize(count);
	MeshPoints joined;
	for (uint32_t i = 0; i < count; i++) {
		remap[i] = -1;
		if (into[i] == i) {
			remap[i] = int(joined.positions.size());
			joined.positions.push_back(r_points.positions[i]);
			joined.fixed.push_back(r_points.fixed[i]);
		}
	}

	HashSet<Vector3i> triangles;
	for (uint32_t i = 0; i < r_points.indices.size(); i += 3) {
		uint32_t corners[3];
		for (int e = 0; e < 3; e++) {
			corners[e] = uint32_t(remap[into[r_points.indices[i + e]]]);
		}
		uint32_t sorted[3] = { corners[0], corners[1], corners[2] };
		std::sort(sorted, sorted + 3);
		const Vector3i key(sorted[0], sorted[1], sorted[2]);
		if (sorted[0] == sorted[1] || sorted[1] == sorted[2] || triangles.has(key)) {
			continue;
		}
		triangles.insert(key);
		for (const uint32_t corner : corners) {
			joined.indices.push_back(corner);
		}
	}
	r_points = joined;
}

void BoneClothSimulator3D::_grow_mesh_tree(const MeshPoints &p_points, const LocalVector<LocalVector<uint32_t>> &p_neighbors, LocalVector<int> &r_parents) const {
	const uint32_t count = p_points.positions.size();
	r_parents.resize(count);
	LocalVector<uint8_t> marks;
	marks.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		r_parents[i] = -1;
		marks[i] = 0;
	}

	struct Next {
		float distance = 0.0f;
		uint32_t point = 0;

		bool operator<(const Next &p_other) const {
			return distance < p_other.distance;
		}
	};

	LocalVector<Next> next;
	for (uint32_t i = 0; i < count; i++) {
		if (p_points.fixed[i]) {
			next.push_back({ 0.0f, i });
		}
	}

	while (!next.is_empty()) {
		for (const Next &entry : next) {
			const uint32_t point = entry.point;
			if (p_points.fixed[point]) {
				continue;
			}

			const Vector3 &position = p_points.positions[point];
			float best_cost = FLT_MAX;
			for (const uint32_t other : p_neighbors[point]) {
				if (marks[other] == 0) {
					continue;
				}

				const Vector3 &other_position = p_points.positions[other];
				const float cost = p_points.fixed[other] ? position.distance_to(other_position) : (other_position - position).angle_to(p_points.positions[r_parents[other]] - other_position);
				if (cost < best_cost) {
					best_cost = cost;
					r_parents[point] = int(other);
				}
			}
			// 1은 이번 단에서 부모를 찾은 점이라 같은 단의 다음 점이 부모로 삼을 수 있다. Magica도 같다.
			if (r_parents[point] >= 0) {
				marks[point] = 1;
			}
		}

		for (const Next &entry : next) {
			marks[entry.point] = 2;
		}

		HashMap<uint32_t, float> found;
		for (const Next &entry : next) {
			for (const uint32_t other : p_neighbors[entry.point]) {
				if (marks[other] != 0) {
					continue;
				}
				const float distance = p_points.positions[entry.point].distance_to(p_points.positions[other]);
				const float *known = found.getptr(other);
				if (!known || distance < *known) {
					found[other] = distance;
				}
			}
		}

		next.clear();
		for (const KeyValue<uint32_t, float> &entry : found) {
			next.push_back({ entry.value, entry.key });
		}
		next.sort();
	}
}

void BoneClothSimulator3D::_build_mesh(Skeleton3D *p_skeleton) {
	chains.clear();
	links.clear();
	bend_joints.clear();
	bends.clear();

	MeshPoints points;
	if (!_read_mesh(p_skeleton, points)) {
		return;
	}
	_reduce_mesh(points);
	_join_pieces(points);

	const uint32_t count = points.positions.size();
	LocalVector<LocalVector<uint32_t>> neighbors;
	neighbors.resize(count);
	HashSet<uint64_t> edges;
	LocalVector<uint64_t> edge_list;
	for (uint32_t i = 0; i < points.indices.size(); i += 3) {
		for (int e = 0; e < 3; e++) {
			const uint32_t a = points.indices[i + e];
			const uint32_t b = points.indices[i + (e + 1) % 3];
			if (edges.has(pack_edge(a, b))) {
				continue;
			}
			edges.insert(pack_edge(a, b));
			edge_list.push_back(pack_edge(a, b));
			neighbors[a].push_back(b);
			neighbors[b].push_back(a);
		}
	}

	LocalVector<int> parents;
	_grow_mesh_tree(points, neighbors, parents);

	LocalVector<LocalVector<uint32_t>> children;
	children.resize(count);
	LocalVector<float> tree_length;
	tree_length.resize(count);
	LocalVector<uint32_t> heights;
	heights.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		tree_length[i] = 0.0f;
		heights[i] = 0;
		if (parents[i] >= 0) {
			children[parents[i]].push_back(i);
		}
	}

	LocalVector<uint32_t> order;
	for (uint32_t i = 0; i < count; i++) {
		if (points.fixed[i]) {
			order.push_back(i);
		}
	}
	for (uint32_t i = 0; i < order.size(); i++) {
		for (const uint32_t child : children[order[i]]) {
			tree_length[child] = tree_length[order[i]] + points.positions[child].distance_to(points.positions[order[i]]);
			order.push_back(child);
		}
	}
	float longest = 0.0f;
	for (int64_t i = int64_t(order.size()) - 1; i >= 0; i--) {
		const uint32_t point = order[i];
		longest = MAX(longest, tree_length[point]);
		if (parents[point] >= 0) {
			heights[parents[point]] = MAX(heights[parents[point]], heights[point] + 1);
		}
	}

	LocalVector<int> refs;
	refs.resize(count);
	for (int &ref : refs) {
		ref = -1;
	}
	LocalVector<JointRef> joint_refs;

	const auto add_joint = [&](Chain &r_chain, uint32_t p_point) {
		Joint joint;
		joint.rest_location = points.positions[p_point];
		joint.display_location = joint.rest_location;
		joint.length_rate = longest > 0.0f ? tree_length[p_point] / longest : 0.0f;
		r_chain.joints.push_back(joint);
	};

	struct Start {
		uint32_t point = 0;
		uint32_t first = 0;
		int parent_chain = -1;
		uint32_t parent_joint = 0;
	};
	LocalVector<Start> starts;
	for (uint32_t i = 0; i < count; i++) {
		if (points.fixed[i]) {
			starts.push_back({ i, i, -1, 0 });
		}
	}

	for (uint32_t s = 0; s < starts.size(); s++) {
		const Start start = starts[s];
		const uint32_t c = chains.size();
		chains.push_back(Chain());
		chains[c].parent_chain = start.parent_chain;
		chains[c].parent_joint = start.parent_joint;
		if (start.parent_chain >= 0) {
			add_joint(chains[c], start.point);
			chains[start.parent_chain].joints[start.parent_joint].branched = true;
		}

		uint32_t point = start.first;
		while (true) {
			refs[point] = int(joint_refs.size());
			joint_refs.push_back({ c, chains[c].joints.size() });
			add_joint(chains[c], point);
			if (children[point].is_empty()) {
				break;
			}

			uint32_t next = children[point][0];
			for (const uint32_t child : children[point]) {
				if (heights[child] > heights[next]) {
					next = child;
				}
			}
			for (const uint32_t child : children[point]) {
				if (child != next) {
					starts.push_back({ point, child, int(c), chains[c].joints.size() - 1 });
				}
			}
			point = next;
		}
	}

	for (const uint64_t edge : edge_list) {
		const uint32_t a = uint32_t(edge >> 32);
		const uint32_t b = uint32_t(edge & 0xffffffffu);
		if (refs[a] < 0 || refs[b] < 0 || (points.fixed[a] && points.fixed[b]) || parents[a] == int(b) || parents[b] == int(a)) {
			continue;
		}

		Link link;
		link.a = joint_refs[refs[a]];
		link.b = joint_refs[refs[b]];
		link.length = points.positions[a].distance_to(points.positions[b]);
		links.push_back(link);
	}

	bend_joints = joint_refs;
	LocalVector<Vector3i> triangles;
	for (uint32_t i = 0; i < points.indices.size(); i += 3) {
		const int a = refs[points.indices[i]];
		const int b = refs[points.indices[i + 1]];
		const int c = refs[points.indices[i + 2]];
		if (a >= 0 && b >= 0 && c >= 0) {
			int sorted[3] = { a, b, c };
			std::sort(sorted, sorted + 3);
			triangles.push_back(Vector3i(sorted[0], sorted[1], sorted[2]));
		}
	}
	_add_bends(triangles);
}

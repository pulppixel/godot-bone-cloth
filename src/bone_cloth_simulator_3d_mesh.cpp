#include "bone_cloth_simulator_3d.h"

#include "meshoptimizer/meshoptimizer.h"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/skin.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>

#include <algorithm>

using namespace godot;

static constexpr float WELD_DISTANCE = 0.0001f;
static constexpr float FIXED_WEIGHT = 0.5f;
static constexpr float EQUILATERAL_AREA = 0.4330127f;
static constexpr float MAPPING_REACH = 1.5f;
static constexpr float MAPPING_FALLOFF = 3.0f;
static constexpr uint64_t CUSTOM_FORMATS = uint64_t(0xfff) << Mesh::ARRAY_FORMAT_CUSTOM_BASE;

static void read_binds(const Skeleton3D *p_skeleton, const Ref<Skin> &p_skin, LocalVector<int> &r_bones, LocalVector<Transform3D> &r_rests) {
	for (int b = 0; b < p_skin->get_bind_count(); b++) {
		int bone = p_skin->get_bind_bone(b);
		if (bone < 0) {
			bone = p_skeleton->find_bone(p_skin->get_bind_name(b));
		}
		r_bones.push_back(bone);
		r_rests.push_back(bone < 0 ? Transform3D() : p_skeleton->get_bone_global_rest(bone) * p_skin->get_bind_pose(b));
	}
}

static Transform3D get_vertex_rest(const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, int64_t p_vertex, int64_t p_per_vertex, const LocalVector<Transform3D> &p_rests) {
	Transform3D sum;
	sum.basis = Basis(Vector3(), Vector3(), Vector3());
	bool found = false;
	for (int64_t k = 0; k < p_per_vertex; k++) {
		const int bind = p_bones[p_vertex * p_per_vertex + k];
		const float weight = p_weights[p_vertex * p_per_vertex + k];
		if (weight <= 0.0f || bind < 0 || bind >= int(p_rests.size())) {
			continue;
		}
		sum.basis += p_rests[bind].basis * weight;
		sum.origin += p_rests[bind].origin * weight;
		found = true;
	}
	return found ? sum : Transform3D();
}

static void get_vertex_weights(const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, int64_t p_vertex, int64_t p_per_vertex, const LocalVector<int> &p_bind_bones, int *r_bones, float *r_weights) {
	LocalVector<int> bones;
	LocalVector<float> weights;
	for (int64_t k = 0; k < p_per_vertex; k++) {
		const int bind = p_bones[p_vertex * p_per_vertex + k];
		const float weight = p_weights[p_vertex * p_per_vertex + k];
		if (weight <= 0.0f || bind < 0 || bind >= int(p_bind_bones.size()) || p_bind_bones[bind] < 0) {
			continue;
		}
		const int64_t at = bones.find(p_bind_bones[bind]);
		if (at >= 0) {
			weights[at] += weight;
		} else {
			bones.push_back(p_bind_bones[bind]);
			weights.push_back(weight);
		}
	}

	float total = 0.0f;
	for (int slot = 0; slot < 4; slot++) {
		int64_t best = -1;
		for (uint32_t i = 0; i < weights.size(); i++) {
			if (weights[i] > 0.0f && (best < 0 || weights[i] > weights[best])) {
				best = i;
			}
		}
		if (best < 0) {
			break;
		}
		r_bones[slot] = bones[best];
		r_weights[slot] = weights[best];
		total += weights[best];
		weights[best] = 0.0f;
	}
	for (int slot = 0; slot < 4; slot++) {
		r_weights[slot] = total > 0.0f ? r_weights[slot] / total : 0.0f;
	}
}

static Quaternion get_triangle_frame(const Vector3i &p_triangle, const LocalVector<Vector3> &p_positions) {
	const Vector3 &a = p_positions[p_triangle.x];
	const Vector3 edge = p_positions[p_triangle.y] - a;
	const Vector3 normal = edge.cross(p_positions[p_triangle.z] - a);
	if (normal.length_squared() < 1e-16f || edge.length_squared() < 1e-12f) {
		return Quaternion(0, 0, 0, 0);
	}
	const Vector3 z = normal.normalized();
	const Vector3 x = edge.normalized();
	return Basis(x, z.cross(x), z).get_rotation_quaternion();
}

static void map_vertex(const Vector3 &p_position, const LocalVector<Vector3> &p_points, const LocalVector<LocalVector<uint32_t>> &p_neighbors, float p_reach, int32_t *r_bones, float *r_weights) {
	uint32_t nearest = 0;
	float nearest_distance = FLT_MAX;
	for (uint32_t i = 0; i < p_points.size(); i++) {
		const float distance = p_position.distance_squared_to(p_points[i]);
		if (distance < nearest_distance) {
			nearest_distance = distance;
			nearest = i;
		}
	}

	int found[4] = { -1, -1, -1, -1 };
	float strengths[4] = { -1.0f, -1.0f, -1.0f, -1.0f };
	HashSet<uint32_t> seen;
	LocalVector<uint32_t> stack;
	stack.push_back(nearest);
	while (!stack.is_empty()) {
		const uint32_t point = stack[stack.size() - 1];
		stack.remove_at(stack.size() - 1);
		if (seen.has(point)) {
			continue;
		}
		seen.insert(point);

		const float distance = p_position.distance_to(p_points[point]);
		if (distance > p_reach) {
			continue;
		}
		const float strength = Math::pow(CLAMP(1.0f - distance / p_reach, 0.0f, 1.0f), MAPPING_FALLOFF);
		int weakest = 0;
		for (int k = 1; k < 4; k++) {
			if (strengths[k] < strengths[weakest]) {
				weakest = k;
			}
		}
		if (strength > strengths[weakest]) {
			found[weakest] = int(point);
			strengths[weakest] = strength;
		}
		for (const uint32_t next : p_neighbors[point]) {
			if (!seen.has(next) && p_position.distance_to(p_points[next]) <= p_reach) {
				stack.push_back(next);
			}
		}
	}

	float total = 0.0f;
	int count = 0;
	for (int k = 0; k < 4; k++) {
		if (found[k] >= 0) {
			total += strengths[k];
			count++;
		}
	}
	for (int k = 0; k < 4; k++) {
		r_bones[k] = MAX(found[k], 0);
		r_weights[k] = found[k] < 0 ? 0.0f : (total > 0.0f ? strengths[k] / total : 1.0f / count);
	}
	if (count == 0) {
		r_bones[0] = int32_t(nearest);
		r_weights[0] = 1.0f;
	}
}

bool BoneClothSimulator3D::_read_mesh(Skeleton3D *p_skeleton, MeshPoints &r_points) const {
	const MeshInstance3D *mesh_node = Object::cast_to<MeshInstance3D>(get_node_or_null(mesh_instance));
	const Ref<ArrayMesh> mesh = mesh_node ? mesh_node->get_mesh() : Ref<ArrayMesh>();
	if (mesh.is_null() || mesh_node->get_skin().is_null()) {
		return false;
	}

	LocalVector<int> bind_bones;
	LocalVector<Transform3D> binds;
	read_binds(p_skeleton, mesh_node->get_skin(), bind_bones, binds);

	HashSet<int> fixed;
	for (const String &name : mesh_fixed_bones) {
		fixed.insert(p_skeleton->find_bone(name));
	}
	LocalVector<bool> fixed_binds;
	for (const int bone : bind_bones) {
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
			MeshWeights point_weights;
			get_vertex_weights(bones, weights, v, per_vertex, bind_bones, point_weights.bones, point_weights.weights);
			r_points.weights.push_back(point_weights);
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
			compact.weights.push_back(r_points.weights[point]);
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
			joined.weights.push_back(r_points.weights[i]);
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
	_clear_mesh_copy();
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
		joint.bone = refs[p_point];
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

	const uint32_t reached = joint_refs.size();
	mesh_point_joints = joint_refs;
	mesh_point_weights.resize(reached);
	mesh_poses.resize(reached);
	LocalVector<Vector3> rests;
	rests.resize(reached);
	for (uint32_t i = 0; i < count; i++) {
		if (refs[i] >= 0) {
			mesh_point_weights[refs[i]] = points.weights[i];
			rests[refs[i]] = points.positions[i];
		}
	}

	mesh_triangles.clear();
	mesh_triangle_rests.clear();
	for (uint32_t i = 0; i < points.indices.size(); i += 3) {
		const Vector3i triangle(refs[points.indices[i]], refs[points.indices[i + 1]], refs[points.indices[i + 2]]);
		if (triangle.x < 0 || triangle.y < 0 || triangle.z < 0) {
			continue;
		}
		const Quaternion frame = get_triangle_frame(triangle, rests);
		if (frame.length_squared() > 0.0f) {
			mesh_triangles.push_back(triangle);
			mesh_triangle_rests.push_back(frame.inverse());
		}
	}

	mesh_skin_bones.clear();
	for (const MeshWeights &weights : mesh_point_weights) {
		for (const int bone : weights.bones) {
			if (bone >= 0 && !mesh_skin_bones.has(bone)) {
				mesh_skin_bones.push_back(bone);
			}
		}
	}
	mesh_skins.resize(p_skeleton->get_bone_count());

	LocalVector<LocalVector<uint32_t>> point_neighbors;
	point_neighbors.resize(reached);
	float edge_sum = 0.0f;
	uint32_t edge_count = 0;
	for (const uint64_t edge : edge_list) {
		const int a = refs[uint32_t(edge >> 32)];
		const int b = refs[uint32_t(edge & 0xffffffffu)];
		if (a >= 0 && b >= 0) {
			point_neighbors[a].push_back(uint32_t(b));
			point_neighbors[b].push_back(uint32_t(a));
			edge_sum += rests[a].distance_to(rests[b]);
			edge_count++;
		}
	}
	if (edge_count > 0) {
		_make_mesh_copy(p_skeleton, rests, point_neighbors, edge_sum / float(edge_count) * MAPPING_REACH);
	}
}

void BoneClothSimulator3D::_make_mesh_copy(Skeleton3D *p_skeleton, const LocalVector<Vector3> &p_rests, const LocalVector<LocalVector<uint32_t>> &p_neighbors, float p_weight_length) {
	MeshInstance3D *original = Object::cast_to<MeshInstance3D>(get_node_or_null(mesh_instance));
	const Ref<ArrayMesh> mesh = original->get_mesh();
	LocalVector<int> bind_bones;
	LocalVector<Transform3D> binds;
	read_binds(p_skeleton, original->get_skin(), bind_bones, binds);
	const bool relative = mesh->get_blend_shape_mode() == Mesh::BLEND_SHAPE_MODE_RELATIVE;

	Ref<ArrayMesh> copy_mesh;
	copy_mesh.instantiate();
	for (int b = 0; b < mesh->get_blend_shape_count(); b++) {
		copy_mesh->add_blend_shape(mesh->get_blend_shape_name(b));
	}
	copy_mesh->set_blend_shape_mode(mesh->get_blend_shape_mode());

	for (int s = 0; s < mesh->get_surface_count(); s++) {
		Array arrays = mesh->surface_get_arrays(s);
		PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
		PackedVector3Array normals = arrays[Mesh::ARRAY_NORMAL];
		PackedFloat32Array tangents = arrays[Mesh::ARRAY_TANGENT];
		const PackedInt32Array bones = arrays[Mesh::ARRAY_BONES];
		const PackedFloat32Array weights = arrays[Mesh::ARRAY_WEIGHTS];
		const int64_t count = vertices.size();
		const int64_t per_vertex = count > 0 && bones.size() == weights.size() && bones.size() % count == 0 ? bones.size() / count : 0;

		LocalVector<Transform3D> rests;
		rests.resize(count);
		PackedInt32Array point_bones;
		PackedFloat32Array point_weights;
		point_bones.resize(count * 4);
		point_weights.resize(count * 4);
		for (int64_t v = 0; v < count; v++) {
			rests[v] = get_vertex_rest(bones, weights, v, per_vertex, binds);
			vertices.set(v, rests[v].xform(vertices[v]));
			if (normals.size() == count) {
				normals.set(v, rests[v].basis.xform(normals[v]).normalized());
			}
			if (tangents.size() == count * 4) {
				const Vector3 tangent = rests[v].basis.xform(Vector3(tangents[v * 4], tangents[v * 4 + 1], tangents[v * 4 + 2])).normalized();
				tangents.set(v * 4, tangent.x);
				tangents.set(v * 4 + 1, tangent.y);
				tangents.set(v * 4 + 2, tangent.z);
			}
			map_vertex(vertices[v], p_rests, p_neighbors, p_weight_length, point_bones.ptrw() + v * 4, point_weights.ptrw() + v * 4);
		}
		arrays[Mesh::ARRAY_VERTEX] = vertices;
		if (normals.size() == count) {
			arrays[Mesh::ARRAY_NORMAL] = normals;
		}
		if (tangents.size() == count * 4) {
			arrays[Mesh::ARRAY_TANGENT] = tangents;
		}
		arrays[Mesh::ARRAY_BONES] = point_bones;
		arrays[Mesh::ARRAY_WEIGHTS] = point_weights;

		TypedArray<Array> blends = mesh->surface_get_blend_shape_arrays(s);
		for (int b = 0; b < blends.size(); b++) {
			Array blend = blends[b];
			PackedVector3Array blend_vertices = blend[Mesh::ARRAY_VERTEX];
			PackedVector3Array blend_normals = blend[Mesh::ARRAY_NORMAL];
			PackedFloat32Array blend_tangents = blend[Mesh::ARRAY_TANGENT];
			for (int64_t v = 0; v < blend_vertices.size() && v < count; v++) {
				// RELATIVE의 BlendShape는 위치의 차이라서 회전만 적용한다. skeleton.glsl이 기본 위치에 그대로 더한다.
				blend_vertices.set(v, relative ? rests[v].basis.xform(blend_vertices[v]) : rests[v].xform(blend_vertices[v]));
			}
			for (int64_t v = 0; v < blend_normals.size() && v < count; v++) {
				blend_normals.set(v, rests[v].basis.xform(blend_normals[v]));
			}
			for (int64_t v = 0; v * 4 + 2 < blend_tangents.size() && v < count; v++) {
				const Vector3 tangent = rests[v].basis.xform(Vector3(blend_tangents[v * 4], blend_tangents[v * 4 + 1], blend_tangents[v * 4 + 2]));
				blend_tangents.set(v * 4, tangent.x);
				blend_tangents.set(v * 4 + 1, tangent.y);
				blend_tangents.set(v * 4 + 2, tangent.z);
			}
			blend[Mesh::ARRAY_VERTEX] = blend_vertices;
			if (!blend_normals.is_empty()) {
				blend[Mesh::ARRAY_NORMAL] = blend_normals;
			}
			if (!blend_tangents.is_empty()) {
				blend[Mesh::ARRAY_TANGENT] = blend_tangents;
			}
			blends[b] = blend;
		}

		copy_mesh->add_surface_from_arrays(mesh->surface_get_primitive_type(s), arrays, blends, Dictionary(), BitField<Mesh::ArrayFormat>(int64_t(mesh->surface_get_format(s)) & int64_t(CUSTOM_FORMATS)));
		copy_mesh->surface_set_material(s, mesh->surface_get_material(s));
		copy_mesh->surface_set_name(s, mesh->surface_get_name(s));
	}

	Skeleton3D *copy_skeleton = memnew(Skeleton3D);
	copy_skeleton->set_name("BoneClothPoints");
	Ref<Skin> skin;
	skin.instantiate();
	for (uint32_t i = 0; i < p_rests.size(); i++) {
		copy_skeleton->add_bone("point_" + itos(i));
		copy_skeleton->set_bone_rest(i, Transform3D(Basis(), p_rests[i]));
		copy_skeleton->set_bone_pose_position(i, p_rests[i]);
		skin->add_bind(i, Transform3D(Basis(), -p_rests[i]));
	}

	MeshInstance3D *copy = memnew(MeshInstance3D);
	copy->set_name("BoneClothMesh");
	copy->set_mesh(copy_mesh);
	copy->set_skin(skin);
	copy->set_skeleton_path(NodePath(".."));
	copy->set_material_override(original->get_material_override());
	copy->set_material_overlay(original->get_material_overlay());
	copy->set_cast_shadows_setting(original->get_cast_shadows_setting());
	copy->set_transparency(original->get_transparency());
	copy->set_layer_mask(original->get_layer_mask());
	for (int s = 0; s < original->get_surface_override_material_count(); s++) {
		copy->set_surface_override_material(s, original->get_surface_override_material(s));
	}
	copy_skeleton->add_child(copy);
	original->add_child(copy_skeleton, false, Node::INTERNAL_MODE_BACK);

	mesh_original = ObjectID(original->get_instance_id());
	mesh_copy_skeleton = ObjectID(copy_skeleton->get_instance_id());
	mesh_copy = ObjectID(copy->get_instance_id());
	original->connect("visibility_changed", callable_mp(this, &BoneClothSimulator3D::_refresh_mesh_copy));
	_show_mesh_copy(true);
}

void BoneClothSimulator3D::_clear_mesh_copy() {
	MeshInstance3D *original = Object::cast_to<MeshInstance3D>(ObjectDB::get_instance(mesh_original));
	if (original) {
		const Callable refresh = callable_mp(this, &BoneClothSimulator3D::_refresh_mesh_copy);
		if (original->is_connected("visibility_changed", refresh)) {
			original->disconnect("visibility_changed", refresh);
		}
		RenderingServer::get_singleton()->instance_set_visible(original->get_instance(), original->is_visible_in_tree());
	}

	Node *copy_skeleton = Object::cast_to<Node>(ObjectDB::get_instance(mesh_copy_skeleton));
	if (copy_skeleton) {
		if (copy_skeleton->get_parent()) {
			copy_skeleton->get_parent()->remove_child(copy_skeleton);
		}
		memdelete(copy_skeleton);
	}

	mesh_original = ObjectID();
	mesh_copy_skeleton = ObjectID();
	mesh_copy = ObjectID();
	mesh_copy_shown = false;
	mesh_point_joints.clear();
}

void BoneClothSimulator3D::_show_mesh_copy(bool p_shown) {
	MeshInstance3D *original = Object::cast_to<MeshInstance3D>(ObjectDB::get_instance(mesh_original));
	MeshInstance3D *copy = Object::cast_to<MeshInstance3D>(ObjectDB::get_instance(mesh_copy));
	if (!original || !copy) {
		return;
	}

	mesh_copy_shown = p_shown;
	copy->set_visible(p_shown);
	RenderingServer::get_singleton()->instance_set_visible(original->get_instance(), original->is_visible_in_tree() && !p_shown);
}

void BoneClothSimulator3D::_refresh_mesh_copy() {
	_show_mesh_copy(mesh_copy_shown);
}

void BoneClothSimulator3D::_read_mesh_pose(Skeleton3D *p_skeleton) {
	for (const int bone : mesh_skin_bones) {
		mesh_skins[bone] = p_skeleton->get_bone_global_pose(bone) * p_skeleton->get_bone_global_rest(bone).affine_inverse();
	}

	for (uint32_t i = 0; i < mesh_point_joints.size(); i++) {
		const MeshWeights &weights = mesh_point_weights[i];
		Transform3D skin;
		skin.basis = Basis(Vector3(), Vector3(), Vector3());
		for (int k = 0; k < 4; k++) {
			if (weights.bones[k] >= 0) {
				skin.basis += mesh_skins[weights.bones[k]].basis * weights.weights[k];
				skin.origin += mesh_skins[weights.bones[k]].origin * weights.weights[k];
			}
		}
		mesh_poses[i] = Transform3D(skin.basis.orthonormalized(), skin.xform(_get_joint(mesh_point_joints[i]).rest_location));
	}

	for (Chain &chain : chains) {
		for (Joint &joint : chain.joints) {
			const Transform3D &pose = mesh_poses[joint.bone];
			joint.current_pose_location = pose.origin;
			joint.animated_location = pose.origin;
			joint.pose_basis = pose.basis;
			joint.animated_basis = pose.basis;
		}
	}
}

void BoneClothSimulator3D::_write_mesh(float p_weight) {
	Skeleton3D *copy_skeleton = Object::cast_to<Skeleton3D>(ObjectDB::get_instance(mesh_copy_skeleton));
	if (!copy_skeleton) {
		return;
	}

	const uint32_t count = mesh_point_joints.size();
	LocalVector<Vector3> positions;
	LocalVector<Quaternion> turns;
	positions.resize(count);
	turns.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		positions[i] = _get_joint(mesh_point_joints[i]).display_location;
		turns[i] = Quaternion(0, 0, 0, 0);
	}

	for (uint32_t t = 0; t < mesh_triangles.size(); t++) {
		const Quaternion frame = get_triangle_frame(mesh_triangles[t], positions);
		if (frame.length_squared() <= 0.0f) {
			continue;
		}
		const Quaternion turn = frame * mesh_triangle_rests[t];
		for (int corner = 0; corner < 3; corner++) {
			Quaternion &sum = turns[mesh_triangles[t][corner]];
			sum += sum.dot(turn) < 0.0f ? -turn : turn;
		}
	}

	for (uint32_t i = 0; i < count; i++) {
		Transform3D pose(Basis(turns[i].length_squared() > 0.0f ? turns[i].normalized() : Quaternion()), positions[i]);
		if (p_weight < 1.0f) {
			pose = mesh_poses[i].interpolate_with(pose, p_weight);
		}
		copy_skeleton->set_bone_pose_position(i, pose.origin);
		copy_skeleton->set_bone_pose_rotation(i, pose.basis.get_rotation_quaternion());
	}

	MeshInstance3D *original = Object::cast_to<MeshInstance3D>(ObjectDB::get_instance(mesh_original));
	MeshInstance3D *copy = Object::cast_to<MeshInstance3D>(ObjectDB::get_instance(mesh_copy));
	for (int b = 0; original && copy && b < original->get_blend_shape_count(); b++) {
		if (copy->get_blend_shape_value(b) != original->get_blend_shape_value(b)) {
			copy->set_blend_shape_value(b, original->get_blend_shape_value(b));
		}
	}
}

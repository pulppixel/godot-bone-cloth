#include "editor/bone_cloth_3d_gizmo_plugin.h"

#include "collision/bone_cloth_collision_3d.h"
#include "collision/bone_cloth_collision_capsule_3d.h"
#include "collision/bone_cloth_collision_plane_3d.h"
#include "collision/bone_cloth_collision_sphere_3d.h"
#include "collision/bone_cloth_collision_tapered_capsule_3d.h"

#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

// BoneClothCollision3D

void BoneClothCollision3DGizmoPlugin::_bind_methods() {
}

// The engine's colours, read from the editor settings so a change there reaches ours too. The engine
// switches its material on is_selected(), which an extension cannot reach; get_material() gives the same switch, the selected
// lines on top of the mesh as the engine's selected shader puts them, the others faded and behind it.
BoneClothCollision3DGizmoPlugin::BoneClothCollision3DGizmoPlugin() {
	Ref<EditorSettings> settings = EditorInterface::get_singleton()->get_editor_settings();
	create_material("collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_collision"), false, true);
	create_material("inside_collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_inside_collision"), false, true);
}

bool BoneClothCollision3DGizmoPlugin::_has_gizmo(Node3D *p_for_node_3d) const {
	return Object::cast_to<BoneClothCollision3D>(p_for_node_3d) != nullptr;
}

String BoneClothCollision3DGizmoPlugin::_get_gizmo_name() const {
	return "BoneClothCollision3D";
}

int32_t BoneClothCollision3DGizmoPlugin::_get_priority() const {
	return -1;
}

// The lines are also the gizmo's collision segments, so a click on them selects the shape, as CollisionShape3D's.
void BoneClothCollision3DGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo) {
	p_gizmo->clear();
	Node3D *node = p_gizmo->get_node_3d();

	PackedVector3Array lines;
	bool inside = false;

	BoneClothCollisionSphere3D *sphere = Object::cast_to<BoneClothCollisionSphere3D>(node);
	if (sphere) {
		_add_sphere(lines, sphere->get_radius());
		inside = sphere->is_inside();
	}

	BoneClothCollisionCapsule3D *capsule = Object::cast_to<BoneClothCollisionCapsule3D>(node);
	if (capsule) {
		_add_tapered_capsule(lines, capsule->get_radius(), capsule->get_radius(), capsule->get_height());
		inside = capsule->is_inside();
	}

	BoneClothCollisionTaperedCapsule3D *tapered_capsule = Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node);
	if (tapered_capsule) {
		_add_tapered_capsule(lines, tapered_capsule->get_top_radius(), tapered_capsule->get_bottom_radius(), tapered_capsule->get_height());
		inside = tapered_capsule->is_inside();
	}

	BoneClothCollisionPlane3D *plane = Object::cast_to<BoneClothCollisionPlane3D>(node);
	if (plane) {
		_add_plane(lines);
	}

	if (lines.is_empty()) {
		return;
	}
	p_gizmo->add_lines(lines, get_material(inside ? "inside_collision" : "collision", p_gizmo));
	p_gizmo->add_collision_segments(lines);
}

// An arc around p_center from angle p_from to p_to, angle 0 along p_axis_x and a quarter turn along p_axis_y, in steps of a
// sixteenth of a turn as the engine's spheres and capsules (spring_bone_3d_gizmo_plugin.cpp draw_sphere, draw_capsule). The count
// is even, so the arc's middle is a vertex: a capsule's end arcs are centred on its poles, and the drawing reaches them.
void BoneClothCollision3DGizmoPlugin::_add_arc(PackedVector3Array &r_lines, const Vector3 &p_center, const Vector3 &p_axis_x, const Vector3 &p_axis_y, float p_radius, float p_from, float p_to) {
	static constexpr float STEP_ANGLE = (float)Math::TAU / 16.0f;

	if (p_radius < CMP_EPSILON || p_to - p_from < CMP_EPSILON) {
		return;
	}
	int steps = MAX(1, (int)Math::ceil((p_to - p_from) / STEP_ANGLE - 0.001f));
	steps += steps % 2;
	const float step = (p_to - p_from) / steps;
	for (int i = 0; i < steps; i++) {
		const float from = p_from + step * i;
		const float to = from + step;
		r_lines.push_back(p_center + (p_axis_x * Math::cos(from) + p_axis_y * Math::sin(from)) * p_radius);
		r_lines.push_back(p_center + (p_axis_x * Math::cos(to) + p_axis_y * Math::sin(to)) * p_radius);
	}
}

// Three circles, one in each plane of the node, as the engine's.
void BoneClothCollision3DGizmoPlugin::_add_sphere(PackedVector3Array &r_lines, float p_radius) {
	_add_arc(r_lines, Vector3(), Vector3(1, 0, 0), Vector3(0, 1, 0), p_radius, 0.0f, Math::TAU);
	_add_arc(r_lines, Vector3(), Vector3(0, 1, 0), Vector3(0, 0, 1), p_radius, 0.0f, Math::TAU);
	_add_arc(r_lines, Vector3(), Vector3(0, 0, 1), Vector3(1, 0, 0), p_radius, 0.0f, Math::TAU);
}

// The convex hull of the two end spheres along Y, top_radius at +Y: the shape the tapered capsule collides with. KawaiiPhysics draws
// the shape its solver collides with rather than the engine's; ours collides with the hull itself.
// Drawn as the engine's capsule is: the outline in the XY and ZY planes, a ring at each end and four side lines. On a taper the side
// touches each sphere off its middle, by the side's slope toward the thin end, and the rings and outlines meet there. Equal radii
// draw BoneClothCollisionCapsule3D. The engine's capsule turns the end arcs of its XY outline inward, into the capsule; ours keeps them on the surface.
void BoneClothCollision3DGizmoPlugin::_add_tapered_capsule(PackedVector3Array &r_lines, float p_top_radius, float p_bottom_radius, float p_height) {
	const Vector3 up(0, 1, 0);
	const Vector3 top_center = up * (p_height * 0.5f - p_top_radius);
	const Vector3 bottom_center = up * (p_bottom_radius - p_height * 0.5f);

	// The slope by which the side's outward normal leans toward +Y. At the shortest height the setters allow, the smaller sphere is
	// inside the larger and the slope is a quarter turn: only the larger sphere's outline is left.
	const float distance = top_center.y - bottom_center.y;
	const float slope = distance > CMP_EPSILON ? Math::asin((p_bottom_radius - p_top_radius) / distance) : 0.0f;
	const float side_cos = Math::cos(slope);
	const float side_sin = Math::sin(slope);

	const Vector3 sides[2] = { Vector3(1, 0, 0), Vector3(0, 0, 1) };
	for (const Vector3 &side : sides) {
		_add_arc(r_lines, top_center, side, up, p_top_radius, slope, (float)Math::PI - slope);
		_add_arc(r_lines, bottom_center, side, up, p_bottom_radius, (float)Math::PI - slope, (float)Math::TAU + slope);
	}

	if (side_cos < CMP_EPSILON) {
		return;
	}
	_add_arc(r_lines, top_center + up * (p_top_radius * side_sin), Vector3(0, 0, 1), Vector3(1, 0, 0), p_top_radius * side_cos, 0.0f, Math::TAU);
	_add_arc(r_lines, bottom_center + up * (p_bottom_radius * side_sin), Vector3(0, 0, 1), Vector3(1, 0, 0), p_bottom_radius * side_cos, 0.0f, Math::TAU);

	const Vector3 directions[4] = { Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 0, 1), Vector3(0, 0, -1) };
	for (const Vector3 &direction : directions) {
		const Vector3 normal = direction * side_cos + up * side_sin;
		r_lines.push_back(top_center + normal * p_top_radius);
		r_lines.push_back(bottom_center + normal * p_bottom_radius);
	}
}

// The engine's plane: an arrow up its normal, two arrows crossed, and a dashed square of 1 m around it.
void BoneClothCollision3DGizmoPlugin::_add_plane(PackedVector3Array &r_lines) {
	static constexpr float HALF_PI = (float)Math::PI * 0.5f;
	static constexpr float ARROW_LENGTH = 0.3f;
	static constexpr float ARROW_HALF_WIDTH = 0.05f;
	static constexpr float ARROW_TOP_HALF_WIDTH = 0.1f;
	static constexpr float ARROW_TOP = 0.5f;
	static constexpr float RECT_SIZE = 1.0f;
	static constexpr int RECT_STEP_COUNT = 9;
	static constexpr float RECT_HALF_SIZE = RECT_SIZE * 0.5f;
	static constexpr float RECT_STEP = RECT_SIZE / RECT_STEP_COUNT;

	const Vector3 arrow[7] = {
		Vector3(0, ARROW_TOP, 0),
		Vector3(-ARROW_TOP_HALF_WIDTH, ARROW_LENGTH, 0),
		Vector3(-ARROW_HALF_WIDTH, ARROW_LENGTH, 0),
		Vector3(-ARROW_HALF_WIDTH, 0, 0),
		Vector3(ARROW_HALF_WIDTH, 0, 0),
		Vector3(ARROW_HALF_WIDTH, ARROW_LENGTH, 0),
		Vector3(ARROW_TOP_HALF_WIDTH, ARROW_LENGTH, 0),
	};
	for (int i = 0; i < 2; i++) {
		const Basis turn(Vector3(0, 1, 0), HALF_PI * i);
		for (int j = 0; j < 7; j++) {
			r_lines.push_back(turn.xform(arrow[j]));
			r_lines.push_back(turn.xform(arrow[(j + 1) % 7]));
		}
	}

	for (int i = 0; i < 4; i++) {
		const Basis turn(Vector3(0, 1, 0), HALF_PI * i);
		for (int j = 0; j < RECT_STEP_COUNT; j += 2) {
			r_lines.push_back(turn.xform(Vector3(RECT_HALF_SIZE, 0, RECT_HALF_SIZE - RECT_STEP * j)));
			r_lines.push_back(turn.xform(Vector3(RECT_HALF_SIZE, 0, RECT_HALF_SIZE - RECT_STEP * (j + 1))));
		}
	}
}

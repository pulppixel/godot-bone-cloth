#include "editor/bone_cloth_3d_gizmo_plugin.h"

#include "bone_cloth_simulator_3d.h"
#include "collision/bone_cloth_collision_3d.h"
#include "collision/bone_cloth_collision_capsule_3d.h"
#include "collision/bone_cloth_collision_plane_3d.h"
#include "collision/bone_cloth_collision_sphere_3d.h"
#include "collision/bone_cloth_collision_tapered_capsule_3d.h"

#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_selection.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

// An arc around p_center from angle p_from to p_to, angle 0 along p_axis_x and a quarter turn along p_axis_y, in steps of a
// sixteenth of a turn as the engine's spheres and capsules (spring_bone_3d_gizmo_plugin.cpp draw_sphere, draw_capsule). The count
// is even, so the arc's middle is a vertex: a capsule's end arcs are centered on its poles, and the drawing reaches them.
static void add_arc(PackedVector3Array &r_lines, const Vector3 &p_center, const Vector3 &p_axis_x, const Vector3 &p_axis_y, float p_radius, float p_from, float p_to) {
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

// Three circles around p_center, one in each plane of the node, as the engine's.
static void add_sphere(PackedVector3Array &r_lines, const Vector3 &p_center, float p_radius) {
	add_arc(r_lines, p_center, Vector3(1, 0, 0), Vector3(0, 1, 0), p_radius, 0.0f, Math::TAU);
	add_arc(r_lines, p_center, Vector3(0, 1, 0), Vector3(0, 0, 1), p_radius, 0.0f, Math::TAU);
	add_arc(r_lines, p_center, Vector3(0, 0, 1), Vector3(1, 0, 0), p_radius, 0.0f, Math::TAU);
}

// BoneClothCollision3D

void BoneClothCollision3DGizmoPlugin::_bind_methods() {
}

// The engine's colors, read from the editor settings so a change there reaches ours too. The engine
// switches its material on is_selected(), which an extension cannot reach; get_material() gives the same switch, the selected
// lines on top of the mesh as the engine's selected shader puts them, the others faded and behind it.
// The mirror's color is ours and fixed, as plugins fix their gizmo colors (Phantom Camera, Magica Cloth 2's GizmoUtility): orange,
// the hue opposite the collision's light blue.
BoneClothCollision3DGizmoPlugin::BoneClothCollision3DGizmoPlugin() {
	Ref<EditorSettings> settings = EditorInterface::get_singleton()->get_editor_settings();
	create_material("collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_collision"), false, true);
	create_material("inside_collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_inside_collision"), false, true);
	create_material("mirror_collision", Color(1.0, 0.7, 0.4), false, true);
	create_handle_material("handles");
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

// The handle names as CollisionShape3D's (Gizmo3DHelper::_cylinder_or_capsule_or_cone_frustum_get_handle_name); the tapered capsule's
// two radii have one handle each, on their end spheres, where the engine's cone frustum moves both with one.
String BoneClothCollision3DGizmoPlugin::_get_handle_name(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) const {
	Node3D *node = p_gizmo->get_node_3d();
	if (Object::cast_to<BoneClothCollisionSphere3D>(node)) {
		return "Radius";
	}
	if (Object::cast_to<BoneClothCollisionCapsule3D>(node)) {
		return p_handle_id < 2 ? "Height" : "Radius";
	}
	if (Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node)) {
		static const char *names[4] = { "Height", "Height", "Top Radius", "Bottom Radius" };
		return names[p_handle_id];
	}
	return "";
}

// The size the drag starts from, as the engine's: the sphere's radius, the capsule's (radius, height), the tapered capsule's
// (top radius, bottom radius, height) as Gizmo3DHelper's cone frustum.
Variant BoneClothCollision3DGizmoPlugin::_get_handle_value(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) const {
	Node3D *node = p_gizmo->get_node_3d();
	BoneClothCollisionSphere3D *sphere = Object::cast_to<BoneClothCollisionSphere3D>(node);
	if (sphere) {
		return sphere->get_radius();
	}
	BoneClothCollisionCapsule3D *capsule = Object::cast_to<BoneClothCollisionCapsule3D>(node);
	if (capsule) {
		return Vector2(capsule->get_radius(), capsule->get_height());
	}
	BoneClothCollisionTaperedCapsule3D *tapered_capsule = Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node);
	if (tapered_capsule) {
		return Vector3(tapered_capsule->get_top_radius(), tapered_capsule->get_bottom_radius(), tapered_capsule->get_height());
	}
	return Variant();
}

void BoneClothCollision3DGizmoPlugin::_begin_handle_action(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) {
	initial_value = _get_handle_value(p_gizmo, p_handle_id, p_secondary);
	initial_transform = p_gizmo->get_node_3d()->get_global_transform();
}

// How far along the line from p_from in p_direction, in the node's space when the drag began, the mouse ray passes nearest:
// Gizmo3DHelper::get_segment and the closest points it takes on the handle's axis.
float BoneClothCollision3DGizmoPlugin::_get_distance_along(const Vector3 &p_from, const Vector3 &p_direction, Camera3D *p_camera, const Vector2 &p_screen_pos) const {
	const Transform3D to_node = initial_transform.affine_inverse();
	const Vector3 ray_from = to_node.xform(p_camera->project_ray_origin(p_screen_pos));
	const Vector3 ray_to = to_node.xform(p_camera->project_ray_origin(p_screen_pos) + p_camera->project_ray_normal(p_screen_pos) * 4096.0f);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(p_from - p_direction * 4096.0f, p_from + p_direction * 4096.0f, ray_from, ray_to);
	return p_direction.dot(closest[0] - p_from);
}

// The engine's handles, Gizmo3DHelper::_cylinder_or_capsule_or_cone_frustum_set_handle: a height handle moves its own end and keeps the
// other where it was, moving the node by half the change; with Alt held both ends move and the node stays. A radius handle grows the
// height when the end spheres would not fit, and a shorter height brings the radii down; both measured from the drag's start, so a
// drag back gives the old size back. Never below 1 mm, as the engine's. The node on a bone takes its move into position_offset
// (BoneClothCollision3D::_take_offsets_from_transform). The engine's grid snap is Node3DEditor's, which an extension cannot read.
void BoneClothCollision3DGizmoPlugin::_set_handle(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary, Camera3D *p_camera, const Vector2 &p_screen_pos) {
	static constexpr float MIN_SIZE = 0.001f;

	Node3D *node = p_gizmo->get_node_3d();
	const Vector3 up(0, 1, 0);
	const Vector3 side(1, 0, 0);

	BoneClothCollisionSphere3D *sphere = Object::cast_to<BoneClothCollisionSphere3D>(node);
	if (sphere) {
		sphere->set_radius(MAX(_get_distance_along(Vector3(), side, p_camera, p_screen_pos), MIN_SIZE));
		return;
	}

	BoneClothCollisionCapsule3D *capsule = Object::cast_to<BoneClothCollisionCapsule3D>(node);
	BoneClothCollisionTaperedCapsule3D *tapered_capsule = Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node);
	if (!capsule && !tapered_capsule) {
		return;
	}

	float top_radius;
	float bottom_radius;
	float height;
	if (capsule) {
		const Vector2 initial = initial_value;
		top_radius = initial.x;
		bottom_radius = initial.x;
		height = initial.y;
	} else {
		const Vector3 initial = initial_value;
		top_radius = initial.x;
		bottom_radius = initial.y;
		height = initial.z;
	}

	if (p_handle_id < 2) {
		const float sign = p_handle_id == 0 ? 1.0f : -1.0f;
		const float distance = _get_distance_along(Vector3(), up * sign, p_camera, p_screen_pos);
		const bool keep_center = Input::get_singleton()->is_key_pressed(KEY_ALT);
		const float initial_height = height;
		height = MAX(keep_center ? distance * 2.0f : initial_height * 0.5f + distance, MIN_SIZE);
		top_radius = MIN(top_radius, height * 0.5f);
		bottom_radius = MIN(bottom_radius, height * 0.5f);
		node->set_global_position(keep_center ? initial_transform.origin : initial_transform.xform(up * ((height - initial_height) * 0.5f * sign)));
	} else if (capsule) {
		top_radius = MAX(_get_distance_along(Vector3(), side, p_camera, p_screen_pos), MIN_SIZE);
		height = MAX(height, top_radius * 2.0f);
	} else if (p_handle_id == 2) {
		const float distance = _get_distance_along(up * (height * 0.5f - top_radius), side, p_camera, p_screen_pos);
		top_radius = MAX(distance, MIN_SIZE);
		height = MAX(height, top_radius * 2.0f);
	} else {
		const float distance = _get_distance_along(up * (bottom_radius - height * 0.5f), side, p_camera, p_screen_pos);
		bottom_radius = MAX(distance, MIN_SIZE);
		height = MAX(height, bottom_radius * 2.0f);
	}

	// The height first: a radius set after it is never more than half of it, so the setters change nothing else.
	if (capsule) {
		capsule->set_height(height);
		capsule->set_radius(top_radius);
	} else {
		tapered_capsule->set_height(height);
		tapered_capsule->set_top_radius(top_radius);
		tapered_capsule->set_bottom_radius(bottom_radius);
	}
}

// One undo step for the size and the node's place, as Gizmo3DHelper::cylinder_commit_handle; Esc puts back the drag's start.
// Each setter keeps the radii within half the height, so any order of the three gives the recorded size.
void BoneClothCollision3DGizmoPlugin::_commit_handle(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary, const Variant &p_restore, bool p_cancel) {
	Node3D *node = p_gizmo->get_node_3d();

	PackedStringArray properties;
	Array initial_values;
	if (Object::cast_to<BoneClothCollisionSphere3D>(node)) {
		properties = { "radius" };
		initial_values = { initial_value };
	} else if (Object::cast_to<BoneClothCollisionCapsule3D>(node)) {
		const Vector2 initial = initial_value;
		properties = { "radius", "height", "global_position" };
		initial_values = { initial.x, initial.y, initial_transform.origin };
	} else if (Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node)) {
		const Vector3 initial = initial_value;
		properties = { "top_radius", "bottom_radius", "height", "global_position" };
		initial_values = { initial.x, initial.y, initial.z, initial_transform.origin };
	} else {
		return;
	}

	if (p_cancel) {
		for (int64_t i = 0; i < properties.size(); i++) {
			node->set(properties[i], initial_values[i]);
		}

		return;
	}

	EditorUndoRedoManager *undo_redo = EditorInterface::get_singleton()->get_editor_undo_redo();
	undo_redo->create_action("Change Collision " + _get_handle_name(p_gizmo, p_handle_id, p_secondary));
	for (int64_t i = 0; i < properties.size(); i++) {
		undo_redo->add_do_property(node, properties[i], node->get(properties[i]));
		undo_redo->add_undo_property(node, properties[i], initial_values[i]);
	}

	undo_redo->commit_action();
}

// The lines are also the gizmo's collision segments, so a click on them selects the shape, as CollisionShape3D's.
// The handles, shown by the engine only on the selected node: the sphere's radius on its X, as the engine's sphere; the capsule's
// ends and radius as the engine's capsule; the tapered capsule's ends and each end sphere's radius on its X.
void BoneClothCollision3DGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo) {
	p_gizmo->clear();
	Node3D *node = p_gizmo->get_node_3d();

	PackedVector3Array lines;
	PackedVector3Array handles;
	bool inside = false;

	BoneClothCollisionSphere3D *sphere = Object::cast_to<BoneClothCollisionSphere3D>(node);
	if (sphere) {
		add_sphere(lines, Vector3(), sphere->get_radius());
		handles.push_back(Vector3(sphere->get_radius(), 0, 0));
		inside = sphere->is_inside();
	}

	BoneClothCollisionCapsule3D *capsule = Object::cast_to<BoneClothCollisionCapsule3D>(node);
	if (capsule) {
		_add_tapered_capsule(lines, capsule->get_radius(), capsule->get_radius(), capsule->get_height());
		handles.push_back(Vector3(0, capsule->get_height() * 0.5f, 0));
		handles.push_back(Vector3(0, capsule->get_height() * -0.5f, 0));
		handles.push_back(Vector3(capsule->get_radius(), 0, 0));
		inside = capsule->is_inside();
	}

	BoneClothCollisionTaperedCapsule3D *tapered_capsule = Object::cast_to<BoneClothCollisionTaperedCapsule3D>(node);
	if (tapered_capsule) {
		const float top_radius = tapered_capsule->get_top_radius();
		const float bottom_radius = tapered_capsule->get_bottom_radius();
		const float half_height = tapered_capsule->get_height() * 0.5f;
		_add_tapered_capsule(lines, top_radius, bottom_radius, tapered_capsule->get_height());
		handles.push_back(Vector3(0, half_height, 0));
		handles.push_back(Vector3(0, -half_height, 0));
		handles.push_back(Vector3(top_radius, half_height - top_radius, 0));
		handles.push_back(Vector3(bottom_radius, bottom_radius - half_height, 0));
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

	// A mirrored node draws its shape on the mirror bone too, in the mirror's color, and a click there selects it; the handles stay on
	// the node. KawaiiPhysics draws its mirrors in a material of their own too, but leaves them out of the click.
	BoneClothCollision3D *collision = Object::cast_to<BoneClothCollision3D>(node);
	Skeleton3D *skeleton = collision->get_skeleton();
	if (skeleton && collision->has_mirror()) {
		const Transform3D to_mirror = node->get_global_transform().affine_inverse() * skeleton->get_global_transform() * collision->mirror_pose;
		PackedVector3Array mirror_lines;
		mirror_lines.resize(lines.size());
		for (int64_t i = 0; i < lines.size(); i++) {
			mirror_lines[i] = to_mirror.xform(lines[i]);
		}

		p_gizmo->add_lines(mirror_lines, get_material("mirror_collision", p_gizmo));
		p_gizmo->add_collision_segments(mirror_lines);
	}

	if (!handles.is_empty()) {
		p_gizmo->add_handles(handles, get_material("handles"), PackedInt32Array());
	}
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
		add_arc(r_lines, top_center, side, up, p_top_radius, slope, (float)Math::PI - slope);
		add_arc(r_lines, bottom_center, side, up, p_bottom_radius, (float)Math::PI - slope, (float)Math::TAU + slope);
	}

	if (side_cos < CMP_EPSILON) {
		return;
	}
	add_arc(r_lines, top_center + up * (p_top_radius * side_sin), Vector3(0, 0, 1), Vector3(1, 0, 0), p_top_radius * side_cos, 0.0f, Math::TAU);
	add_arc(r_lines, bottom_center + up * (p_bottom_radius * side_sin), Vector3(0, 0, 1), Vector3(1, 0, 0), p_bottom_radius * side_cos, 0.0f, Math::TAU);

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

// BoneClothSimulator3D

void BoneClothSimulator3DGizmoPlugin::_bind_methods() {
}

// The engine's joint color, read from the editor settings, selected and faded as the collisions are.
BoneClothSimulator3DGizmoPlugin::BoneClothSimulator3DGizmoPlugin() {
	Ref<EditorSettings> settings = EditorInterface::get_singleton()->get_editor_settings();
	create_material("joint", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_joint"), false, true);
}

bool BoneClothSimulator3DGizmoPlugin::_has_gizmo(Node3D *p_for_node_3d) const {
	return Object::cast_to<BoneClothSimulator3D>(p_for_node_3d) != nullptr;
}

String BoneClothSimulator3DGizmoPlugin::_get_gizmo_name() const {
	return "BoneClothSimulator3D";
}

int32_t BoneClothSimulator3DGizmoPlugin::_get_priority() const {
	return -1;
}

// Drawn only while the simulator or one of its collisions is selected, as KawaiiPhysics draws only in the edit mode of its node selected
// in the anim graph (FAnimNodeEditMode). Every simulator drawn at once buries the one being set up.
static bool is_simulator_selected(BoneClothSimulator3D *p_simulator) {
	const TypedArray<Node> nodes = EditorInterface::get_singleton()->get_selection()->get_selected_nodes();
	for (int64_t i = 0; i < nodes.size(); i++) {
		Node *node = Object::cast_to<Node>(nodes[i]);
		if (node == p_simulator || (Object::cast_to<BoneClothCollision3D>(node) && node->get_parent() == p_simulator)) {
			return true;
		}
	}
	return false;
}

// Each joint where its bone is drawn this frame, a line to its parent and a sphere of the radius it collides with. The root follows
// the pose and collides with nothing, so it has no sphere, as in the engine's. A line between the two joints of each link.
// With a limit angle and Show Limit Angle checked in the simulator's menu, a cone at each parent around the bone's pose direction, drawn
// as SpotLight3D's: a circle, lines out to it and the axis. A joint whose bone lies along the cone's side is at its limit; a joint whose
// curve brings the limit to 0 is held on the axis. Off by default, as Magica Cloth 2 draws no cone; KawaiiPhysics has a switch for its
// cones alone (bEnableDebugDrawLimitAngle).
// Half the bone long with four lines: as long as the bone with eight, the cones of a six-chain skirt at 75 degrees covered each other
// and hid the skirt's shape.
void BoneClothSimulator3DGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo) {
	static constexpr float CONE_SCALE = 0.5f;
	static constexpr int CONE_LINE_COUNT = 4;

	p_gizmo->clear();
	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(p_gizmo->get_node_3d());
	Skeleton3D *skeleton = simulator->get_skeleton();

	// A changed setting leaves the joints and links stale, chain indices among them, until the next frame builds them again.
	if (!skeleton || simulator->joints_dirty || !is_simulator_selected(simulator)) {
		return;
	}

	// The menu's check, kept in the project's editor metadata (BoneClothSimulator3DMenu).
	const bool show_limit_angle = EditorInterface::get_singleton()->get_editor_settings()->get_project_metadata("bone_cloth", "show_limit_angle", false);

	PackedVector3Array lines;

	for (const BoneClothSimulator3D::Chain &chain : simulator->chains) {
		for (uint32_t i = 1; i < chain.joints.size(); i++) {
			const BoneClothSimulator3D::Joint &joint = chain.joints[i];
			const BoneClothSimulator3D::Joint &parent = chain.joints[i - 1];
			lines.push_back(parent.display_location);
			lines.push_back(joint.display_location);
			add_sphere(lines, joint.display_location, joint.radius);

			const Vector3 pose_vector = joint.current_pose_location - parent.current_pose_location;
			const float length = pose_vector.length();
			if (!show_limit_angle || simulator->limit_angle <= 0.0f || length < CMP_EPSILON) {
				continue;
			}

			const Vector3 apex = parent.display_location;
			const Vector3 direction = pose_vector / length;
			const float cone_length = length * CONE_SCALE;
			lines.push_back(apex);
			lines.push_back(apex + direction * cone_length);

			const float circle_radius = cone_length * Math::sin(joint.limit_angle);
			if (circle_radius < CMP_EPSILON) {
				continue;
			}

			const Vector3 circle_center = apex + direction * (cone_length * Math::cos(joint.limit_angle));
			const Vector3 axis_x = direction.get_any_perpendicular();
			const Vector3 axis_y = direction.cross(axis_x);
			add_arc(lines, circle_center, axis_x, axis_y, circle_radius, 0.0f, Math::TAU);
			for (int k = 0; k < CONE_LINE_COUNT; k++) {
				const float angle = (float)Math::TAU * k / CONE_LINE_COUNT;
				lines.push_back(apex);
				lines.push_back(circle_center + (axis_x * Math::cos(angle) + axis_y * Math::sin(angle)) * circle_radius);
			}
		}
	}

	for (const BoneClothSimulator3D::Link &link : simulator->links) {
		lines.push_back(simulator->chains[link.chain_a].joints[link.depth].display_location);
		lines.push_back(simulator->chains[link.chain_b].joints[link.depth].display_location);
	}

	if (lines.is_empty()) {
		return;
	}

	// The solver works in skeleton space, the lines are in the node's.
	const Transform3D to_node = simulator->get_global_transform().affine_inverse() * skeleton->get_global_transform();
	Vector3 *points = lines.ptrw();
	for (int64_t i = 0; i < lines.size(); i++) {
		points[i] = to_node.xform(points[i]);
	}
	p_gizmo->add_lines(lines, get_material("joint", p_gizmo));
	p_gizmo->add_collision_segments(lines);
}

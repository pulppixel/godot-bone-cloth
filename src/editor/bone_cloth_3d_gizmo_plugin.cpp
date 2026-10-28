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

// 호 그리기. p_center 중심, p_from ~ p_to 각도(0: p_axis_x 쪽, 1/4바퀴: p_axis_y 쪽).
// 1/16바퀴마다 점을 찍고, 점 개수를 짝수로 맞춘다(호 가운데에 점이 와야 캡슐 끝이 꼭대기까지 그려진다).
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

// 구 그리기. p_center 중심으로 XY, YZ, ZX 평면에 원을 하나씩 그린다.
static void add_sphere(PackedVector3Array &r_lines, const Vector3 &p_center, float p_radius) {
	add_arc(r_lines, p_center, Vector3(1, 0, 0), Vector3(0, 1, 0), p_radius, 0.0f, Math::TAU);
	add_arc(r_lines, p_center, Vector3(0, 1, 0), Vector3(0, 0, 1), p_radius, 0.0f, Math::TAU);
	add_arc(r_lines, p_center, Vector3(0, 0, 1), Vector3(1, 0, 0), p_radius, 0.0f, Math::TAU);
}

// BoneClothCollision3D

void BoneClothCollision3DGizmoPlugin::_bind_methods() {
}

// 에디터 설정의 spring bone 색을 사용한다(사용자가 색을 바꾸면 여기도 바뀐다).
// 선택된 gizmo는 선을 메시 앞에, 나머지는 흐리게 메시 뒤에 그린다(GDExtension은 is_selected()를 못 써서 get_material()로 구분한다).
// mirror 색은 초록으로 고정.
BoneClothCollision3DGizmoPlugin::BoneClothCollision3DGizmoPlugin() {
	Ref<EditorSettings> settings = EditorInterface::get_singleton()->get_editor_settings();
	create_material("collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_collision"), false, true);
	create_material("inside_collision", settings->get_setting("editors/3d_gizmos/gizmo_colors/spring_bone_inside_collision"), false, true);
	create_material("mirror_collision", Color(0.5, 1.0, 0.5), false, true);
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

// handle 이름(마우스를 올리면 보인다).
// tapered capsule은 두 반지름에 handle이 하나씩 있다.
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

// gizmo를 끌기 시작할 때의 크기.
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

// 마우스가 가리키는 곳이 p_from에서 p_direction 쪽으로 얼마나 떨어져 있는지(끌기 시작할 때의 노드 기준).
float BoneClothCollision3DGizmoPlugin::_get_distance_along(const Vector3 &p_from, const Vector3 &p_direction, Camera3D *p_camera, const Vector2 &p_screen_pos) const {
	const Transform3D to_node = initial_transform.affine_inverse();
	const Vector3 ray_from = to_node.xform(p_camera->project_ray_origin(p_screen_pos));
	const Vector3 ray_to = to_node.xform(p_camera->project_ray_origin(p_screen_pos) + p_camera->project_ray_normal(p_screen_pos) * 4096.0f);
	const PackedVector3Array closest = Geometry3D::get_singleton()->get_closest_points_between_segments(p_from - p_direction * 4096.0f, p_from + p_direction * 4096.0f, ray_from, ray_to);
	return p_direction.dot(closest[0] - p_from);
}

// handle을 끌고 드래그하는 동안, 크기를 바꿔준다.
// height handle: 한쪽을 끄는 동안, 반대쪽 끝은 가만히 둔다. Alt를 누르면 양쪽 끝이 같이 움직인다.
// 그리드 스냅은 하지 않는다 (GDExtension은 스냅 설정을 읽지 못하기 때문).
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

	// height를 먼저 넣는다. 그래야 다음에 넣는 반지름이 height의 절반을 넘지 않아서, setter가 다른 값을 고치지 않는다.
	if (capsule) {
		capsule->set_height(height);
		capsule->set_radius(top_radius);
	} else {
		tapered_capsule->set_height(height);
		tapered_capsule->set_top_radius(top_radius);
		tapered_capsule->set_bottom_radius(bottom_radius);
	}
}

// 끌기를 마치면 크기와 노드 위치를 undo 한 번으로 묶는다. Esc면 끌기 시작 상태로 돌린다.
// setter가 반지름을 height 절반 이하로 맞추므로, 세 값을 어떤 순서로 넣어도 기록한 크기가 된다.
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

// collision 모양을 선으로 그린다.
// handle은 선택된 노드에만 보인다.
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

	// mirror가 있으면 mirror bone 쪽에도 초록으로 같은 모양을 그린다.
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

// tapered capsule 그리기(두 반지름이 같으면 캡슐).
// XY, ZY 평면의 윤곽선, 양 끝 원, 옆선 네 개를 그린다.
void BoneClothCollision3DGizmoPlugin::_add_tapered_capsule(PackedVector3Array &r_lines, float p_top_radius, float p_bottom_radius, float p_height) {
	const Vector3 up(0, 1, 0);
	const Vector3 top_center = up * (p_height * 0.5f - p_top_radius);
	const Vector3 bottom_center = up * (p_bottom_radius - p_height * 0.5f);

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

// 평면 그리기: 법선(노드 Y) 방향 화살표 두 개를 십자로 겹치고, 둘레에 1m 점선 사각형을 그린다.
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

// joint 색. 에디터 설정의 spring bone joint 색을 쓴다(선택 표시는 collision gizmo와 같다).
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

// simulator나 그 자식 collision이 선택됐을 때만 그린다(모든 simulator를 그리면 작업 중인 것이 안 보인다).
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

// simulator gizmo 그리기.
// 각도 제한 원뿔은 길이가 bone의 절반, 선이 4개다(길면 치마의 원뿔끼리 겹쳐서 치마 모양이 안 보인다).
void BoneClothSimulator3DGizmoPlugin::_redraw(const Ref<EditorNode3DGizmo> &p_gizmo) {
	static constexpr float CONE_SCALE = 0.5f;
	static constexpr int CONE_LINE_COUNT = 4;

	p_gizmo->clear();
	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(p_gizmo->get_node_3d());
	Skeleton3D *skeleton = simulator->get_skeleton();

	// 설정이 바뀐 직후(joints_dirty)에는 그리지 않는다.
	// joint와 link가 옛 값이라 없는 chain 번호를 읽을 수 있다 (다음 프레임에 다시 만들어진다).
	if (!skeleton || simulator->joints_dirty || !is_simulator_selected(simulator)) {
		return;
	}

	// Show Radius, Show Limit Angle 메뉴의 체크 상태(프로젝트의 에디터 metadata에 저장된다).
	const bool show_radius = EditorInterface::get_singleton()->get_editor_settings()->get_project_metadata("bone_cloth", "show_radius", true);
	const bool show_limit_angle = EditorInterface::get_singleton()->get_editor_settings()->get_project_metadata("bone_cloth", "show_limit_angle", false);

	PackedVector3Array lines;

	for (const BoneClothSimulator3D::Chain &chain : simulator->chains) {
		for (uint32_t i = 1; i < chain.joints.size(); i++) {
			const BoneClothSimulator3D::Joint &joint = chain.joints[i];
			const BoneClothSimulator3D::Joint &parent = chain.joints[i - 1];
			lines.push_back(parent.display_location);
			lines.push_back(joint.display_location);
			if (show_radius) {
				add_sphere(lines, joint.display_location, joint.radius);
			}

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

	// 선 좌표를 skeleton 공간에서 노드 공간으로 바꾼다(simulator는 skeleton 공간으로 계산한다).
	const Transform3D to_node = simulator->get_global_transform().affine_inverse() * skeleton->get_global_transform();
	Vector3 *points = lines.ptrw();
	for (int64_t i = 0; i < lines.size(); i++) {
		points[i] = to_node.xform(points[i]);
	}
	p_gizmo->add_lines(lines, get_material("joint", p_gizmo));
	p_gizmo->add_collision_segments(lines);
}

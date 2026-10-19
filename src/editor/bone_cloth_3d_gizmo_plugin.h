// bone cloth 노드를 3D 화면에 그리는 gizmo(에디터 전용).

#pragma once

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>

using namespace godot;

// collision gizmo.
class BoneClothCollision3DGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(BoneClothCollision3DGizmoPlugin, EditorNode3DGizmoPlugin)

	// handle을 끌기 시작할 때의 크기와 노드 위치.
	// 여기서부터 측정을 시작하고, Esc를 누르면 이 값으로 되돌린다.
	Variant initial_value;
	Transform3D initial_transform;

	static void _add_tapered_capsule(PackedVector3Array &r_lines, float p_top_radius, float p_bottom_radius, float p_height);
	static void _add_plane(PackedVector3Array &r_lines);
	float _get_distance_along(const Vector3 &p_from, const Vector3 &p_direction, Camera3D *p_camera, const Vector2 &p_screen_pos) const;

protected:
	static void _bind_methods();

public:
	bool _has_gizmo(Node3D *p_for_node_3d) const override;
	String _get_gizmo_name() const override;
	int32_t _get_priority() const override;
	void _redraw(const Ref<EditorNode3DGizmo> &p_gizmo) override;

	String _get_handle_name(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) const override;
	Variant _get_handle_value(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) const override;
	void _begin_handle_action(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary) override;
	void _set_handle(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary, Camera3D *p_camera, const Vector2 &p_screen_pos) override;
	void _commit_handle(const Ref<EditorNode3DGizmo> &p_gizmo, int32_t p_handle_id, bool p_secondary, const Variant &p_restore, bool p_cancel) override;

	BoneClothCollision3DGizmoPlugin();
};

// simulator gizmo.
class BoneClothSimulator3DGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(BoneClothSimulator3DGizmoPlugin, EditorNode3DGizmoPlugin)

protected:
	static void _bind_methods();

public:
	bool _has_gizmo(Node3D *p_for_node_3d) const override;
	String _get_gizmo_name() const override;
	int32_t _get_priority() const override;
	void _redraw(const Ref<EditorNode3DGizmo> &p_gizmo) override;

	BoneClothSimulator3DGizmoPlugin();
};

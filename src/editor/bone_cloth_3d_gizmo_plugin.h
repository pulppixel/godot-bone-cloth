#pragma once

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>

using namespace godot;

enum BoneClothShow {
	SHOW_RADIUS,
	SHOW_LIMIT_ANGLE,
	SHOW_MAX_DISTANCE,
	SHOW_BACKSTOP,
	SHOW_COUNT,
};

struct BoneClothShowOption {
	const char *label;
	const char *key;
	bool shown;
};

static constexpr BoneClothShowOption BONE_CLOTH_SHOW[SHOW_COUNT] = {
	{ "Show Radius", "show_radius", true },
	{ "Show Limit Angle", "show_limit_angle", false },
	{ "Show Max Distance", "show_max_distance", false },
	{ "Show Backstop", "show_backstop", false },
};

bool is_bone_cloth_shown(BoneClothShow p_show);

class BoneClothCollision3DGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(BoneClothCollision3DGizmoPlugin, EditorNode3DGizmoPlugin)

	Variant initial_value;
	Transform3D initial_transform;
	Vector3 initial_position_offset;
	Quaternion initial_rotation_offset;

	float _get_distance_along(const Vector3 &p_from, const Vector3 &p_direction, Camera3D *p_camera, const Vector2 &p_screen_pos) const;
	float _drag_face(Node3D *p_node, const Vector3 &p_direction, float p_initial_size, Camera3D *p_camera, const Vector2 &p_screen_pos) const;

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

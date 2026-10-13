// The bone cloth nodes drawn in the 3D viewport, as editor/scene/3d/gizmos/spring_bone_3d_gizmo_plugin.cpp draws the engine's
// SpringBone nodes. Editor only, registered at the EDITOR level and added by BoneClothSimulator3DEditorPlugin.

#pragma once

#include <godot_cpp/classes/editor_node3d_gizmo.hpp>
#include <godot_cpp/classes/editor_node3d_gizmo_plugin.hpp>

using namespace godot;

// Each collision shape as wire lines in the node's space, in the editor's spring bone collision colours.
class BoneClothCollision3DGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(BoneClothCollision3DGizmoPlugin, EditorNode3DGizmoPlugin)

	static void _add_tapered_capsule(PackedVector3Array &r_lines, float p_top_radius, float p_bottom_radius, float p_height);
	static void _add_plane(PackedVector3Array &r_lines);

protected:
	static void _bind_methods();

public:
	bool _has_gizmo(Node3D *p_for_node_3d) const override;
	String _get_gizmo_name() const override;
	int32_t _get_priority() const override;
	void _redraw(const Ref<EditorNode3DGizmo> &p_gizmo) override;

	BoneClothCollision3DGizmoPlugin();
};

// Each chain as the solver holds it this frame: the joints, the sphere each collides with, the links and the limit angle's cones.
// Redrawn every frame from the simulator's state, as KawaiiPhysics' edit mode and Magica Cloth 2's gizmo draw theirs. The engine's
// SpringBoneSimulator3D skins its lines to the bones instead; a cone skinned to its bone would turn with the cloth, and its axis is
// the pose direction.
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

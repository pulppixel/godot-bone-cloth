// The editor side of BoneClothSimulator3D: a menu named after the node in the 3D viewport's toolbar, shown while one is
// selected, the place of the Skeleton3D and Mesh menus (editor/scene/3d/skeleton_3d_editor_plugin.cpp,
// mesh_instance_3d_editor_plugin.cpp), and the gizmos of the bone cloth nodes. Editor only, registered at the EDITOR level.

#pragma once

#include "bone_cloth_simulator_3d.h"

#include "editor/bone_cloth_3d_gizmo_plugin.h"

#include <godot_cpp/classes/editor_file_dialog.hpp>
#include <godot_cpp/classes/editor_file_system_directory.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/menu_button.hpp>
#include <godot_cpp/classes/popup_menu.hpp>

using namespace godot;

// Presets, a submenu listing every BoneClothPreset file in the project, addons included, each time it opens; a pick
// applies it as one undo step. Save Preset... writes the node's values to a new preset file.
class BoneClothSimulator3DMenu : public MenuButton {
	GDCLASS(BoneClothSimulator3DMenu, MenuButton)

	uint64_t simulator_id = 0;
	PopupMenu *presets_menu = nullptr;
	PackedStringArray preset_paths;
	EditorFileDialog *save_dialog = nullptr;

	void _find_presets(EditorFileSystemDirectory *p_directory);
	void _on_presets_about_to_popup();
	void _on_preset_pressed(int p_id);
	void _on_option_pressed(int p_id);
	void _on_save_path_selected(const String &p_path);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void set_simulator(BoneClothSimulator3D *p_simulator);

	BoneClothSimulator3DMenu();
};

class BoneClothSimulator3DEditorPlugin : public EditorPlugin {
	GDCLASS(BoneClothSimulator3DEditorPlugin, EditorPlugin)

	BoneClothSimulator3DMenu *menu = nullptr;
	Ref<BoneClothCollision3DGizmoPlugin> collision_gizmo_plugin;
	Ref<BoneClothSimulator3DGizmoPlugin> simulator_gizmo_plugin;

protected:
	static void _bind_methods();

public:
	void _enter_tree() override;
	void _exit_tree() override;
	bool _handles(Object *p_object) const override;
	void _edit(Object *p_object) override;
	void _make_visible(bool p_visible) override;
};

// The editor side of BoneClothSimulator3D: a Presets menu at the top of its inspector, the form of the Curve inspector's
// Presets menu (editor/scene/curve_editor_plugin.cpp). Editor only, registered at the EDITOR level.

#pragma once

#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/editor_file_dialog.hpp>
#include <godot_cpp/classes/editor_file_system_directory.hpp>
#include <godot_cpp/classes/editor_inspector_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/menu_button.hpp>


namespace godot {

	// Lists every BoneClothPreset file in the project, addons included, each time it opens; a pick applies it as one undo step.
	// The last item saves the node's values as a new preset file.
	class BoneClothPresetMenu : public MenuButton {
		GDCLASS(BoneClothPresetMenu, MenuButton)

		uint64_t simulator_id = 0;
		PackedStringArray preset_paths;
		EditorFileDialog* save_dialog = nullptr;

		void _find_presets(EditorFileSystemDirectory* p_directory);
		void _on_about_to_popup();
		void _on_id_pressed(int p_id);
		void _on_save_path_selected(const String& p_path);

	protected:
		static void _bind_methods();

	public:
		void set_simulator(BoneClothSimulator3D* p_simulator);

		BoneClothPresetMenu();
	};

	class EditorInspectorPluginBoneClothSimulator3D : public EditorInspectorPlugin {
		GDCLASS(EditorInspectorPluginBoneClothSimulator3D, EditorInspectorPlugin)

	protected:
		static void _bind_methods();

	public:
		bool _can_handle(Object* p_object) const override;
		void _parse_category(Object* p_object, const String& p_category) override;
	};

	class BoneClothSimulator3DEditorPlugin : public EditorPlugin {
		GDCLASS(BoneClothSimulator3DEditorPlugin, EditorPlugin)

			Ref<EditorInspectorPluginBoneClothSimulator3D> inspector_plugin;

	protected:
		static void _bind_methods();

	public:
		void _enter_tree() override;
		void _exit_tree() override;
	};

}

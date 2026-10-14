#include "editor/bone_cloth_simulator_3d_editor_plugin.h"

#include <godot_cpp/classes/editor_file_system.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/resource_saver.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

enum {
	OPTION_PRESETS,
	OPTION_SAVE_PRESET,
	OPTION_SHOW_LIMIT_ANGLE,
};

void BoneClothSimulator3DMenu::_bind_methods() {
}

// The node's icon on the button, as the Mesh menu sets its own when the theme changes.
// An extension's icon is not in the editor theme: it is loaded from the path in bone_cloth.gdextension, and only once the file is imported,
// as the editor loads it for the scene tree.
// The plugin starts before the editor's first import, so the icon is also looked for each time the menu shows.
void BoneClothSimulator3DMenu::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_THEME_CHANGED:
		case NOTIFICATION_VISIBILITY_CHANGED: {
			const String icon_path = "res://addons/bone_cloth/icons/BoneClothSimulator3D.svg";
			if (is_visible() && ResourceLoader::get_singleton()->exists(icon_path)) {
				set_button_icon(ResourceLoader::get_singleton()->load(icon_path));
			}
		} break;
	}
}

// The Mesh and Skeleton3D menus' button: text, opens on hover, flat look without the icon tint
// (mesh_instance_3d_editor_plugin.cpp:797-802).
BoneClothSimulator3DMenu::BoneClothSimulator3DMenu() {
	set_text("BoneClothSimulator3D");
	set_switch_on_hover(true);
	set_flat(false);
	set_theme_type_variation("FlatMenuButtonNoIconTint");

	presets_menu = memnew(PopupMenu);
	presets_menu->connect("about_to_popup", callable_mp(this, &BoneClothSimulator3DMenu::_on_presets_about_to_popup));
	presets_menu->connect("id_pressed", callable_mp(this, &BoneClothSimulator3DMenu::_on_preset_pressed));
	PopupMenu *options = get_popup();
	options->add_submenu_node_item("Presets", presets_menu, OPTION_PRESETS);
	options->add_item("Save Preset...", OPTION_SAVE_PRESET);
	options->add_separator();
	options->add_check_item("Show Limit Angle", OPTION_SHOW_LIMIT_ANGLE);
	options->set_item_checked(options->get_item_index(OPTION_SHOW_LIMIT_ANGLE), EditorInterface::get_singleton()->get_editor_settings()->get_project_metadata("bone_cloth", "show_limit_angle", false));
	options->connect("id_pressed", callable_mp(this, &BoneClothSimulator3DMenu::_on_option_pressed));

	save_dialog = memnew(EditorFileDialog);
	save_dialog->set_file_mode(FileDialog::FILE_MODE_SAVE_FILE);
	save_dialog->set_access(FileDialog::ACCESS_RESOURCES);
	save_dialog->add_filter("*.tres", "BoneClothPreset");
	save_dialog->set_title("Save Preset");
	save_dialog->connect("file_selected", callable_mp(this, &BoneClothSimulator3DMenu::_on_save_path_selected));
	add_child(save_dialog);
}

// 0 when no simulator is being edited.
void BoneClothSimulator3DMenu::set_simulator(BoneClothSimulator3D *p_simulator) {
	simulator_id = p_simulator ? p_simulator->get_instance_id() : 0;
}

// By the type the editor's file system recorded for each file, as Quick Load reads it, but into the addons folder too.
void BoneClothSimulator3DMenu::_find_presets(EditorFileSystemDirectory *p_directory) {
	for (int i = 0; i < p_directory->get_subdir_count(); i++) {
		_find_presets(p_directory->get_subdir(i));
	}
	for (int i = 0; i < p_directory->get_file_count(); i++) {
		if (p_directory->get_file_type(i) == StringName("BoneClothPreset")) {
			preset_paths.push_back(p_directory->get_file_path(i));
		}
	}
}

// Built each time the submenu opens, so a preset saved or copied in a moment ago is there.
void BoneClothSimulator3DMenu::_on_presets_about_to_popup() {
	presets_menu->clear();
	preset_paths.clear();
	_find_presets(EditorInterface::get_singleton()->get_resource_filesystem()->get_filesystem());
	preset_paths.sort();

	for (int i = 0; i < preset_paths.size(); i++) {
		presets_menu->add_item(preset_paths[i].get_file().get_basename().capitalize(), i);
		presets_menu->set_item_tooltip(i, preset_paths[i]);
	}
	if (preset_paths.is_empty()) {
		presets_menu->add_item("No presets found");
		presets_menu->set_item_disabled(0, true);
	}
}

void BoneClothSimulator3DMenu::_on_preset_pressed(int p_id) {
	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);
	ERR_FAIL_INDEX(p_id, preset_paths.size());
	const Ref<BoneClothPreset> preset = ResourceLoader::get_singleton()->load(preset_paths[p_id]);
	ERR_FAIL_COND_MSG(preset.is_null(), "Not a BoneClothPreset: '" + preset_paths[p_id] + "'.");

	// The values one by one, so undo puts back each value and curve the node had (apply_preset sets the same).
	EditorUndoRedoManager *undo_redo = EditorInterface::get_singleton()->get_editor_undo_redo();
	undo_redo->create_action("Apply Bone Cloth Preset", UndoRedo::MERGE_DISABLE, simulator);
	for (const char *name : BoneClothPreset::VALUES) {
		undo_redo->add_do_property(simulator, name, BoneClothPreset::copy_value(preset->get(name)));
		undo_redo->add_undo_property(simulator, name, simulator->get(name));
	}
	undo_redo->commit_action();
}

void BoneClothSimulator3DMenu::_on_option_pressed(int p_id) {
	// The gizmo's limit angle cones, for every simulator, kept per project as the editor keeps its own view choices.
	// The selected simulator redraws every frame and shows the change at once.
	if (p_id == OPTION_SHOW_LIMIT_ANGLE) {
		PopupMenu *options = get_popup();
		const int index = options->get_item_index(OPTION_SHOW_LIMIT_ANGLE);
		const bool show = !options->is_item_checked(index);
		options->set_item_checked(index, show);
		EditorInterface::get_singleton()->get_editor_settings()->set_project_metadata("bone_cloth", "show_limit_angle", show);
		return;
	}

	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);

	if (p_id == OPTION_SAVE_PRESET) {
		save_dialog->set_current_file(String(simulator->get_name()).to_snake_case() + ".tres");
		save_dialog->popup_file_dialog();
	}
}

// Always a new file, as KawaiiPhysics' Export Preset and Magica Cloth's Save: the node never writes into a preset it came from.
void BoneClothSimulator3DMenu::_on_save_path_selected(const String &p_path) {
	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);

	const Error error = ResourceSaver::get_singleton()->save(simulator->make_preset(), p_path);
	ERR_FAIL_COND_MSG(error != OK, "Could not save the preset to '" + p_path + "'.");
	EditorInterface::get_singleton()->get_resource_filesystem()->update_file(p_path);
}

void BoneClothSimulator3DEditorPlugin::_bind_methods() {
}

void BoneClothSimulator3DEditorPlugin::_enter_tree() {
	menu = memnew(BoneClothSimulator3DMenu);
	menu->hide();
	add_control_to_container(CONTAINER_SPATIAL_EDITOR_MENU, menu);

	collision_gizmo_plugin.instantiate();
	add_node_3d_gizmo_plugin(collision_gizmo_plugin);
	simulator_gizmo_plugin.instantiate();
	add_node_3d_gizmo_plugin(simulator_gizmo_plugin);
}

void BoneClothSimulator3DEditorPlugin::_exit_tree() {
	remove_control_from_container(CONTAINER_SPATIAL_EDITOR_MENU, menu);
	menu->queue_free();
	menu = nullptr;

	remove_node_3d_gizmo_plugin(collision_gizmo_plugin);
	collision_gizmo_plugin.unref();
	remove_node_3d_gizmo_plugin(simulator_gizmo_plugin);
	simulator_gizmo_plugin.unref();
}

bool BoneClothSimulator3DEditorPlugin::_handles(Object *p_object) const {
	return Object::cast_to<BoneClothSimulator3D>(p_object) != nullptr;
}

void BoneClothSimulator3DEditorPlugin::_edit(Object *p_object) {
	menu->set_simulator(Object::cast_to<BoneClothSimulator3D>(p_object));
}

// Shown while a simulator is selected, as MeshInstance3DEditorPlugin::make_visible does with its Mesh menu.
void BoneClothSimulator3DEditorPlugin::_make_visible(bool p_visible) {
	menu->set_visible(p_visible);
	if (!p_visible) {
		menu->set_simulator(nullptr);
	}
}

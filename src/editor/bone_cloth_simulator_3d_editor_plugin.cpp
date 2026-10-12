#include "editor/bone_cloth_simulator_3d_editor_plugin.h"

#include <godot_cpp/classes/editor_file_system.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/popup_menu.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/resource_saver.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/core/class_db.hpp>


using namespace godot;

// Item ids: a preset's id is its index in preset_paths; these two sit far past any list.
static constexpr int SAVE_ID = 1 << 20;
static constexpr int NONE_ID = SAVE_ID + 1;

void BoneClothPresetMenu::_bind_methods()
{
}

BoneClothPresetMenu::BoneClothPresetMenu()
{
	set_text("Presets");
	set_flat(false);
	connect("about_to_popup", callable_mp(this, &BoneClothPresetMenu::_on_about_to_popup));
	get_popup()->connect("id_pressed", callable_mp(this, &BoneClothPresetMenu::_on_id_pressed));

	save_dialog = memnew(EditorFileDialog);
	save_dialog->set_file_mode(FileDialog::FILE_MODE_SAVE_FILE);
	save_dialog->set_access(FileDialog::ACCESS_RESOURCES);
	save_dialog->add_filter("*.tres", "BoneClothPreset");
	save_dialog->set_title("Save Preset");
	save_dialog->connect("file_selected", callable_mp(this, &BoneClothPresetMenu::_on_save_path_selected));
	add_child(save_dialog);
}

void BoneClothPresetMenu::set_simulator(BoneClothSimulator3D* p_simulator)
{
	simulator_id = p_simulator->get_instance_id();
}

// By the type the editor's file system recorded for each file, as Quick Load reads it, but into the addons folder too
void BoneClothPresetMenu::_find_presets(EditorFileSystemDirectory* p_directory)
{
	for (int i = 0; i < p_directory->get_subdir_count(); i++) {
		_find_presets(p_directory->get_subdir(i));
	}
	for (int i = 0; i < p_directory->get_file_count(); i++) {
		if (p_directory->get_file_type(i) == StringName("BoneClothPreset")) {
			preset_paths.push_back(p_directory->get_file_path(i));
		}
	}
}

// Built each time the menu opens, so a preset saved or copied in a moment ago is there.
void BoneClothPresetMenu::_on_about_to_popup()
{
	PopupMenu* popup = get_popup();
	popup->clear();
	preset_paths.clear();
	_find_presets(EditorInterface::get_singleton()->get_resource_filesystem()->get_filesystem());
	preset_paths.sort();

	for (int i = 0; i < preset_paths.size(); i++) {
		popup->add_item(preset_paths[i].get_file().get_basename().capitalize(), i);
		popup->set_item_tooltip(i, preset_paths[i]);
	}
	if (preset_paths.is_empty()) {
		popup->add_item("No presets found", NONE_ID);
		popup->set_item_disabled(0, true);
	}

	popup->add_separator();
	const Ref<Theme> theme = EditorInterface::get_singleton()->get_editor_theme();
	popup->add_icon_item(theme->get_icon("Save", "EditorIcons"), "Save Preset...", SAVE_ID);
}

void BoneClothPresetMenu::_on_id_pressed(int p_id)
{
	BoneClothSimulator3D* simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);

	if (p_id == SAVE_ID) {
		save_dialog->set_current_file(String(simulator->get_name()).to_snake_case() + ".tres");
		save_dialog->popup_file_dialog();
		return;
	}

	ERR_FAIL_INDEX(p_id, preset_paths.size());
	const Ref<BoneClothPreset> preset = ResourceLoader::get_singleton()->load(preset_paths[p_id]);
	ERR_FAIL_COND_MSG(preset.is_null(), "Not a BoneClothPreset: '" + preset_paths[p_id] + "'.");

	// The values one by one, so undo puts back each value and curve the node had (apply_preset sets the same).
	EditorUndoRedoManager* undo_redo = EditorInterface::get_singleton()->get_editor_undo_redo();
	undo_redo->create_action("Apply Bone Cloth Preset", UndoRedo::MERGE_DISABLE, simulator);
	for (const char* name : BoneClothPreset::VALUES) {
		undo_redo->add_do_property(simulator, name, BoneClothPreset::copy_value(preset->get(name)));
		undo_redo->add_undo_property(simulator, name, simulator->get(name));
	}
	undo_redo->commit_action();
}

// Always a new file, as KawaiiPhysics' Export Preset and Magica Cloth's Save: the node never writes into a preset it came from.
void BoneClothPresetMenu::_on_save_path_selected(const String& p_path)
{
	BoneClothSimulator3D* simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);

	const Error error = ResourceSaver::get_singleton()->save(simulator->make_preset(), p_path);
	ERR_FAIL_COND_MSG(error != OK, "Could not save the preset to '" + p_path + "'.");
	EditorInterface::get_singleton()->get_resource_filesystem()->update_file(p_path);
}

void EditorInspectorPluginBoneClothSimulator3D::_bind_methods()
{
}

bool EditorInspectorPluginBoneClothSimulator3D::_can_handle(Object* p_object) const
{
	return Object::cast_to<BoneClothSimulator3D>(p_object) != nullptr;
}

// Under the class's own heading, above its first value: both reference plugins put presets first.
void EditorInspectorPluginBoneClothSimulator3D::_parse_category(Object* p_object, const String& p_category)
{
	if (p_category != "BoneClothSimulator3D") {
		return;
	}
	BoneClothPresetMenu* menu = memnew(BoneClothPresetMenu);
	menu->set_simulator(Object::cast_to<BoneClothSimulator3D>(p_object));
	add_custom_control(menu);
}

void BoneClothSimulator3DEditorPlugin::_bind_methods()
{
}

void BoneClothSimulator3DEditorPlugin::_enter_tree()
{
	inspector_plugin.instantiate();
	add_inspector_plugin(inspector_plugin);
}

void BoneClothSimulator3DEditorPlugin::_exit_tree()
{
	remove_inspector_plugin(inspector_plugin);
	inspector_plugin.unref();
}

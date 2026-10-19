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

// 메뉴 버튼에 노드 아이콘을 넣는다(theme가 바뀌면 다시).
// GDExtension 아이콘은 에디터 theme에 없어서 파일(bone_cloth.gdextension에 적힌 경로)에서 읽는다.
// 플러그인이 첫 import보다 먼저 시작하므로, 메뉴가 보일 때마다 다시 찾는다.
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

// 메뉴 버튼 모양. Mesh, Skeleton3D 메뉴와 같게 한다(글자, 마우스를 올리면 열림, 아이콘 색 그대로).
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

// 메뉴가 다룰 simulator(노드 번호). 없으면 0.
void BoneClothSimulator3DMenu::set_simulator(BoneClothSimulator3D *p_simulator) {
	simulator_id = p_simulator ? p_simulator->get_instance_id() : 0;
}

// 폴더를 모두 돌며 프리셋 파일을 찾는다(addons 포함). 에디터가 기록한 파일 타입으로 구분한다.
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

// Presets 메뉴가 열리기 직전에 목록을 새로 만든다(방금 저장하거나 복사한 프리셋도 보이게).
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

	// 프리셋 값을 하나씩 넣는다(undo하면 값과 curve가 하나하나 원래대로 돌아간다). apply_preset과 같은 값이다.
	EditorUndoRedoManager *undo_redo = EditorInterface::get_singleton()->get_editor_undo_redo();
	undo_redo->create_action("Apply Bone Cloth Preset", UndoRedo::MERGE_DISABLE, simulator);
	for (const char *name : BoneClothPreset::VALUES) {
		undo_redo->add_do_property(simulator, name, BoneClothPreset::copy_value(preset->get(name)));
		undo_redo->add_undo_property(simulator, name, simulator->get(name));
	}
	undo_redo->commit_action();
}

void BoneClothSimulator3DMenu::_on_option_pressed(int p_id) {
	// Show Limit Angle 켜고 끄기. 모든 simulator에 적용하고 프로젝트마다 저장한다.
	// 선택된 simulator는 매 프레임 다시 그리므로 바로 보인다.
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

// 항상 새 파일로 저장한다(노드가 불러온 프리셋 파일을 덮어쓰지 않게).
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

// simulator를 선택했을 때만 메뉴를 보인다.
void BoneClothSimulator3DEditorPlugin::_make_visible(bool p_visible) {
	menu->set_visible(p_visible);
	if (!p_visible) {
		menu->set_simulator(nullptr);
	}
}

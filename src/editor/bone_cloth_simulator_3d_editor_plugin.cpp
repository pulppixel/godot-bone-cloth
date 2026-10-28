#include "editor/bone_cloth_simulator_3d_editor_plugin.h"

#include <godot_cpp/classes/editor_file_system.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/resource_saver.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/tree_item.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

enum {
	OPTION_ADD_CHAINS,
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
	options->add_item("Add Chains...", OPTION_ADD_CHAINS);
	options->add_separator();
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

	// 0: 체크 상자와 bone 이름, 1: 그 bone에서 만들 chain의 크기.
	chains_dialog = memnew(ConfirmationDialog);
	chains_dialog->set_title("Add Chains");
	chains_dialog->set_ok_button_text("Add");
	chains_dialog->connect("confirmed", callable_mp(this, &BoneClothSimulator3DMenu::_on_chains_confirmed));
	bones_tree = memnew(Tree);
	bones_tree->set_columns(2);
	bones_tree->set_column_expand(1, false);
	bones_tree->set_hide_root(true);
	bones_tree->connect("item_edited", callable_mp(this, &BoneClothSimulator3DMenu::_on_bone_edited));
	chains_dialog->add_child(bones_tree);
	add_child(chains_dialog);
}

static void collect_line(Skeleton3D *p_skeleton, int p_root, int p_next, LocalVector<Vector2i> &r_chains) {
	PackedInt32Array line;
	line.push_back(p_root);
	for (int bone = p_next; bone >= 0;) {
		line.push_back(bone);
		const PackedInt32Array children = p_skeleton->get_bone_children(bone);
		bone = children.is_empty() ? -1 : children[0];
	}
	r_chains.push_back(Vector2i(p_root, line[line.size() - 1]));

	for (int i = 1; i < line.size(); i++) {
		const PackedInt32Array children = p_skeleton->get_bone_children(line[i]);
		for (int k = 1; k < children.size(); k++) {
			collect_line(p_skeleton, line[i], children[k], r_chains);
		}
	}
}

/*
	체크한 bone 하나에서 만들 chain 목록.

	Hair1 - Hair2 - Hair3        체크: Hair1
			  |
			  +-- HairB1         chain 0: Hair1 ~ Hair3, chain 1: Hair2 ~ HairB1
*/
static void collect_chains(Skeleton3D *p_skeleton, int p_root, LocalVector<Vector2i> &r_chains) {
	const PackedInt32Array children = p_skeleton->get_bone_children(p_root);
	if (children.is_empty()) {
		r_chains.push_back(Vector2i(p_root, p_root));
		return;
	}
	for (int child : children) {
		collect_line(p_skeleton, p_root, child, r_chains);
	}
}

void BoneClothSimulator3DMenu::_fill_bones_tree(BoneClothSimulator3D *p_simulator) {
	bones_tree->clear();
	Skeleton3D *skeleton = p_simulator->get_skeleton();

	PackedInt32Array roots;
	PackedInt32Array locked;
	for (int i = 0; i < p_simulator->get_chain_count(); i++) {
		const int root = skeleton->find_bone(p_simulator->get_root_bone_name(i));
		const int end = skeleton->find_bone(p_simulator->get_end_bone_name(i));
		if (root >= 0) {
			roots.push_back(root);
		}
		for (int bone = end >= 0 ? end : root; bone >= 0; bone = skeleton->get_bone_parent(bone)) {
			locked.push_back(bone);
		}
	}

	TreeItem *tree_root = bones_tree->create_item();
	const PackedInt32Array top = skeleton->get_parentless_bones();
	for (int i = 0; i < top.size(); i++) {
		TreeItem *item = _add_bone_item(skeleton, top[i], tree_root, roots, locked);
		item->set_collapsed(i > 0);
	}
}

// bone 하나의 줄(예: "Hair1    4 bones, 0.330 m").
TreeItem *BoneClothSimulator3DMenu::_add_bone_item(Skeleton3D *p_skeleton, int p_bone, TreeItem *p_parent, const PackedInt32Array &p_roots, const PackedInt32Array &p_locked) {
	TreeItem *item = bones_tree->create_item(p_parent);
	item->set_cell_mode(0, TreeItem::CELL_MODE_CHECK);
	item->set_text(0, p_skeleton->get_bone_name(p_bone));
	item->set_icon(0, EditorInterface::get_singleton()->get_editor_theme()->get_icon("Bone", "EditorIcons"));
	item->set_metadata(0, p_bone);
	item->set_checked(0, p_roots.has(p_bone));
	item->set_editable(0, !p_locked.has(p_bone));
	item->set_collapsed(true);

	LocalVector<Vector2i> chains;
	collect_chains(p_skeleton, p_bone, chains);
	int bone_count = 1;
	float length = 0.0f;
	for (int bone = chains[0].y; bone != p_bone; bone = p_skeleton->get_bone_parent(bone)) {
		bone_count++;
		length += p_skeleton->get_bone_global_rest(bone).origin.distance_to(p_skeleton->get_bone_global_rest(p_skeleton->get_bone_parent(bone)).origin);
	}
	String size = vformat("%d bones, %.3f m", bone_count, length);
	if (chains.size() > 1) {
		size = vformat("%d chains, ", int(chains.size())) + size;
	}
	item->set_text(1, size);

	const PackedInt32Array children = p_skeleton->get_bone_children(p_bone);
	for (int child : children) {
		_add_bone_item(p_skeleton, child, item, p_roots, p_locked);
	}
	return item;
}

void BoneClothSimulator3DMenu::_set_covered(TreeItem *p_first, bool p_covered) {
	for (TreeItem *item = p_first; item; item = item->get_next()) {
		item->set_checked(0, false);
		item->set_editable(0, !p_covered);
		_set_covered(item->get_first_child(), p_covered);
	}
}

void BoneClothSimulator3DMenu::_on_bone_edited() {
	TreeItem *item = bones_tree->get_edited();
	_set_covered(item->get_first_child(), item->is_checked(0));
}

// 체크한 bone마다 chain을 트리 순서대로 뒤에 붙인다.
void BoneClothSimulator3DMenu::_on_chains_confirmed() {
	BoneClothSimulator3D *simulator = Object::cast_to<BoneClothSimulator3D>(ObjectDB::get_instance(simulator_id));
	ERR_FAIL_NULL(simulator);
	Skeleton3D *skeleton = simulator->get_skeleton();
	ERR_FAIL_NULL(skeleton);

	LocalVector<Vector2i> chains;
	for (TreeItem *item = bones_tree->get_root()->get_first_child(); item; item = item->get_next_in_tree()) {
		if (item->is_checked(0) && item->is_editable(0)) {
			collect_chains(skeleton, item->get_metadata(0), chains);
		}
	}
	if (chains.is_empty()) {
		return;
	}

	const int count = simulator->get_chain_count();
	EditorUndoRedoManager *undo_redo = EditorInterface::get_singleton()->get_editor_undo_redo();
	undo_redo->create_action("Add Bone Cloth Chains", UndoRedo::MERGE_DISABLE, simulator);
	undo_redo->add_do_property(simulator, "chain_count", count + int(chains.size()));
	for (uint32_t i = 0; i < chains.size(); i++) {
		const String path = "chains/" + itos(count + int(i)) + "/";
		undo_redo->add_do_property(simulator, path + String("root_bone_name"), skeleton->get_bone_name(chains[i].x));
		undo_redo->add_do_property(simulator, path + String("end_bone_name"), skeleton->get_bone_name(chains[i].y));
	}
	undo_redo->add_undo_property(simulator, "chain_count", count);
	undo_redo->commit_action();
}

// 메뉴가 다룰 simulator(노드 번호). 없으면 0.
void BoneClothSimulator3DMenu::set_simulator(BoneClothSimulator3D *p_simulator) {
	simulator_id = p_simulator ? p_simulator->get_instance_id() : 0;
}

// 폴더를 모두 돌며 프리셋 파일을 찾는다(addons 포함).
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

	// 프리셋 값을 하나씩 넣는다(undo하면 값과 curve가 하나하나 원래대로 돌아간다).
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

	if (p_id == OPTION_ADD_CHAINS) {
		ERR_FAIL_NULL_MSG(simulator->get_skeleton(), "Add Chains needs the BoneClothSimulator3D under a Skeleton3D.");
		_fill_bones_tree(simulator);
		const float scale = EditorInterface::get_singleton()->get_editor_scale();
		chains_dialog->popup_centered(Vector2i(Vector2(500.0f, 600.0f) * scale));
	}

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

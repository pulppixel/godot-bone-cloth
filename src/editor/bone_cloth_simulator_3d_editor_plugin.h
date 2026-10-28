// BoneClothSimulator3D의 에디터 기능(에디터 전용).
// simulator를 선택하면 3D 화면 위 툴바에 BoneClothSimulator3D 메뉴가 생긴다.
// gizmo도 여기서 추가한다.

#pragma once

#include "bone_cloth_simulator_3d.h"

#include "editor/bone_cloth_3d_gizmo_plugin.h"

#include <godot_cpp/classes/confirmation_dialog.hpp>
#include <godot_cpp/classes/editor_file_dialog.hpp>
#include <godot_cpp/classes/editor_file_system_directory.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/menu_button.hpp>
#include <godot_cpp/classes/popup_menu.hpp>
#include <godot_cpp/classes/tree.hpp>

using namespace godot;

// Edit Chains...: skeleton의 bone 트리에서 체크한 bone마다 chain을 넣고, 체크를 풀면 뺀다.
// Presets: 프로젝트의 프리셋 파일 목록.
// Save Preset...: 노드 값을 새 프리셋 파일로 저장한다.
// Show Radius: joint마다 반지름 구를 그릴지 말지 여부를 선택한다.
// Show Limit Angle: "각도 제한 원뿔"을 그릴지 말지 여부를 선택한다.
class BoneClothSimulator3DMenu : public MenuButton {
	GDCLASS(BoneClothSimulator3DMenu, MenuButton)

	uint64_t simulator_id = 0;
	PopupMenu *presets_menu = nullptr;
	PackedStringArray preset_paths;
	EditorFileDialog *save_dialog = nullptr;
	ConfirmationDialog *chains_dialog = nullptr;
	Tree *bones_tree = nullptr;
	PackedInt32Array top_roots;

	void _fill_bones_tree(BoneClothSimulator3D *p_simulator);
	TreeItem *_add_bone_item(Skeleton3D *p_skeleton, int p_bone, TreeItem *p_parent, const PackedInt32Array &p_in_chain, const PackedInt32Array &p_locked, bool p_covered);
	void _set_covered(TreeItem *p_first, bool p_covered);
	void _on_bone_edited();
	void _on_chains_confirmed();
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

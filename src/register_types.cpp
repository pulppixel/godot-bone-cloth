#include "register_types.h"

#include "bone_cloth_preset.h"
#include "bone_cloth_simulator_3d.h"
#include "collision/bone_cloth_collision_3d.h"
#include "collision/bone_cloth_collision_capsule_3d.h"
#include "collision/bone_cloth_collision_plane_3d.h"
#include "collision/bone_cloth_collision_sphere_3d.h"
#include "collision/bone_cloth_collision_tapered_capsule_3d.h"

#include "editor/bone_cloth_3d_gizmo_plugin.h"
#include "editor/bone_cloth_simulator_3d_editor_plugin.h"

#include <gdextension_interface.h>
#include <godot_cpp/classes/editor_plugin_registration.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_bone_cloth_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(BoneClothPreset);
		GDREGISTER_CLASS(BoneClothSimulator3D);
		GDREGISTER_VIRTUAL_CLASS(BoneClothCollision3D);
		GDREGISTER_CLASS(BoneClothCollisionSphere3D);
		GDREGISTER_CLASS(BoneClothCollisionCapsule3D);
		GDREGISTER_CLASS(BoneClothCollisionPlane3D);
		GDREGISTER_CLASS(BoneClothCollisionTaperedCapsule3D);
		BoneClothSimulator3D::add_monitors();
	}

	// 에디터 class는 internal로 등록한다(클래스 레퍼런스, Create 창, 자동완성에 안 나온다).
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		GDREGISTER_INTERNAL_CLASS(BoneClothCollision3DGizmoPlugin);
		GDREGISTER_INTERNAL_CLASS(BoneClothSimulator3DGizmoPlugin);
		GDREGISTER_INTERNAL_CLASS(BoneClothSimulator3DMenu);
		GDREGISTER_INTERNAL_CLASS(BoneClothSimulator3DEditorPlugin);
		EditorPlugins::add_by_type<BoneClothSimulator3DEditorPlugin>();
	}
}

void uninitialize_bone_cloth_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		BoneClothSimulator3D::remove_monitors();
	}
}

extern "C" {
	GDExtensionBool GDE_EXPORT bone_cloth_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
		GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
		init_obj.register_initializer(initialize_bone_cloth_module);
		init_obj.register_terminator(uninitialize_bone_cloth_module);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
		return init_obj.init();
	}
}

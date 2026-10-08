#include "register_types.h"

#include <gdextension_interface.h>

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "bone_cloth_simulator_3d.h"

using namespace godot;

void initialize_bone_cloth_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	GDREGISTER_CLASS(BoneClothSimulator3D);
}

void uninitialize_bone_cloth_module(ModuleInitializationLevel p_level) {
}

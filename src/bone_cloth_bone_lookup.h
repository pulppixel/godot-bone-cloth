#pragma once

#include <godot_cpp/classes/skeleton3d.hpp>

namespace godot {

inline int find_bone_or_warn(const Skeleton3D *p_skeleton, const String &p_bone_name, const String &p_label) {
	const int found = p_skeleton->find_bone(p_bone_name);
	if (found < 0 && !p_bone_name.is_empty()) {
		WARN_PRINT_ED(p_label + String(" not found: '") + p_bone_name + String("'."));
	}
	return found;
}

inline void read_bone_name(const Skeleton3D *p_skeleton, int &r_bone, String &r_bone_name, const String &p_label) {
	if (!p_skeleton || r_bone < 0) {
		return;
	}

	if (r_bone >= p_skeleton->get_bone_count()) {
		WARN_PRINT_ED(p_label + String(" index ") + itos(r_bone) + String(" is out of range."));
		r_bone = -1;
	} else {
		r_bone_name = p_skeleton->get_bone_name(r_bone);
	}
}

} // namespace godot

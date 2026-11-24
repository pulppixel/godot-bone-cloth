#include "bone_cloth_preset.h"

#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

static bool is_in_preset(const String &p_name) {
	static const char *const NOT_IN_PRESET[] = { "link_mode", "sync/value", "sync/bone_count", "anchor", "anchor_inertia", "backstop/bone_name", "backstop/bone", "backstop/position_offset", "chain_count", "ticks_per_second", "max_steps_per_frame", "camera_culling", "distance_culling", "distance_culling_fade" };
	for (const char *name : NOT_IN_PRESET) {
		if (p_name == name) {
			return false;
		}
	}
	return true;
}

void BoneClothPreset::_bind_methods() {
}

TypedArray<Dictionary> BoneClothPreset::_get_value_properties() {
	TypedArray<Dictionary> kept;
	const TypedArray<Dictionary> properties = ClassDB::class_get_property_list("BoneClothSimulator3D", true);
	for (int i = 0; i < properties.size(); i++) {
		const Dictionary property = properties[i];
		const int usage = property["usage"];
		const bool group = usage & (PROPERTY_USAGE_GROUP | PROPERTY_USAGE_SUBGROUP);
		if (group || ((usage & PROPERTY_USAGE_STORAGE) && is_in_preset(property["name"]))) {
			kept.push_back(property);
		}
	}
	return kept;
}

PackedStringArray BoneClothPreset::get_value_names() {
	PackedStringArray names;
	const TypedArray<Dictionary> properties = _get_value_properties();
	for (int i = 0; i < properties.size(); i++) {
		const Dictionary property = properties[i];
		if (int(property["usage"]) & PROPERTY_USAGE_STORAGE) {
			names.push_back(property["name"]);
		}
	}
	return names;
}

Variant BoneClothPreset::_get_default(const StringName &p_name) {
	return ClassDB::class_get_property_default_value("BoneClothSimulator3D", p_name);
}

Variant BoneClothPreset::copy_value(const Variant &p_value) {
	const Ref<Curve> curve = p_value;
	return curve.is_valid() ? Variant(curve->duplicate()) : p_value;
}

bool BoneClothPreset::_set(const StringName &p_name, const Variant &p_value) {
	if (!get_value_names().has(p_name)) {
		return false;
	}

	// 시뮬레이터 값이 float이라서, 0.1을 넣어도 기본값 0.1f와 같게 맞춘다.
	const Variant value = p_value.get_type() == Variant::FLOAT ? Variant(float(p_value)) : p_value;
	if (value == _get_default(p_name)) {
		values.erase(p_name);
	} else {
		values[p_name] = value;
	}
	emit_changed();
	return true;
}

bool BoneClothPreset::_get(const StringName &p_name, Variant &r_ret) const {
	const Variant *value = values.getptr(p_name);
	if (value) {
		r_ret = *value;
		return true;
	}

	if (!get_value_names().has(p_name)) {
		return false;
	}
	r_ret = _get_default(p_name);
	return true;
}

// 기본값인 값은 STORAGE를 빼서, 프리셋 파일에 바꾼 값만 저장한다.
void BoneClothPreset::_get_property_list(List<PropertyInfo> *p_list) const {
	const TypedArray<Dictionary> properties = _get_value_properties();
	for (int i = 0; i < properties.size(); i++) {
		const Dictionary property = properties[i];
		const StringName name = property["name"];
		uint32_t usage = int(property["usage"]);
		if (!values.has(name)) {
			usage &= ~PROPERTY_USAGE_STORAGE;
		}
		p_list->push_back(PropertyInfo(Variant::Type(int(property["type"])), name, PropertyHint(int(property["hint"])), property["hint_string"], usage, property["class_name"]));
	}
}

bool BoneClothPreset::_property_can_revert(const StringName &p_name) const {
	return values.has(p_name);
}

bool BoneClothPreset::_property_get_revert(const StringName &p_name, Variant &r_property) const {
	if (!values.has(p_name)) {
		return false;
	}
	r_property = _get_default(p_name);
	return true;
}

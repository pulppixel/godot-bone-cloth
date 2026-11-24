#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/templates/hash_map.hpp>

using namespace godot;

class BoneClothPreset : public Resource {
	GDCLASS(BoneClothPreset, Resource)

	HashMap<StringName, Variant> values;

	static TypedArray<Dictionary> _get_value_properties();
	static Variant _get_default(const StringName &p_name);

protected:
	static void _bind_methods();
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	bool _property_can_revert(const StringName &p_name) const;
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const;

public:
	static PackedStringArray get_value_names();
	static Variant copy_value(const Variant &p_value);
};

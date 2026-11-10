#include "bone_cloth_wind_3d.h"

#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

static const char *WIND_GROUP = "bone_cloth_wind";

void BoneClothWind3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_strength", "strength"), &BoneClothWind3D::set_strength);
	ClassDB::bind_method(D_METHOD("get_strength"), &BoneClothWind3D::get_strength);
	ClassDB::bind_method(D_METHOD("set_turbulence", "turbulence"), &BoneClothWind3D::set_turbulence);
	ClassDB::bind_method(D_METHOD("get_turbulence"), &BoneClothWind3D::get_turbulence);
	ClassDB::bind_method(D_METHOD("set_frequency", "frequency"), &BoneClothWind3D::set_frequency);
	ClassDB::bind_method(D_METHOD("get_frequency"), &BoneClothWind3D::get_frequency);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "strength", PROPERTY_HINT_RANGE, "0,30,0.01,or_greater,suffix:m/s^2"), "set_strength", "get_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "turbulence", PROPERTY_HINT_RANGE, "0,2,0.01"), "set_turbulence", "get_turbulence");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frequency", PROPERTY_HINT_RANGE, "0,2,0.01"), "set_frequency", "get_frequency");
}

void BoneClothWind3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_ENTER_TREE) {
		add_to_group(WIND_GROUP);
	}
}

BoneClothWind3D::Total BoneClothWind3D::get_total_wind(const Node3D *p_node) {
	Total total;
	float weight = 0.0f;
	const Ref<World3D> world = p_node->get_world_3d();
	const TypedArray<Node> winds = p_node->get_tree()->get_nodes_in_group(WIND_GROUP);
	for (int i = 0; i < winds.size(); i++) {
		const BoneClothWind3D *wind = Object::cast_to<BoneClothWind3D>(winds[i]);
		if (wind && wind->is_visible_in_tree() && wind->get_world_3d() == world) {
			total.force -= wind->get_global_basis().get_column(2).normalized() * wind->strength;
			total.turbulence += wind->turbulence * wind->strength;
			total.frequency += wind->frequency * wind->strength;
			weight += wind->strength;
		}
	}
	if (weight > 0.0f) {
		total.turbulence /= weight;
		total.frequency /= weight;
	}
	return total;
}

void BoneClothWind3D::set_strength(float p_strength) {
	strength = p_strength;
}

float BoneClothWind3D::get_strength() const {
	return strength;
}

void BoneClothWind3D::set_turbulence(float p_turbulence) {
	turbulence = p_turbulence;
}

float BoneClothWind3D::get_turbulence() const {
	return turbulence;
}

void BoneClothWind3D::set_frequency(float p_frequency) {
	frequency = p_frequency;
}

float BoneClothWind3D::get_frequency() const {
	return frequency;
}

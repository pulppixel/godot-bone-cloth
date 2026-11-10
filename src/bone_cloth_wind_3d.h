#pragma once

#include <godot_cpp/classes/node3d.hpp>

using namespace godot;

class BoneClothWind3D : public Node3D {
	GDCLASS(BoneClothWind3D, Node3D)

	float strength = 5.0f;
	float turbulence = 1.0f;
	float frequency = 1.0f;

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	struct Total {
		Vector3 force;
		float turbulence = 0.0f;
		float frequency = 0.0f;
	};

	static Total get_total_wind(const Node3D *p_node);

	void set_strength(float p_strength);
	float get_strength() const;
	void set_turbulence(float p_turbulence);
	float get_turbulence() const;
	void set_frequency(float p_frequency);
	float get_frequency() const;
};

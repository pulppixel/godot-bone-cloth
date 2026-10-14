// The physics values of a BoneClothSimulator3D saved as a resource, to start a skirt or hair from known numbers.
// Applied by copy, as Magica Cloth's and KawaiiPhysics' presets are: the chains, the link mode and the collisions stay the node's.

#pragma once

#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

class BoneClothPreset : public Resource {
	GDCLASS(BoneClothPreset, Resource)

	// The same defaults as BoneClothSimulator3D.
	float link_stiffness = 0.0f;
	float end_bone_length = 0.1f;
	float drag = 0.1f;
	Ref<Curve> drag_damping_curve;
	float stiffness = 0.05f;
	Ref<Curve> stiffness_damping_curve;
	float radius = 0.03f;
	Ref<Curve> radius_damping_curve;
	float limit_angle = 0.0f;
	Ref<Curve> limit_angle_damping_curve;
	float gravity = 0.0f;
	Ref<Curve> gravity_damping_curve;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	float inertia = 1.0f;
	float inertia_smoothing = 0.4f;
	float movement_speed_limit = 5.0f;
	float rotation_speed_limit = Math::deg_to_rad(720.0f);
	float joint_speed_limit = 4.0f;
	float teleport_distance = 0.5f;
	float teleport_angle = Math::deg_to_rad(90.0f);

protected:
	static void _bind_methods();

public:
	// The property paths a preset carries, the same on the preset and on the simulator, so the copy goes name by name.
	static constexpr const char *VALUES[] = {
		"link_stiffness",
		"end_bone_length",
		"drag/value",
		"drag/damping_curve",
		"stiffness/value",
		"stiffness/damping_curve",
		"radius/value",
		"radius/damping_curve",
		"limit_angle/value",
		"limit_angle/damping_curve",
		"gravity/value",
		"gravity/damping_curve",
		"gravity/direction",
		"inertia",
		"inertia_smoothing",
		"movement_speed_limit",
		"rotation_speed_limit",
		"joint_speed_limit",
		"teleport_distance",
		"teleport_angle",
	};

	// A curve goes over as a copy, so tuning the node's curve after an apply leaves the preset's file as it was.
	static Variant copy_value(const Variant &p_value);
	void set_link_stiffness(float p_stiffness);
	float get_link_stiffness() const;
	void set_end_bone_length(float p_length);
	float get_end_bone_length() const;
	void set_drag(float p_drag);
	float get_drag() const;
	void set_drag_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_drag_damping_curve() const;
	void set_stiffness(float p_stiffness);
	float get_stiffness() const;
	void set_stiffness_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_stiffness_damping_curve() const;
	void set_radius(float p_radius);
	float get_radius() const;
	void set_radius_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_radius_damping_curve() const;
	void set_limit_angle(float p_angle);
	float get_limit_angle() const;
	void set_limit_angle_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_limit_angle_damping_curve() const;
	void set_gravity(float p_gravity);
	float get_gravity() const;
	void set_gravity_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_gravity_damping_curve() const;
	void set_gravity_direction(const Vector3 &p_direction);
	Vector3 get_gravity_direction() const;
	void set_inertia(float p_inertia);
	float get_inertia() const;
	void set_inertia_smoothing(float p_smoothing);
	float get_inertia_smoothing() const;
	void set_movement_speed_limit(float p_limit);
	float get_movement_speed_limit() const;
	void set_rotation_speed_limit(float p_limit);
	float get_rotation_speed_limit() const;
	void set_joint_speed_limit(float p_limit);
	float get_joint_speed_limit() const;
	void set_teleport_distance(float p_distance);
	float get_teleport_distance() const;
	void set_teleport_angle(float p_angle);
	float get_teleport_angle() const;
};

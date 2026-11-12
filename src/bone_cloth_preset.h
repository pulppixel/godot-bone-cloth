#pragma once

#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

class BoneClothPreset : public Resource {
	GDCLASS(BoneClothPreset, Resource)

	float link_stiffness = 0.0f;
	bool link_split = false;
	float link_spacing = 0.0f;
	Ref<Curve> link_spacing_damping_curve;
	float drag = 0.1f;
	Ref<Curve> drag_damping_curve;
	float stiffness = 0.05f;
	Ref<Curve> stiffness_damping_curve;
	float radius = 0.03f;
	Ref<Curve> radius_damping_curve;
	bool rest_overlap = true;
	float limit_angle = 0.0f;
	Ref<Curve> limit_angle_damping_curve;
	float limit_angle_stiffness = 1.0f;
	float gravity = 0.0f;
	Ref<Curve> gravity_damping_curve;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	float wind_influence = 1.0f;
	Ref<Curve> wind_influence_damping_curve;
	float wind_sync = 0.7f;
	float wind_turbulence = 1.0f;
	float wind_frequency = 1.0f;
	float inertia = 1.0f;
	float inertia_smoothing = 0.4f;
	float depth_inertia = 0.0f;
	float local_movement_speed_limit = 0.0f;
	float local_rotation_speed_limit = 0.0f;
	float movement_speed_limit = 5.0f;
	float rotation_speed_limit = Math::deg_to_rad(720.0f);
	float joint_speed_limit = 4.0f;
	float teleport_distance = 0.5f;
	float teleport_angle = Math::deg_to_rad(90.0f);
	float root_spring_stiffness = 0.0f;
	float root_spring_limit = 0.04f;
	float root_spring_forward_limit = 0.005f;
	float max_distance = 0.0f;
	Ref<Curve> max_distance_damping_curve;
	float backstop_distance = 0.0f;
	Ref<Curve> backstop_damping_curve;
	float backstop_radius = 10.0f;
	float movement_limit_stiffness = 1.0f;

protected:
	static void _bind_methods();

public:
	static PackedStringArray get_value_names();
	static Variant copy_value(const Variant &p_value);
	void set_link_stiffness(float p_stiffness);
	float get_link_stiffness() const;
	void set_link_split_enabled(bool p_enabled);
	bool is_link_split_enabled() const;
	void set_link_spacing(float p_spacing);
	float get_link_spacing() const;
	void set_link_spacing_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_link_spacing_damping_curve() const;
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
	void set_rest_overlap_enabled(bool p_enabled);
	bool is_rest_overlap_enabled() const;
	void set_limit_angle(float p_angle);
	float get_limit_angle() const;
	void set_limit_angle_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_limit_angle_damping_curve() const;
	void set_limit_angle_stiffness(float p_stiffness);
	float get_limit_angle_stiffness() const;
	void set_gravity(float p_gravity);
	float get_gravity() const;
	void set_gravity_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_gravity_damping_curve() const;
	void set_gravity_direction(const Vector3 &p_direction);
	Vector3 get_gravity_direction() const;
	void set_wind_influence(float p_influence);
	float get_wind_influence() const;
	void set_wind_influence_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_wind_influence_damping_curve() const;
	void set_wind_sync(float p_sync);
	float get_wind_sync() const;
	void set_wind_turbulence(float p_turbulence);
	float get_wind_turbulence() const;
	void set_wind_frequency(float p_frequency);
	float get_wind_frequency() const;
	void set_inertia(float p_inertia);
	float get_inertia() const;
	void set_inertia_smoothing(float p_smoothing);
	float get_inertia_smoothing() const;
	void set_depth_inertia(float p_depth_inertia);
	float get_depth_inertia() const;
	void set_local_movement_speed_limit(float p_limit);
	float get_local_movement_speed_limit() const;
	void set_local_rotation_speed_limit(float p_limit);
	float get_local_rotation_speed_limit() const;
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
	void set_root_spring_stiffness(float p_stiffness);
	float get_root_spring_stiffness() const;
	void set_root_spring_limit(float p_limit);
	float get_root_spring_limit() const;
	void set_root_spring_forward_limit(float p_limit);
	float get_root_spring_forward_limit() const;
	void set_max_distance(float p_distance);
	float get_max_distance() const;
	void set_max_distance_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_max_distance_damping_curve() const;
	void set_backstop_distance(float p_distance);
	float get_backstop_distance() const;
	void set_backstop_damping_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_backstop_damping_curve() const;
	void set_backstop_radius(float p_radius);
	float get_backstop_radius() const;
	void set_movement_limit_stiffness(float p_stiffness);
	float get_movement_limit_stiffness() const;
};

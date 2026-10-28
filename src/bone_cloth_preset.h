// preset을 통해 치마나 머리카락 등의 값을 설정할 수 있다.
// 이 작업을 위해 BoneClothSimulator3D의 설정 값을 Godot Resource로 저장한다.
// 동작은 참조 복사가 아닌, 값 복사로 이루어진다. (chain, link mode, collision은 rig마다 다르기 때문에 preset에 넣지 않는다)

#pragma once

#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

class BoneClothPreset : public Resource {
	GDCLASS(BoneClothPreset, Resource)

	// 기본값은 BoneClothSimulator3D와 같다.
	float link_stiffness = 0.0f;
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

protected:
	static void _bind_methods();

public:
	// 이 배열의 이름으로 preset과 simulator 사이에 값을 복사한다.
	// 쓰는 곳: apply_preset, make_preset, 에디터의 Presets 메뉴
	static constexpr const char *VALUES[] = {
		"link_stiffness",
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
		"depth_inertia",
		"local_movement_speed_limit",
		"local_rotation_speed_limit",
		"movement_speed_limit",
		"rotation_speed_limit",
		"joint_speed_limit",
		"teleport_distance",
		"teleport_angle",
		"root_spring_stiffness",
		"root_spring_limit",
		"root_spring_forward_limit",
	};

	// curve는 복사본을 넘긴다. 대상 노드의 값을 바꿔도, 이 preset의 값이 바뀌지 않게 할 수 있다.
	static Variant copy_value(const Variant &p_value);
	void set_link_stiffness(float p_stiffness);
	float get_link_stiffness() const;
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
};

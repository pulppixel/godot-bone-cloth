#include "bone_cloth_simulator_3d.h"

#include "bone_cloth_bone_lookup.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

#include <atomic>
#include <chrono>

using namespace godot;

struct MonitorSum {
	std::atomic<uint64_t> sum = 0;
	uint64_t read_frame = 0;
	double average = 0.0;

	double read() {
		const uint64_t frame = Engine::get_singleton()->get_process_frames();
		if (frame != read_frame) {
			average = double(sum.exchange(0)) / double(frame - read_frame);
			read_frame = frame;
		}
		return average;
	}
};

static MonitorSum monitor_time;
static MonitorSum monitor_joints;

static double get_monitor_time() {
	return monitor_time.read() / 1e9;
}

static int64_t get_monitor_joints() {
	return int64_t(Math::round(monitor_joints.read()));
}

void BoneClothSimulator3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("apply_preset", "preset"), &BoneClothSimulator3D::apply_preset);
	ClassDB::bind_method(D_METHOD("make_preset"), &BoneClothSimulator3D::make_preset);

	ClassDB::bind_method(D_METHOD("set_chain_count", "count"), &BoneClothSimulator3D::set_chain_count);
	ClassDB::bind_method(D_METHOD("get_chain_count"), &BoneClothSimulator3D::get_chain_count);
	ClassDB::bind_method(D_METHOD("set_root_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_root_bone_name);
	ClassDB::bind_method(D_METHOD("get_root_bone_name", "index"), &BoneClothSimulator3D::get_root_bone_name);
	ClassDB::bind_method(D_METHOD("set_root_bone", "index", "bone"), &BoneClothSimulator3D::set_root_bone);
	ClassDB::bind_method(D_METHOD("get_root_bone", "index"), &BoneClothSimulator3D::get_root_bone);
	ClassDB::bind_method(D_METHOD("set_end_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_end_bone_name);
	ClassDB::bind_method(D_METHOD("get_end_bone_name", "index"), &BoneClothSimulator3D::get_end_bone_name);
	ClassDB::bind_method(D_METHOD("set_end_bone", "index", "bone"), &BoneClothSimulator3D::set_end_bone);
	ClassDB::bind_method(D_METHOD("get_end_bone", "index"), &BoneClothSimulator3D::get_end_bone);
	ClassDB::bind_method(D_METHOD("set_extend_end_bone", "index", "enabled"), &BoneClothSimulator3D::set_extend_end_bone);
	ClassDB::bind_method(D_METHOD("is_end_bone_extended", "index"), &BoneClothSimulator3D::is_end_bone_extended);
	ClassDB::bind_method(D_METHOD("set_end_bone_direction", "index", "bone_direction"), &BoneClothSimulator3D::set_end_bone_direction);
	ClassDB::bind_method(D_METHOD("get_end_bone_direction", "index"), &BoneClothSimulator3D::get_end_bone_direction);
	ClassDB::bind_method(D_METHOD("set_end_bone_length", "index", "length"), &BoneClothSimulator3D::set_end_bone_length);
	ClassDB::bind_method(D_METHOD("get_end_bone_length", "index"), &BoneClothSimulator3D::get_end_bone_length);

	ClassDB::bind_method(D_METHOD("set_enable_all_child_collisions", "index", "enabled"), &BoneClothSimulator3D::set_enable_all_child_collisions);
	ClassDB::bind_method(D_METHOD("are_all_child_collisions_enabled", "index"), &BoneClothSimulator3D::are_all_child_collisions_enabled);
	ClassDB::bind_method(D_METHOD("set_exclude_collision_path", "index", "collision", "node_path"), &BoneClothSimulator3D::set_exclude_collision_path);
	ClassDB::bind_method(D_METHOD("get_exclude_collision_path", "index", "collision"), &BoneClothSimulator3D::get_exclude_collision_path);
	ClassDB::bind_method(D_METHOD("set_exclude_collision_count", "index", "count"), &BoneClothSimulator3D::set_exclude_collision_count);
	ClassDB::bind_method(D_METHOD("get_exclude_collision_count", "index"), &BoneClothSimulator3D::get_exclude_collision_count);
	ClassDB::bind_method(D_METHOD("clear_exclude_collisions", "index"), &BoneClothSimulator3D::clear_exclude_collisions);
	ClassDB::bind_method(D_METHOD("set_collision_path", "index", "collision", "node_path"), &BoneClothSimulator3D::set_collision_path);
	ClassDB::bind_method(D_METHOD("get_collision_path", "index", "collision"), &BoneClothSimulator3D::get_collision_path);
	ClassDB::bind_method(D_METHOD("set_collision_count", "index", "count"), &BoneClothSimulator3D::set_collision_count);
	ClassDB::bind_method(D_METHOD("get_collision_count", "index"), &BoneClothSimulator3D::get_collision_count);
	ClassDB::bind_method(D_METHOD("clear_collisions", "index"), &BoneClothSimulator3D::clear_collisions);

	ClassDB::bind_method(D_METHOD("set_link_mode", "mode"), &BoneClothSimulator3D::set_link_mode);
	ClassDB::bind_method(D_METHOD("get_link_mode"), &BoneClothSimulator3D::get_link_mode);
	ClassDB::bind_method(D_METHOD("set_link_stiffness", "stiffness"), &BoneClothSimulator3D::set_link_stiffness);
	ClassDB::bind_method(D_METHOD("get_link_stiffness"), &BoneClothSimulator3D::get_link_stiffness);
	ClassDB::bind_method(D_METHOD("set_link_split_enabled", "enabled"), &BoneClothSimulator3D::set_link_split_enabled);
	ClassDB::bind_method(D_METHOD("is_link_split_enabled"), &BoneClothSimulator3D::is_link_split_enabled);
	ClassDB::bind_method(D_METHOD("set_link_spacing", "spacing"), &BoneClothSimulator3D::set_link_spacing);
	ClassDB::bind_method(D_METHOD("get_link_spacing"), &BoneClothSimulator3D::get_link_spacing);
	ClassDB::bind_method(D_METHOD("set_link_spacing_damping_curve", "curve"), &BoneClothSimulator3D::set_link_spacing_damping_curve);
	ClassDB::bind_method(D_METHOD("get_link_spacing_damping_curve"), &BoneClothSimulator3D::get_link_spacing_damping_curve);
	ClassDB::bind_method(D_METHOD("set_link_bending", "bending"), &BoneClothSimulator3D::set_link_bending);
	ClassDB::bind_method(D_METHOD("get_link_bending"), &BoneClothSimulator3D::get_link_bending);

	ClassDB::bind_method(D_METHOD("set_drag", "drag"), &BoneClothSimulator3D::set_drag);
	ClassDB::bind_method(D_METHOD("get_drag"), &BoneClothSimulator3D::get_drag);
	ClassDB::bind_method(D_METHOD("set_drag_damping_curve", "curve"), &BoneClothSimulator3D::set_drag_damping_curve);
	ClassDB::bind_method(D_METHOD("get_drag_damping_curve"), &BoneClothSimulator3D::get_drag_damping_curve);

	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothSimulator3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothSimulator3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_stiffness_damping_curve", "curve"), &BoneClothSimulator3D::set_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("get_stiffness_damping_curve"), &BoneClothSimulator3D::get_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("set_stiffness_velocity_damping", "damping"), &BoneClothSimulator3D::set_stiffness_velocity_damping);
	ClassDB::bind_method(D_METHOD("get_stiffness_velocity_damping"), &BoneClothSimulator3D::get_stiffness_velocity_damping);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothSimulator3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothSimulator3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius_damping_curve", "curve"), &BoneClothSimulator3D::set_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("get_radius_damping_curve"), &BoneClothSimulator3D::get_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("set_rest_overlap_enabled", "enabled"), &BoneClothSimulator3D::set_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("is_rest_overlap_enabled"), &BoneClothSimulator3D::is_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("set_collision_friction", "friction"), &BoneClothSimulator3D::set_collision_friction);
	ClassDB::bind_method(D_METHOD("get_collision_friction"), &BoneClothSimulator3D::get_collision_friction);
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothSimulator3D::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothSimulator3D::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_limit_angle_damping_curve", "curve"), &BoneClothSimulator3D::set_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("get_limit_angle_damping_curve"), &BoneClothSimulator3D::get_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("set_limit_angle_stiffness", "stiffness"), &BoneClothSimulator3D::set_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("get_limit_angle_stiffness"), &BoneClothSimulator3D::get_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("set_limit_angle_mode", "mode"), &BoneClothSimulator3D::set_limit_angle_mode);
	ClassDB::bind_method(D_METHOD("get_limit_angle_mode"), &BoneClothSimulator3D::get_limit_angle_mode);
	ClassDB::bind_method(D_METHOD("set_limit_angle_lower", "lower"), &BoneClothSimulator3D::set_limit_angle_lower);
	ClassDB::bind_method(D_METHOD("get_limit_angle_lower"), &BoneClothSimulator3D::get_limit_angle_lower);
	ClassDB::bind_method(D_METHOD("set_limit_angle_upper", "upper"), &BoneClothSimulator3D::set_limit_angle_upper);
	ClassDB::bind_method(D_METHOD("get_limit_angle_upper"), &BoneClothSimulator3D::get_limit_angle_upper);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity_damping_curve", "curve"), &BoneClothSimulator3D::set_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("get_gravity_damping_curve"), &BoneClothSimulator3D::get_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity_direction", "direction"), &BoneClothSimulator3D::set_gravity_direction);
	ClassDB::bind_method(D_METHOD("get_gravity_direction"), &BoneClothSimulator3D::get_gravity_direction);
	ClassDB::bind_method(D_METHOD("set_gravity_falloff", "falloff"), &BoneClothSimulator3D::set_gravity_falloff);
	ClassDB::bind_method(D_METHOD("get_gravity_falloff"), &BoneClothSimulator3D::get_gravity_falloff);
	ClassDB::bind_method(D_METHOD("set_wind_influence", "influence"), &BoneClothSimulator3D::set_wind_influence);
	ClassDB::bind_method(D_METHOD("get_wind_influence"), &BoneClothSimulator3D::get_wind_influence);
	ClassDB::bind_method(D_METHOD("set_wind_influence_damping_curve", "curve"), &BoneClothSimulator3D::set_wind_influence_damping_curve);
	ClassDB::bind_method(D_METHOD("get_wind_influence_damping_curve"), &BoneClothSimulator3D::get_wind_influence_damping_curve);
	ClassDB::bind_method(D_METHOD("set_wind_sync", "sync"), &BoneClothSimulator3D::set_wind_sync);
	ClassDB::bind_method(D_METHOD("get_wind_sync"), &BoneClothSimulator3D::get_wind_sync);
	ClassDB::bind_method(D_METHOD("set_wind_turbulence", "turbulence"), &BoneClothSimulator3D::set_wind_turbulence);
	ClassDB::bind_method(D_METHOD("get_wind_turbulence"), &BoneClothSimulator3D::get_wind_turbulence);
	ClassDB::bind_method(D_METHOD("set_wind_frequency", "frequency"), &BoneClothSimulator3D::set_wind_frequency);
	ClassDB::bind_method(D_METHOD("get_wind_frequency"), &BoneClothSimulator3D::get_wind_frequency);
	ClassDB::bind_method(D_METHOD("set_sync_bone_count", "count"), &BoneClothSimulator3D::set_sync_bone_count);
	ClassDB::bind_method(D_METHOD("get_sync_bone_count"), &BoneClothSimulator3D::get_sync_bone_count);
	ClassDB::bind_method(D_METHOD("set_sync_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_sync_bone_name);
	ClassDB::bind_method(D_METHOD("get_sync_bone_name", "index"), &BoneClothSimulator3D::get_sync_bone_name);
	ClassDB::bind_method(D_METHOD("set_sync_bone", "index", "bone"), &BoneClothSimulator3D::set_sync_bone);
	ClassDB::bind_method(D_METHOD("get_sync_bone", "index"), &BoneClothSimulator3D::get_sync_bone);
	ClassDB::bind_method(D_METHOD("set_sync_bone_direction", "index", "axis", "direction"), &BoneClothSimulator3D::set_sync_bone_direction);
	ClassDB::bind_method(D_METHOD("get_sync_bone_direction", "index", "axis"), &BoneClothSimulator3D::get_sync_bone_direction);
	ClassDB::bind_method(D_METHOD("set_sync_bone_falloff_inner", "index", "radius"), &BoneClothSimulator3D::set_sync_bone_falloff_inner);
	ClassDB::bind_method(D_METHOD("get_sync_bone_falloff_inner", "index"), &BoneClothSimulator3D::get_sync_bone_falloff_inner);
	ClassDB::bind_method(D_METHOD("set_sync_bone_falloff_outer", "index", "radius"), &BoneClothSimulator3D::set_sync_bone_falloff_outer);
	ClassDB::bind_method(D_METHOD("get_sync_bone_falloff_outer", "index"), &BoneClothSimulator3D::get_sync_bone_falloff_outer);
	ClassDB::bind_method(D_METHOD("set_sync_bone_falloff_amount", "index", "amount"), &BoneClothSimulator3D::set_sync_bone_falloff_amount);
	ClassDB::bind_method(D_METHOD("get_sync_bone_falloff_amount", "index"), &BoneClothSimulator3D::get_sync_bone_falloff_amount);
	ClassDB::bind_method(D_METHOD("set_sync", "sync"), &BoneClothSimulator3D::set_sync);
	ClassDB::bind_method(D_METHOD("get_sync"), &BoneClothSimulator3D::get_sync);

	ClassDB::bind_method(D_METHOD("set_anchor", "anchor"), &BoneClothSimulator3D::set_anchor);
	ClassDB::bind_method(D_METHOD("get_anchor"), &BoneClothSimulator3D::get_anchor);
	ClassDB::bind_method(D_METHOD("set_anchor_inertia", "inertia"), &BoneClothSimulator3D::set_anchor_inertia);
	ClassDB::bind_method(D_METHOD("get_anchor_inertia"), &BoneClothSimulator3D::get_anchor_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothSimulator3D::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothSimulator3D::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia_smoothing", "smoothing"), &BoneClothSimulator3D::set_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("get_inertia_smoothing"), &BoneClothSimulator3D::get_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("set_depth_inertia", "depth_inertia"), &BoneClothSimulator3D::set_depth_inertia);
	ClassDB::bind_method(D_METHOD("get_depth_inertia"), &BoneClothSimulator3D::get_depth_inertia);
	ClassDB::bind_method(D_METHOD("set_centrifugal_acceleration", "acceleration"), &BoneClothSimulator3D::set_centrifugal_acceleration);
	ClassDB::bind_method(D_METHOD("get_centrifugal_acceleration"), &BoneClothSimulator3D::get_centrifugal_acceleration);
	ClassDB::bind_method(D_METHOD("set_local_movement_speed_limit", "limit"), &BoneClothSimulator3D::set_local_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_local_movement_speed_limit"), &BoneClothSimulator3D::get_local_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_local_rotation_speed_limit", "limit"), &BoneClothSimulator3D::set_local_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_local_rotation_speed_limit"), &BoneClothSimulator3D::get_local_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_movement_speed_limit", "limit"), &BoneClothSimulator3D::set_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_movement_speed_limit"), &BoneClothSimulator3D::get_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_rotation_speed_limit", "limit"), &BoneClothSimulator3D::set_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_rotation_speed_limit"), &BoneClothSimulator3D::get_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_joint_speed_limit", "limit"), &BoneClothSimulator3D::set_joint_speed_limit);
	ClassDB::bind_method(D_METHOD("get_joint_speed_limit"), &BoneClothSimulator3D::get_joint_speed_limit);
	ClassDB::bind_method(D_METHOD("set_shock_propagation", "propagation"), &BoneClothSimulator3D::set_shock_propagation);
	ClassDB::bind_method(D_METHOD("get_shock_propagation"), &BoneClothSimulator3D::get_shock_propagation);
	ClassDB::bind_method(D_METHOD("set_shock_threshold", "threshold"), &BoneClothSimulator3D::set_shock_threshold);
	ClassDB::bind_method(D_METHOD("get_shock_threshold"), &BoneClothSimulator3D::get_shock_threshold);
	ClassDB::bind_method(D_METHOD("set_teleport_distance", "distance"), &BoneClothSimulator3D::set_teleport_distance);
	ClassDB::bind_method(D_METHOD("get_teleport_distance"), &BoneClothSimulator3D::get_teleport_distance);
	ClassDB::bind_method(D_METHOD("set_teleport_angle", "angle"), &BoneClothSimulator3D::set_teleport_angle);
	ClassDB::bind_method(D_METHOD("get_teleport_angle"), &BoneClothSimulator3D::get_teleport_angle);
	ClassDB::bind_method(D_METHOD("set_teleport_mode", "mode"), &BoneClothSimulator3D::set_teleport_mode);
	ClassDB::bind_method(D_METHOD("get_teleport_mode"), &BoneClothSimulator3D::get_teleport_mode);
	ClassDB::bind_method(D_METHOD("set_root_spring_stiffness", "stiffness"), &BoneClothSimulator3D::set_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("get_root_spring_stiffness"), &BoneClothSimulator3D::get_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("set_root_spring_limit", "limit"), &BoneClothSimulator3D::set_root_spring_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_limit"), &BoneClothSimulator3D::get_root_spring_limit);
	ClassDB::bind_method(D_METHOD("set_root_spring_forward_limit", "limit"), &BoneClothSimulator3D::set_root_spring_forward_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_forward_limit"), &BoneClothSimulator3D::get_root_spring_forward_limit);
	ClassDB::bind_method(D_METHOD("set_root_spring_noise", "noise"), &BoneClothSimulator3D::set_root_spring_noise);
	ClassDB::bind_method(D_METHOD("get_root_spring_noise"), &BoneClothSimulator3D::get_root_spring_noise);
	ClassDB::bind_method(D_METHOD("set_max_distance", "distance"), &BoneClothSimulator3D::set_max_distance);
	ClassDB::bind_method(D_METHOD("get_max_distance"), &BoneClothSimulator3D::get_max_distance);
	ClassDB::bind_method(D_METHOD("set_max_distance_damping_curve", "curve"), &BoneClothSimulator3D::set_max_distance_damping_curve);
	ClassDB::bind_method(D_METHOD("get_max_distance_damping_curve"), &BoneClothSimulator3D::get_max_distance_damping_curve);
	ClassDB::bind_method(D_METHOD("set_backstop_bone_name", "bone_name"), &BoneClothSimulator3D::set_backstop_bone_name);
	ClassDB::bind_method(D_METHOD("get_backstop_bone_name"), &BoneClothSimulator3D::get_backstop_bone_name);
	ClassDB::bind_method(D_METHOD("set_backstop_bone", "bone"), &BoneClothSimulator3D::set_backstop_bone);
	ClassDB::bind_method(D_METHOD("get_backstop_bone"), &BoneClothSimulator3D::get_backstop_bone);
	ClassDB::bind_method(D_METHOD("set_backstop_position_offset", "offset"), &BoneClothSimulator3D::set_backstop_position_offset);
	ClassDB::bind_method(D_METHOD("get_backstop_position_offset"), &BoneClothSimulator3D::get_backstop_position_offset);
	ClassDB::bind_method(D_METHOD("set_backstop_distance", "distance"), &BoneClothSimulator3D::set_backstop_distance);
	ClassDB::bind_method(D_METHOD("get_backstop_distance"), &BoneClothSimulator3D::get_backstop_distance);
	ClassDB::bind_method(D_METHOD("set_backstop_damping_curve", "curve"), &BoneClothSimulator3D::set_backstop_damping_curve);
	ClassDB::bind_method(D_METHOD("get_backstop_damping_curve"), &BoneClothSimulator3D::get_backstop_damping_curve);
	ClassDB::bind_method(D_METHOD("set_backstop_radius", "radius"), &BoneClothSimulator3D::set_backstop_radius);
	ClassDB::bind_method(D_METHOD("get_backstop_radius"), &BoneClothSimulator3D::get_backstop_radius);
	ClassDB::bind_method(D_METHOD("set_movement_limit_stiffness", "stiffness"), &BoneClothSimulator3D::set_movement_limit_stiffness);
	ClassDB::bind_method(D_METHOD("get_movement_limit_stiffness"), &BoneClothSimulator3D::get_movement_limit_stiffness);
	ClassDB::bind_method(D_METHOD("set_tether_limit", "limit"), &BoneClothSimulator3D::set_tether_limit);
	ClassDB::bind_method(D_METHOD("get_tether_limit"), &BoneClothSimulator3D::get_tether_limit);
	ClassDB::bind_method(D_METHOD("set_ticks_per_second", "ticks"), &BoneClothSimulator3D::set_ticks_per_second);
	ClassDB::bind_method(D_METHOD("get_ticks_per_second"), &BoneClothSimulator3D::get_ticks_per_second);
	ClassDB::bind_method(D_METHOD("set_max_steps_per_frame", "steps"), &BoneClothSimulator3D::set_max_steps_per_frame);
	ClassDB::bind_method(D_METHOD("get_max_steps_per_frame"), &BoneClothSimulator3D::get_max_steps_per_frame);
	ClassDB::bind_method(D_METHOD("set_stabilization_time", "seconds"), &BoneClothSimulator3D::set_stabilization_time);
	ClassDB::bind_method(D_METHOD("get_stabilization_time"), &BoneClothSimulator3D::get_stabilization_time);
	ClassDB::bind_method(D_METHOD("set_camera_culling", "mode"), &BoneClothSimulator3D::set_camera_culling);
	ClassDB::bind_method(D_METHOD("get_camera_culling"), &BoneClothSimulator3D::get_camera_culling);
	ClassDB::bind_method(D_METHOD("set_distance_culling", "distance"), &BoneClothSimulator3D::set_distance_culling);
	ClassDB::bind_method(D_METHOD("get_distance_culling"), &BoneClothSimulator3D::get_distance_culling);
	ClassDB::bind_method(D_METHOD("set_distance_culling_fade", "fade"), &BoneClothSimulator3D::set_distance_culling_fade);
	ClassDB::bind_method(D_METHOD("get_distance_culling_fade"), &BoneClothSimulator3D::get_distance_culling_fade);

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), "set_link_mode", "get_link_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "link_split"), "set_link_split_enabled", "is_link_split_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_spacing/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_spacing", "get_link_spacing");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "link_spacing/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_link_spacing_damping_curve", "get_link_spacing_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_bending", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_bending", "get_link_bending");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_damping_curve", "get_drag_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_stiffness_damping_curve", "get_stiffness_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/velocity_damping", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_stiffness_velocity_damping", "get_stiffness_velocity_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_radius_damping_curve", "get_radius_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "radius/rest_overlap"), "set_rest_overlap_enabled", "is_rest_overlap_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_friction", PROPERTY_HINT_RANGE, "0,0.5,0.001"), "set_collision_friction", "get_collision_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_limit_angle_damping_curve", "get_limit_angle_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_limit_angle_stiffness", "get_limit_angle_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "limit_angle/mode", PROPERTY_HINT_ENUM, "Cone,Per Axis"), "set_limit_angle_mode", "get_limit_angle_mode");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "limit_angle/lower", PROPERTY_HINT_RANGE, "-180,0,0.1,radians_as_degrees"), "set_limit_angle_lower", "get_limit_angle_lower");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "limit_angle/upper", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle_upper", "get_limit_angle_upper");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_gravity_damping_curve", "get_gravity_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), "set_gravity_direction", "get_gravity_direction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/falloff", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_gravity_falloff", "get_gravity_falloff");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_influence/value", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_wind_influence", "get_wind_influence");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_influence/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_wind_influence_damping_curve", "get_wind_influence_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_sync", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_wind_sync", "get_wind_sync");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_turbulence", PROPERTY_HINT_RANGE, "0,2,0.01"), "set_wind_turbulence", "get_wind_turbulence");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_frequency", PROPERTY_HINT_RANGE, "0,2,0.01"), "set_wind_frequency", "get_wind_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sync/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_sync", "get_sync");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "sync/bone_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Sync Bones,sync/bones/"), "set_sync_bone_count", "get_sync_bone_count");

	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "anchor", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_anchor", "get_anchor");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "anchor_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_anchor_inertia", "get_anchor_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia_smoothing", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia_smoothing", "get_inertia_smoothing");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_depth_inertia", "get_depth_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "centrifugal_acceleration", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_centrifugal_acceleration", "get_centrifugal_acceleration");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_local_movement_speed_limit", "get_local_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_local_rotation_speed_limit", "get_local_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_speed_limit", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m/s"), "set_joint_speed_limit", "get_joint_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shock_propagation", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_shock_propagation", "get_shock_propagation");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shock_threshold", PROPERTY_HINT_RANGE, "0,200,0.1,or_greater,suffix:m/s^2"), "set_shock_threshold", "get_shock_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "teleport_mode", PROPERTY_HINT_ENUM, "Keep,Reset"), "set_teleport_mode", "get_teleport_mode");

	ADD_GROUP("Root Spring", "root_spring_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_root_spring_stiffness", "get_root_spring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_limit", "get_root_spring_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_forward_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_forward_limit", "get_root_spring_forward_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_noise", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_root_spring_noise", "get_root_spring_noise");

	ADD_GROUP("Movement Limit", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_distance/value", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_max_distance", "get_max_distance");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "max_distance/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_max_distance_damping_curve", "get_max_distance_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "backstop/bone_name"), "set_backstop_bone_name", "get_backstop_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "backstop/bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_backstop_bone", "get_backstop_bone");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "backstop/position_offset", PROPERTY_HINT_NONE, "suffix:m"), "set_backstop_position_offset", "get_backstop_position_offset");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "backstop/distance", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_backstop_distance", "get_backstop_distance");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "backstop/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_backstop_damping_curve", "get_backstop_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "backstop/radius", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_backstop_radius", "get_backstop_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_limit_stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_movement_limit_stiffness", "get_movement_limit_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tether_limit", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_tether_limit", "get_tether_limit");

	ADD_GROUP("Simulation", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "ticks_per_second", PROPERTY_HINT_RANGE, "30,150,1,or_greater"), "set_ticks_per_second", "get_ticks_per_second");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_steps_per_frame", PROPERTY_HINT_RANGE, "1,8,1,or_greater"), "set_max_steps_per_frame", "get_max_steps_per_frame");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stabilization_time", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater,suffix:s"), "set_stabilization_time", "get_stabilization_time");

	ADD_GROUP("Culling", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "camera_culling", PROPERTY_HINT_ENUM, "Off,Reset,Keep"), "set_camera_culling", "get_camera_culling");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "distance_culling", PROPERTY_HINT_RANGE, "0,100,0.1,or_greater,suffix:m"), "set_distance_culling", "get_distance_culling");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "distance_culling_fade", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_distance_culling_fade", "get_distance_culling_fade");

	ADD_GROUP("", "");

	// godot-cpp에는 ADD_ARRAY_COUNT가 없어서 PROPERTY_USAGE_ARRAY로 대신한다.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), "set_chain_count", "get_chain_count");

	BIND_ENUM_CONSTANT(LINK_MODE_NONE);
	BIND_ENUM_CONSTANT(LINK_MODE_SEQUENTIAL);
	BIND_ENUM_CONSTANT(LINK_MODE_LOOP);
	BIND_ENUM_CONSTANT(CAMERA_CULLING_OFF);
	BIND_ENUM_CONSTANT(CAMERA_CULLING_RESET);
	BIND_ENUM_CONSTANT(CAMERA_CULLING_KEEP);
	BIND_ENUM_CONSTANT(LIMIT_ANGLE_MODE_CONE);
	BIND_ENUM_CONSTANT(LIMIT_ANGLE_MODE_PER_AXIS);
	BIND_ENUM_CONSTANT(TELEPORT_MODE_KEEP);
	BIND_ENUM_CONSTANT(TELEPORT_MODE_RESET);
	BIND_ENUM_CONSTANT(SYNC_DIRECTION_BOTH);
	BIND_ENUM_CONSTANT(SYNC_DIRECTION_POSITIVE);
	BIND_ENUM_CONSTANT(SYNC_DIRECTION_NEGATIVE);
	BIND_ENUM_CONSTANT(SYNC_DIRECTION_NONE);
}

void BoneClothSimulator3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint()) {
				set_notify_transform(true);
			}
		} break;
		case NOTIFICATION_CHILD_ORDER_CHANGED:
		case NOTIFICATION_TRANSFORM_CHANGED: {
			update_configuration_warnings();
		} break;
	}
}

bool BoneClothSimulator3D::_set(const StringName &p_name, const Variant &p_value) {
	const String path = p_name;
	if (path.begins_with("sync/bones/")) {
		const int which = path.get_slice("/", 2).to_int();
		const String what = path.get_slice("/", 3);
		ERR_FAIL_INDEX_V(which, int(sync_bones.size()), false);

		if (what == "bone_name") {
			set_sync_bone_name(which, p_value);
		} else if (what == "bone") {
			set_sync_bone(which, p_value);
		} else if (what.begins_with("direction_")) {
			set_sync_bone_direction(which, Vector3::Axis(what.unicode_at(10) - 'x'), SyncDirection(int(p_value)));
		} else if (what == "falloff_inner") {
			set_sync_bone_falloff_inner(which, p_value);
		} else if (what == "falloff_outer") {
			set_sync_bone_falloff_outer(which, p_value);
		} else if (what == "falloff_amount") {
			set_sync_bone_falloff_amount(which, p_value);
		} else {
			return false;
		}

		return true;
	}

	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone_name") {
		set_root_bone_name(which, p_value);
	} else if (what == "root_bone") {
		set_root_bone(which, p_value);
	} else if (what == "end_bone_name") {
		set_end_bone_name(which, p_value);
	} else if (what == "end_bone") {
		const String option = path.get_slice("/", 3);
		if (option.is_empty()) {
			set_end_bone(which, p_value);
		} else if (option == "direction") {
			set_end_bone_direction(which, BoneDirection(int(p_value)));
		} else if (option == "length") {
			set_end_bone_length(which, p_value);
		} else {
			return false;
		}
	} else if (what == "extend_end_bone") {
		set_extend_end_bone(which, p_value);
	} else if (what == "enable_all_child_collisions") {
		set_enable_all_child_collisions(which, p_value);
	} else if (what == "exclude_collision_count") {
		set_exclude_collision_count(which, p_value);
	} else if (what == "exclude_collisions") {
		set_exclude_collision_path(which, path.get_slice("/", 3).to_int(), p_value);
	} else if (what == "collision_count") {
		set_collision_count(which, p_value);
	} else if (what == "collisions") {
		set_collision_path(which, path.get_slice("/", 3).to_int(), p_value);
	} else {
		return false;
	}

	return true;
}

bool BoneClothSimulator3D::_get(const StringName &p_name, Variant &r_ret) const {
	const String path = p_name;
	if (path.begins_with("sync/bones/")) {
		const int which = path.get_slice("/", 2).to_int();
		const String what = path.get_slice("/", 3);
		ERR_FAIL_INDEX_V(which, int(sync_bones.size()), false);

		if (what == "bone_name") {
			r_ret = get_sync_bone_name(which);
		} else if (what == "bone") {
			r_ret = get_sync_bone(which);
		} else if (what.begins_with("direction_")) {
			r_ret = int(get_sync_bone_direction(which, Vector3::Axis(what.unicode_at(10) - 'x')));
		} else if (what == "falloff_inner") {
			r_ret = get_sync_bone_falloff_inner(which);
		} else if (what == "falloff_outer") {
			r_ret = get_sync_bone_falloff_outer(which);
		} else if (what == "falloff_amount") {
			r_ret = get_sync_bone_falloff_amount(which);
		} else {
			return false;
		}

		return true;
	}

	if (!path.begins_with("chains/")) {
		return false;
	}

	const int which = path.get_slice("/", 1).to_int();
	const String what = path.get_slice("/", 2);
	ERR_FAIL_INDEX_V(which, int(chains.size()), false);

	if (what == "root_bone_name") {
		r_ret = get_root_bone_name(which);
	} else if (what == "root_bone") {
		r_ret = get_root_bone(which);
	} else if (what == "end_bone_name") {
		r_ret = get_end_bone_name(which);
	} else if (what == "end_bone") {
		const String option = path.get_slice("/", 3);
		if (option.is_empty()) {
			r_ret = get_end_bone(which);
		} else if (option == "direction") {
			r_ret = int(get_end_bone_direction(which));
		} else if (option == "length") {
			r_ret = get_end_bone_length(which);
		} else {
			return false;
		}
	} else if (what == "extend_end_bone") {
		r_ret = is_end_bone_extended(which);
	} else if (what == "enable_all_child_collisions") {
		r_ret = are_all_child_collisions_enabled(which);
	} else if (what == "exclude_collision_count") {
		r_ret = get_exclude_collision_count(which);
	} else if (what == "exclude_collisions") {
		r_ret = get_exclude_collision_path(which, path.get_slice("/", 3).to_int());
	} else if (what == "collision_count") {
		r_ret = get_collision_count(which);
	} else if (what == "collisions") {
		r_ret = get_collision_path(which, path.get_slice("/", 3).to_int());
	} else {
		return false;
	}

	return true;
}

void BoneClothSimulator3D::_get_property_list(List<PropertyInfo> *p_list) const {
	Skeleton3D *skeleton = get_skeleton();
	const PropertyHint hint = skeleton ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE;
	const String bone_names = skeleton ? String(skeleton->get_concatenated_bone_names()) : String();

	for (uint32_t i = 0; i < sync_bones.size(); i++) {
		const String path = "sync/bones/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
		for (const String axis : { "x", "y", "z" }) {
			p_list->push_back(PropertyInfo(Variant::INT, path + String("direction_") + axis, PROPERTY_HINT_ENUM, "Both,Positive,Negative,None"));
		}
		const uint32_t falloff_usage = sync_bones[i].falloff_outer > 0.0f ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NO_EDITOR;
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + String("falloff_inner"), PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", falloff_usage));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + String("falloff_outer"), PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + String("falloff_amount"), PROPERTY_HINT_RANGE, "0,1,0.001", falloff_usage));
	}

	for (uint32_t i = 0; i < chains.size(); i++) {
		const String path = "chains/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("root_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("root_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("end_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("end_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));

		const bool has_end_bone = chains[i].end_bone != -1;
		const uint32_t extend_usage = has_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t tip_usage = has_end_bone && chains[i].extend_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + String("extend_end_bone"), PROPERTY_HINT_NONE, "", extend_usage));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("end_bone/direction"), PROPERTY_HINT_ENUM, "+X,-X,+Y,-Y,+Z,-Z,FromParent", tip_usage));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + String("end_bone/length"), PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", tip_usage));

		const Chain &chain = chains[i];
		const bool all = chain.enable_all_child_collisions;
		const uint32_t exclude_usage = all ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t collision_usage = all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + String("enable_all_child_collisions")));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("exclude_collision_count"), PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY : PROPERTY_USAGE_NONE, String("Exclude Collisions,") + path + String("exclude_collisions/")));
		for (uint32_t j = 0; j < chain.exclude_collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + String("exclude_collisions/") + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", exclude_usage));
		}
		p_list->push_back(PropertyInfo(Variant::INT, path + String("collision_count"), PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, String("Collisions,") + path + String("collisions/")));
		for (uint32_t j = 0; j < chain.collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + String("collisions/") + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", collision_usage));
		}
	}
}

void BoneClothSimulator3D::_validate_property(PropertyInfo &p_property) const {
	if (link_mode == LINK_MODE_NONE && (p_property.name == StringName("link_stiffness") || p_property.name == StringName("link_split") || p_property.name == StringName("link_bending") || String(p_property.name).begins_with("link_spacing/"))) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}

	const bool per_axis = limit_angle_mode == LIMIT_ANGLE_MODE_PER_AXIS;
	if ((per_axis && p_property.name == StringName("limit_angle/value")) || (!per_axis && (p_property.name == StringName("limit_angle/lower") || p_property.name == StringName("limit_angle/upper")))) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}

	if (p_property.name == StringName("backstop/bone_name")) {
		Skeleton3D *skeleton = get_skeleton();
		p_property.hint = skeleton ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE;
		p_property.hint_string = skeleton ? String(skeleton->get_concatenated_bone_names()) : String();
	} else if (backstop_bone_name.is_empty() && String(p_property.name).begins_with("backstop/")) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}

	if (anchor.is_empty() && p_property.name == StringName("anchor_inertia")) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void BoneClothSimulator3D::apply_preset(const Ref<BoneClothPreset> &p_preset) {
	ERR_FAIL_COND_MSG(p_preset.is_null(), "No preset to apply.");
	for (const String &name : BoneClothPreset::get_value_names()) {
		set(name, BoneClothPreset::copy_value(p_preset->get(name)));
	}
}

Ref<BoneClothPreset> BoneClothSimulator3D::make_preset() const {
	Ref<BoneClothPreset> made;
	made.instantiate();
	for (const String &name : BoneClothPreset::get_value_names()) {
		made->set(name, BoneClothPreset::copy_value(get(name)));
	}
	return made;
}

void BoneClothSimulator3D::set_chain_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	chains.resize(p_count);
	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_chain_count() const {
	return chains.size();
}

void BoneClothSimulator3D::set_root_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].root_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_root_bone(p_index, find_bone_or_warn(skeleton, p_bone_name, "Chain " + itos(p_index) + ": root bone"));
	}
}

String BoneClothSimulator3D::get_root_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].root_bone_name;
}

void BoneClothSimulator3D::set_root_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.root_bone = p_bone;
	read_bone_name(get_skeleton(), chain.root_bone, chain.root_bone_name, "Chain " + itos(p_index) + ": root bone");

	joints_dirty = true;
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_root_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].root_bone;
}

void BoneClothSimulator3D::set_end_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].end_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_end_bone(p_index, find_bone_or_warn(skeleton, p_bone_name, "Chain " + itos(p_index) + ": end bone"));
	}
}

String BoneClothSimulator3D::get_end_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].end_bone_name;
}

void BoneClothSimulator3D::set_end_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.end_bone = p_bone;
	read_bone_name(get_skeleton(), chain.end_bone, chain.end_bone_name, "Chain " + itos(p_index) + ": end bone");

	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_end_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].end_bone;
}

void BoneClothSimulator3D::set_extend_end_bone(int p_index, bool p_enabled) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].extend_end_bone = p_enabled;
	joints_dirty = true;
	notify_property_list_changed();
}

bool BoneClothSimulator3D::is_end_bone_extended(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	return chains[p_index].extend_end_bone;
}

void BoneClothSimulator3D::set_end_bone_direction(int p_index, BoneDirection p_bone_direction) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_INDEX(int(p_bone_direction), int(BONE_DIRECTION_FROM_PARENT) + 1);
	chains[p_index].end_bone_direction = p_bone_direction;
	joints_dirty = true;
}

SkeletonModifier3D::BoneDirection BoneClothSimulator3D::get_end_bone_direction(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), BONE_DIRECTION_FROM_PARENT);
	return chains[p_index].end_bone_direction;
}

void BoneClothSimulator3D::set_end_bone_length(int p_index, float p_length) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].end_bone_length = p_length;
	joints_dirty = true;
}

float BoneClothSimulator3D::get_end_bone_length(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0.0f);
	return chains[p_index].end_bone_length;
}

void BoneClothSimulator3D::set_enable_all_child_collisions(int p_index, bool p_enabled) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].enable_all_child_collisions = p_enabled;
	notify_property_list_changed();
	update_configuration_warnings();
}

bool BoneClothSimulator3D::are_all_child_collisions_enabled(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	return chains[p_index].enable_all_child_collisions;
}

void BoneClothSimulator3D::set_exclude_collision_count(int p_index, int p_count) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(!chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the exclude collision list is used only while enable_all_child_collisions is on.");
	ERR_FAIL_COND(p_count < 0);
	chains[p_index].exclude_collisions.resize(p_count);
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_exclude_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].exclude_collisions.size();
}

void BoneClothSimulator3D::set_exclude_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(!chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the exclude collision list is used only while enable_all_child_collisions is on.");
	ERR_FAIL_INDEX(p_collision, int(chains[p_index].exclude_collisions.size()));
	ERR_FAIL_COND_EDMSG(!_is_child_collision(p_node_path), "Collision must be a BoneClothCollision3D child of this BoneClothSimulator3D.");
	chains[p_index].exclude_collisions[p_collision] = p_node_path;
	update_configuration_warnings();
}

NodePath BoneClothSimulator3D::get_exclude_collision_path(int p_index, int p_collision) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), NodePath());
	ERR_FAIL_INDEX_V(p_collision, int(chains[p_index].exclude_collisions.size()), NodePath());
	return chains[p_index].exclude_collisions[p_collision];
}

void BoneClothSimulator3D::clear_exclude_collisions(int p_index) {
	set_exclude_collision_count(p_index, 0);
}

void BoneClothSimulator3D::set_collision_count(int p_index, int p_count) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the collision list is used only while enable_all_child_collisions is off.");
	ERR_FAIL_COND(p_count < 0);
	chains[p_index].collisions.resize(p_count);
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].collisions.size();
}

void BoneClothSimulator3D::set_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	ERR_FAIL_COND_MSG(chains[p_index].enable_all_child_collisions, "Chain " + itos(p_index) + ": the collision list is used only while enable_all_child_collisions is off.");
	ERR_FAIL_INDEX(p_collision, int(chains[p_index].collisions.size()));
	ERR_FAIL_COND_EDMSG(!_is_child_collision(p_node_path), "Collision must be a BoneClothCollision3D child of this BoneClothSimulator3D.");
	chains[p_index].collisions[p_collision] = p_node_path;
	update_configuration_warnings();
}

NodePath BoneClothSimulator3D::get_collision_path(int p_index, int p_collision) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), NodePath());
	ERR_FAIL_INDEX_V(p_collision, int(chains[p_index].collisions.size()), NodePath());
	return chains[p_index].collisions[p_collision];
}

void BoneClothSimulator3D::clear_collisions(int p_index) {
	set_collision_count(p_index, 0);
}

void BoneClothSimulator3D::set_link_mode(LinkMode p_mode) {
	link_mode = p_mode;
	joints_dirty = true;
	notify_property_list_changed();
}

BoneClothSimulator3D::LinkMode BoneClothSimulator3D::get_link_mode() const {
	return link_mode;
}

void BoneClothSimulator3D::set_link_stiffness(float p_stiffness) {
	link_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_link_stiffness() const {
	return link_stiffness;
}

void BoneClothSimulator3D::set_link_split_enabled(bool p_enabled) {
	link_split = p_enabled;
}

bool BoneClothSimulator3D::is_link_split_enabled() const {
	return link_split;
}

void BoneClothSimulator3D::set_link_spacing(float p_spacing) {
	link_spacing = p_spacing;
}

float BoneClothSimulator3D::get_link_spacing() const {
	return link_spacing;
}

void BoneClothSimulator3D::set_link_spacing_damping_curve(const Ref<Curve> &p_curve) {
	link_spacing_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_link_spacing_damping_curve() const {
	return link_spacing_damping_curve;
}

void BoneClothSimulator3D::set_link_bending(float p_bending) {
	link_bending = p_bending;
}

float BoneClothSimulator3D::get_link_bending() const {
	return link_bending;
}

void BoneClothSimulator3D::set_drag(float p_drag) {
	drag = p_drag;
}

float BoneClothSimulator3D::get_drag() const {
	return drag;
}

void BoneClothSimulator3D::set_drag_damping_curve(const Ref<Curve> &p_curve) {
	drag_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_drag_damping_curve() const {
	return drag_damping_curve;
}

void BoneClothSimulator3D::set_stiffness(float p_stiffness) {
	stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_stiffness() const {
	return stiffness;
}

void BoneClothSimulator3D::set_stiffness_damping_curve(const Ref<Curve> &p_curve) {
	stiffness_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_stiffness_damping_curve() const {
	return stiffness_damping_curve;
}

void BoneClothSimulator3D::set_stiffness_velocity_damping(float p_damping) {
	stiffness_velocity_damping = p_damping;
}

float BoneClothSimulator3D::get_stiffness_velocity_damping() const {
	return stiffness_velocity_damping;
}

void BoneClothSimulator3D::set_radius(float p_radius) {
	radius = p_radius;
}

float BoneClothSimulator3D::get_radius() const {
	return radius;
}

void BoneClothSimulator3D::set_radius_damping_curve(const Ref<Curve> &p_curve) {
	radius_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_radius_damping_curve() const {
	return radius_damping_curve;
}

void BoneClothSimulator3D::set_rest_overlap_enabled(bool p_enabled) {
	rest_overlap = p_enabled;
}

bool BoneClothSimulator3D::is_rest_overlap_enabled() const {
	return rest_overlap;
}

void BoneClothSimulator3D::set_collision_friction(float p_friction) {
	collision_friction = p_friction;
}

float BoneClothSimulator3D::get_collision_friction() const {
	return collision_friction;
}

void BoneClothSimulator3D::set_limit_angle(float p_angle) {
	limit_angle = p_angle;
}

float BoneClothSimulator3D::get_limit_angle() const {
	return limit_angle;
}

void BoneClothSimulator3D::set_limit_angle_damping_curve(const Ref<Curve> &p_curve) {
	limit_angle_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_limit_angle_damping_curve() const {
	return limit_angle_damping_curve;
}

void BoneClothSimulator3D::set_limit_angle_stiffness(float p_stiffness) {
	limit_angle_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_limit_angle_stiffness() const {
	return limit_angle_stiffness;
}

void BoneClothSimulator3D::set_limit_angle_mode(LimitAngleMode p_mode) {
	limit_angle_mode = p_mode;
	notify_property_list_changed();
}

BoneClothSimulator3D::LimitAngleMode BoneClothSimulator3D::get_limit_angle_mode() const {
	return limit_angle_mode;
}

void BoneClothSimulator3D::set_limit_angle_lower(const Vector3 &p_lower) {
	limit_angle_lower = p_lower;
}

Vector3 BoneClothSimulator3D::get_limit_angle_lower() const {
	return limit_angle_lower;
}

void BoneClothSimulator3D::set_limit_angle_upper(const Vector3 &p_upper) {
	limit_angle_upper = p_upper;
}

Vector3 BoneClothSimulator3D::get_limit_angle_upper() const {
	return limit_angle_upper;
}

void BoneClothSimulator3D::set_gravity(float p_gravity) {
	gravity = p_gravity;
}

float BoneClothSimulator3D::get_gravity() const {
	return gravity;
}

void BoneClothSimulator3D::set_gravity_damping_curve(const Ref<Curve> &p_curve) {
	gravity_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_gravity_damping_curve() const {
	return gravity_damping_curve;
}

void BoneClothSimulator3D::set_gravity_direction(const Vector3 &p_direction) {
	ERR_FAIL_COND(p_direction.is_zero_approx());
	gravity_direction = p_direction;
}

Vector3 BoneClothSimulator3D::get_gravity_direction() const {
	return gravity_direction;
}

void BoneClothSimulator3D::set_gravity_falloff(float p_falloff) {
	gravity_falloff = p_falloff;
}

float BoneClothSimulator3D::get_gravity_falloff() const {
	return gravity_falloff;
}

void BoneClothSimulator3D::set_wind_influence(float p_influence) {
	wind_influence = p_influence;
}

float BoneClothSimulator3D::get_wind_influence() const {
	return wind_influence;
}

void BoneClothSimulator3D::set_wind_influence_damping_curve(const Ref<Curve> &p_curve) {
	wind_influence_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_wind_influence_damping_curve() const {
	return wind_influence_damping_curve;
}

void BoneClothSimulator3D::set_wind_sync(float p_sync) {
	wind_sync = p_sync;
}

float BoneClothSimulator3D::get_wind_sync() const {
	return wind_sync;
}

void BoneClothSimulator3D::set_wind_turbulence(float p_turbulence) {
	wind_turbulence = p_turbulence;
}

float BoneClothSimulator3D::get_wind_turbulence() const {
	return wind_turbulence;
}

void BoneClothSimulator3D::set_wind_frequency(float p_frequency) {
	wind_frequency = p_frequency;
}

float BoneClothSimulator3D::get_wind_frequency() const {
	return wind_frequency;
}

void BoneClothSimulator3D::set_sync_bone_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	sync_bones.resize(p_count);
	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_sync_bone_count() const {
	return sync_bones.size();
}

void BoneClothSimulator3D::set_sync_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	sync_bones[p_index].bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_sync_bone(p_index, find_bone_or_warn(skeleton, p_bone_name, "Sync bone " + itos(p_index)));
	}
}

String BoneClothSimulator3D::get_sync_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), String());
	return sync_bones[p_index].bone_name;
}

void BoneClothSimulator3D::set_sync_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	SyncBone &sync_bone = sync_bones[p_index];
	sync_bone.bone = p_bone;
	read_bone_name(get_skeleton(), sync_bone.bone, sync_bone.bone_name, "Sync bone " + itos(p_index));

	joints_dirty = true;
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_sync_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), -1);
	return sync_bones[p_index].bone;
}

void BoneClothSimulator3D::set_sync_bone_direction(int p_index, Vector3::Axis p_axis, SyncDirection p_direction) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	ERR_FAIL_INDEX(int(p_axis), 3);
	sync_bones[p_index].directions[p_axis] = p_direction;
}

BoneClothSimulator3D::SyncDirection BoneClothSimulator3D::get_sync_bone_direction(int p_index, Vector3::Axis p_axis) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), SYNC_DIRECTION_BOTH);
	ERR_FAIL_INDEX_V(int(p_axis), 3, SYNC_DIRECTION_BOTH);
	return sync_bones[p_index].directions[p_axis];
}

void BoneClothSimulator3D::set_sync_bone_falloff_inner(int p_index, float p_radius) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	sync_bones[p_index].falloff_inner = p_radius;
}

float BoneClothSimulator3D::get_sync_bone_falloff_inner(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), 0.0f);
	return sync_bones[p_index].falloff_inner;
}

void BoneClothSimulator3D::set_sync_bone_falloff_outer(int p_index, float p_radius) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	const bool was_on = sync_bones[p_index].falloff_outer > 0.0f;
	sync_bones[p_index].falloff_outer = p_radius;
	if (was_on != (p_radius > 0.0f)) {
		notify_property_list_changed();
	}
}

float BoneClothSimulator3D::get_sync_bone_falloff_outer(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), 0.0f);
	return sync_bones[p_index].falloff_outer;
}

void BoneClothSimulator3D::set_sync_bone_falloff_amount(int p_index, float p_amount) {
	ERR_FAIL_INDEX(p_index, int(sync_bones.size()));
	sync_bones[p_index].falloff_amount = p_amount;
}

float BoneClothSimulator3D::get_sync_bone_falloff_amount(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(sync_bones.size()), 1.0f);
	return sync_bones[p_index].falloff_amount;
}

void BoneClothSimulator3D::set_sync(float p_sync) {
	sync = p_sync;
}

float BoneClothSimulator3D::get_sync() const {
	return sync;
}

void BoneClothSimulator3D::set_anchor(const NodePath &p_anchor) {
	anchor = p_anchor;
	has_prev_anchor = false;
	update_configuration_warnings();
	notify_property_list_changed();
}

NodePath BoneClothSimulator3D::get_anchor() const {
	return anchor;
}

void BoneClothSimulator3D::set_anchor_inertia(float p_inertia) {
	anchor_inertia = p_inertia;
}

float BoneClothSimulator3D::get_anchor_inertia() const {
	return anchor_inertia;
}

void BoneClothSimulator3D::set_inertia(float p_inertia) {
	inertia = p_inertia;
}

float BoneClothSimulator3D::get_inertia() const {
	return inertia;
}

void BoneClothSimulator3D::set_inertia_smoothing(float p_smoothing) {
	inertia_smoothing = p_smoothing;
}

float BoneClothSimulator3D::get_inertia_smoothing() const {
	return inertia_smoothing;
}

void BoneClothSimulator3D::set_depth_inertia(float p_depth_inertia) {
	depth_inertia = p_depth_inertia;
}

float BoneClothSimulator3D::get_depth_inertia() const {
	return depth_inertia;
}

void BoneClothSimulator3D::set_centrifugal_acceleration(float p_acceleration) {
	centrifugal_acceleration = p_acceleration;
}

float BoneClothSimulator3D::get_centrifugal_acceleration() const {
	return centrifugal_acceleration;
}

void BoneClothSimulator3D::set_local_movement_speed_limit(float p_limit) {
	local_movement_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_local_movement_speed_limit() const {
	return local_movement_speed_limit;
}

void BoneClothSimulator3D::set_local_rotation_speed_limit(float p_limit) {
	local_rotation_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_local_rotation_speed_limit() const {
	return local_rotation_speed_limit;
}

void BoneClothSimulator3D::set_joint_speed_limit(float p_limit) {
	joint_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_joint_speed_limit() const {
	return joint_speed_limit;
}

void BoneClothSimulator3D::set_movement_speed_limit(float p_limit) {
	movement_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_movement_speed_limit() const {
	return movement_speed_limit;
}

void BoneClothSimulator3D::set_rotation_speed_limit(float p_limit) {
	rotation_speed_limit = p_limit;
}

float BoneClothSimulator3D::get_rotation_speed_limit() const {
	return rotation_speed_limit;
}

void BoneClothSimulator3D::set_shock_propagation(float p_propagation) {
	shock_propagation = p_propagation;
}

float BoneClothSimulator3D::get_shock_propagation() const {
	return shock_propagation;
}

void BoneClothSimulator3D::set_shock_threshold(float p_threshold) {
	shock_threshold = p_threshold;
}

float BoneClothSimulator3D::get_shock_threshold() const {
	return shock_threshold;
}

void BoneClothSimulator3D::set_teleport_distance(float p_distance) {
	teleport_distance = p_distance;
}

float BoneClothSimulator3D::get_teleport_distance() const {
	return teleport_distance;
}

void BoneClothSimulator3D::set_teleport_angle(float p_angle) {
	teleport_angle = p_angle;
}

float BoneClothSimulator3D::get_teleport_angle() const {
	return teleport_angle;
}

void BoneClothSimulator3D::set_teleport_mode(TeleportMode p_mode) {
	teleport_mode = p_mode;
}

BoneClothSimulator3D::TeleportMode BoneClothSimulator3D::get_teleport_mode() const {
	return teleport_mode;
}

void BoneClothSimulator3D::set_root_spring_stiffness(float p_stiffness) {
	root_spring_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_root_spring_stiffness() const {
	return root_spring_stiffness;
}

void BoneClothSimulator3D::set_root_spring_limit(float p_limit) {
	root_spring_limit = p_limit;
}

float BoneClothSimulator3D::get_root_spring_limit() const {
	return root_spring_limit;
}

void BoneClothSimulator3D::set_root_spring_forward_limit(float p_limit) {
	root_spring_forward_limit = p_limit;
}

float BoneClothSimulator3D::get_root_spring_forward_limit() const {
	return root_spring_forward_limit;
}

void BoneClothSimulator3D::set_root_spring_noise(float p_noise) {
	root_spring_noise = p_noise;
}

float BoneClothSimulator3D::get_root_spring_noise() const {
	return root_spring_noise;
}

void BoneClothSimulator3D::set_max_distance(float p_distance) {
	max_distance = p_distance;
}

float BoneClothSimulator3D::get_max_distance() const {
	return max_distance;
}

void BoneClothSimulator3D::set_max_distance_damping_curve(const Ref<Curve> &p_curve) {
	max_distance_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_max_distance_damping_curve() const {
	return max_distance_damping_curve;
}

void BoneClothSimulator3D::set_backstop_bone_name(const String &p_bone_name) {
	backstop_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_backstop_bone(find_bone_or_warn(skeleton, p_bone_name, "Backstop bone"));
	}
	notify_property_list_changed();
}

String BoneClothSimulator3D::get_backstop_bone_name() const {
	return backstop_bone_name;
}

void BoneClothSimulator3D::set_backstop_bone(int p_bone) {
	backstop_bone = p_bone;
	read_bone_name(get_skeleton(), backstop_bone, backstop_bone_name, "Backstop bone");
	needs_reset = true;
	update_configuration_warnings();
	notify_property_list_changed();
}

int BoneClothSimulator3D::get_backstop_bone() const {
	return backstop_bone;
}

void BoneClothSimulator3D::set_backstop_position_offset(const Vector3 &p_offset) {
	backstop_position_offset = p_offset;
}

Vector3 BoneClothSimulator3D::get_backstop_position_offset() const {
	return backstop_position_offset;
}

void BoneClothSimulator3D::set_backstop_distance(float p_distance) {
	backstop_distance = p_distance;
}

float BoneClothSimulator3D::get_backstop_distance() const {
	return backstop_distance;
}

void BoneClothSimulator3D::set_backstop_damping_curve(const Ref<Curve> &p_curve) {
	backstop_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_backstop_damping_curve() const {
	return backstop_damping_curve;
}

void BoneClothSimulator3D::set_backstop_radius(float p_radius) {
	backstop_radius = p_radius;
}

float BoneClothSimulator3D::get_backstop_radius() const {
	return backstop_radius;
}

void BoneClothSimulator3D::set_movement_limit_stiffness(float p_stiffness) {
	movement_limit_stiffness = p_stiffness;
}

float BoneClothSimulator3D::get_movement_limit_stiffness() const {
	return movement_limit_stiffness;
}

void BoneClothSimulator3D::set_tether_limit(float p_limit) {
	tether_limit = p_limit;
}

float BoneClothSimulator3D::get_tether_limit() const {
	return tether_limit;
}

void BoneClothSimulator3D::set_ticks_per_second(int p_ticks) {
	ticks_per_second = MAX(p_ticks, 1);
	step_seconds = 1.0 / ticks_per_second;
	step_scale = 60.0f / ticks_per_second;
	step_time = 0.0;
}

int BoneClothSimulator3D::get_ticks_per_second() const {
	return ticks_per_second;
}

void BoneClothSimulator3D::set_max_steps_per_frame(int p_steps) {
	max_steps_per_frame = MAX(p_steps, 1);
}

int BoneClothSimulator3D::get_max_steps_per_frame() const {
	return max_steps_per_frame;
}

void BoneClothSimulator3D::set_stabilization_time(float p_seconds) {
	stabilization_time = MAX(p_seconds, 0.0f);
}

float BoneClothSimulator3D::get_stabilization_time() const {
	return stabilization_time;
}

void BoneClothSimulator3D::set_camera_culling(CameraCulling p_mode) {
	camera_culling = p_mode;
}

BoneClothSimulator3D::CameraCulling BoneClothSimulator3D::get_camera_culling() const {
	return camera_culling;
}

void BoneClothSimulator3D::set_distance_culling(float p_distance) {
	distance_culling = MAX(p_distance, 0.0f);
}

float BoneClothSimulator3D::get_distance_culling() const {
	return distance_culling;
}

void BoneClothSimulator3D::set_distance_culling_fade(float p_fade) {
	distance_culling_fade = CLAMP(p_fade, 0.0f, 1.0f);
}

float BoneClothSimulator3D::get_distance_culling_fade() const {
	return distance_culling_fade;
}

void BoneClothSimulator3D::reset() {
	needs_reset = true;
}

float BoneClothSimulator3D::get_units_per_meter() const {
	return units_per_meter;
}

static bool is_on_bone_path(const Skeleton3D *p_skeleton, int p_root, int p_end, int p_bone) {
	bool found = false;
	for (int bone = p_end; bone >= 0; bone = p_skeleton->get_bone_parent(bone)) {
		found = found || bone == p_bone;
		if (bone == p_root) {
			return found;
		}
	}

	return false;
}

PackedStringArray BoneClothSimulator3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	if (chains.is_empty()) {
		warnings.push_back("No chains. Add a chain under Chains and pick its root bone and end bone.");
	}

	const Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const Vector3 scale = skeleton->get_global_transform().basis.get_scale();
		if (!(Math::is_zero_approx(scale.x - scale.y) && Math::is_zero_approx(scale.y - scale.z))) {
			warnings.push_back(vformat("The Skeleton3D has a non-uniform scale %s, so the cloth moves differently. Make the scale of the Skeleton3D and its parents uniform (the same on all axes).", scale));
		}
	}

	for (uint32_t s = 0; s < sync_bones.size() && skeleton; s++) {
		if (!sync_bones[s].bone_name.is_empty() && sync_bones[s].bone < 0) {
			warnings.push_back(vformat("Sync bone %d: '%s' is not in the skeleton.", s, sync_bones[s].bone_name));
		}
	}

	if (!anchor.is_empty() && is_inside_tree() && !Object::cast_to<Node3D>(get_node_or_null(anchor))) {
		warnings.push_back(vformat("Anchor '%s' is not a Node3D.", anchor));
	}

	if (skeleton && !backstop_bone_name.is_empty() && backstop_bone < 0) {
		warnings.push_back(vformat("Backstop bone '%s' is not in the skeleton.", backstop_bone_name));
	}

	for (uint32_t c = 0; c < chains.size(); c++) {
		const Chain &chain = chains[c];
		PackedStringArray problems;
		if (skeleton) {
			const bool no_root = chain.root_bone_name.is_empty();
			const bool no_end = chain.end_bone_name.is_empty();
			if (no_root || no_end) {
				problems.push_back(vformat("pick %s", no_root && no_end ? "a root bone and an end bone" : (no_root ? "a root bone" : "an end bone")));
			}
			if (!no_root && chain.root_bone < 0) {
				problems.push_back(vformat("root bone '%s' is not in the skeleton", chain.root_bone_name));
			}
			if (!no_end && chain.end_bone < 0) {
				problems.push_back(vformat("end bone '%s' is not in the skeleton", chain.end_bone_name));
			}

			const bool has_path = is_on_bone_path(skeleton, chain.root_bone, chain.end_bone, chain.root_bone);
			if (chain.root_bone >= 0 && chain.end_bone >= 0 && !has_path) {
				problems.push_back(vformat("end bone '%s' is neither root bone '%s' nor under it", chain.end_bone_name, chain.root_bone_name));
			}

			bool has_trunk = false;
			for (uint32_t other = 0; other < c && has_path; other++) {
				has_trunk = has_trunk || is_on_bone_path(skeleton, chains[other].root_bone, chains[other].end_bone, chain.root_bone);
			}

			for (uint32_t other = c + 1; other < chains.size() && has_path && !has_trunk; other++) {
				const Chain &later = chains[other];
				if (later.root_bone != chain.root_bone && is_on_bone_path(skeleton, later.root_bone, later.end_bone, chain.root_bone)) {
					problems.push_back(vformat("root bone '%s' is on chain %d, so move chain %d above this chain", chain.root_bone_name, other, other));
					break;
				}
			}
		}

		const bool all = chain.enable_all_child_collisions;
		const LocalVector<NodePath> &node_paths = all ? chain.exclude_collisions : chain.collisions;
		for (const NodePath &node_path : node_paths) {
			if (!_is_child_collision(node_path)) {
				problems.push_back(vformat("%s '%s' is not a BoneClothCollision3D child of this node", all ? "exclude collision" : "collision", node_path));
			}
		}

		if (!problems.is_empty()) {
			warnings.push_back(vformat("Chain %d: %s.", c, String("; ").join(problems)));
		}
	}

	return warnings;
}

void BoneClothSimulator3D::_validate_bone_names() {
	for (uint32_t i = 0; i < chains.size(); i++) {
		const String root_name = chains[i].root_bone_name;
		if (!root_name.is_empty()) {
			set_root_bone_name(i, root_name);
		} else if (chains[i].root_bone != -1) {
			set_root_bone(i, chains[i].root_bone);
		}

		const String end_name = chains[i].end_bone_name;
		if (!end_name.is_empty()) {
			set_end_bone_name(i, end_name);
		} else if (chains[i].end_bone != -1) {
			set_end_bone(i, chains[i].end_bone);
		}
	}

	for (uint32_t i = 0; i < sync_bones.size(); i++) {
		const String sync_name = sync_bones[i].bone_name;
		if (!sync_name.is_empty()) {
			set_sync_bone_name(i, sync_name);
		} else if (sync_bones[i].bone != -1) {
			set_sync_bone(i, sync_bones[i].bone);
		}
	}

	const String backstop_name = backstop_bone_name;
	if (!backstop_name.is_empty()) {
		set_backstop_bone_name(backstop_name);
	} else if (backstop_bone != -1) {
		set_backstop_bone(backstop_bone);
	}
}

// 모델 노드에 준 scale은 빼고 계산해야, 모델을 키울 때 옷과 머리카락도 같이 커진다.
float BoneClothSimulator3D::_get_units_per_meter(const Skeleton3D *p_skeleton) const {
	const Node *owner = p_skeleton->get_owner();
	Basis basis;
	for (const Node3D *node = p_skeleton; node && node != owner; node = Object::cast_to<Node3D>(node->get_parent())) {
		basis = node->get_basis() * basis;
	}

	return 1.0f / MAX(basis.get_uniform_scale(), 1e-6f);
}

void BoneClothSimulator3D::_process_modification_with_delta(double p_delta) {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	_rebuild_if_changed(skeleton);
	if (!_update_culling(skeleton)) {
		culled = true;
		monitor_time.sum.fetch_add(std::chrono::nanoseconds(std::chrono::steady_clock::now() - start).count(), std::memory_order_relaxed);
		return;
	}
	const bool resumed = culled;
	culled = false;

	_read_frame(skeleton);

	const Transform3D skeleton_transform = skeleton->get_global_transform_interpolated();
	if (needs_reset) {
		_reset(skeleton_transform);
	} else if (resumed) {
		_reset_motion(skeleton_transform);
	}

	LocalVector<PosedCollision> collisions;
	_gather_collisions(collisions);
	_find_chain_collisions(collisions);

	if (p_delta > 0.0) {
		_follow_anchor();
		if (teleport_mode == TELEPORT_MODE_RESET && _is_teleported(skeleton_transform)) {
			_reset(skeleton_transform);
		}
		FrameMotion motion = _measure_motion(p_delta, skeleton_transform);
		_run_steps(motion, collisions);
		_settle_display(p_delta, motion, collisions);
	} else {
		for (Chain &chain : chains) {
			if (chain.joints.is_empty()) {
				continue;
			}
			const Vector3 shift = _get_root_display(chain) - chain.joints[0].display_location;
			for (Joint &joint : chain.joints) {
				joint.display_location += shift;
			}
		}
	}

	_end_frame(skeleton);

	uint64_t joint_count = 0;
	for (const Chain &chain : chains) {
		if (chain.same_as < 0) {
			joint_count += chain.joints.size();
		}
	}

	monitor_joints.sum.fetch_add(joint_count, std::memory_order_relaxed);
	monitor_time.sum.fetch_add(std::chrono::nanoseconds(std::chrono::steady_clock::now() - start).count(), std::memory_order_relaxed);
}

void BoneClothSimulator3D::add_monitors() {
	Performance *performance = Performance::get_singleton();
	performance->add_custom_monitor("bone_cloth/time", callable_mp_static(&get_monitor_time), Array(), Performance::MONITOR_TYPE_TIME);
	performance->add_custom_monitor("bone_cloth/joints", callable_mp_static(&get_monitor_joints));
}

void BoneClothSimulator3D::remove_monitors() {
	Performance *performance = Performance::get_singleton();
	performance->remove_custom_monitor("bone_cloth/time");
	performance->remove_custom_monitor("bone_cloth/joints");
}

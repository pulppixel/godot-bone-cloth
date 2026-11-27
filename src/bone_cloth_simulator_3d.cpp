#include "bone_cloth_simulator_3d.h"

#include "bone_cloth_bone_lookup.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
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

#define BIND_PROPERTY(m_info, m_name, m_arg) \
	ClassDB::bind_method(D_METHOD("set_" #m_name, m_arg), &BoneClothSimulator3D::set_##m_name); \
	ClassDB::bind_method(D_METHOD("get_" #m_name), &BoneClothSimulator3D::get_##m_name); \
	ADD_PROPERTY(m_info, "set_" #m_name, "get_" #m_name)

void BoneClothSimulator3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("apply_preset", "preset"), &BoneClothSimulator3D::apply_preset);
	ClassDB::bind_method(D_METHOD("make_preset"), &BoneClothSimulator3D::make_preset);

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

	ClassDB::bind_method(D_METHOD("set_link_split_enabled", "enabled"), &BoneClothSimulator3D::set_link_split_enabled);
	ClassDB::bind_method(D_METHOD("is_link_split_enabled"), &BoneClothSimulator3D::is_link_split_enabled);

	ClassDB::bind_method(D_METHOD("set_rest_overlap_enabled", "enabled"), &BoneClothSimulator3D::set_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("is_rest_overlap_enabled"), &BoneClothSimulator3D::is_rest_overlap_enabled);
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

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);
	ClassDB::bind_method(D_METHOD("set_mesh_fixed_bone", "index", "bone_name"), &BoneClothSimulator3D::set_mesh_fixed_bone);
	ClassDB::bind_method(D_METHOD("get_mesh_fixed_bone", "index"), &BoneClothSimulator3D::get_mesh_fixed_bone);

	ADD_GROUP("Mesh", "mesh_");
	BIND_PROPERTY(PropertyInfo(Variant::NODE_PATH, "mesh_instance", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MeshInstance3D"), mesh_instance, "path");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "mesh_surfaces", PROPERTY_HINT_FLAGS), mesh_surfaces, "surfaces");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_point_spacing", PROPERTY_HINT_RANGE, "0.01,0.2,0.001,or_greater,suffix:m"), mesh_point_spacing, "spacing");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "mesh_fixed_bone_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Fixed Bones,mesh_fixed_bones/"), mesh_fixed_bone_count, "count");
	ADD_GROUP("", "");

	// godot-cpp에는 ADD_ARRAY_COUNT가 없어서 PROPERTY_USAGE_ARRAY로 대신한다.
	BIND_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), chain_count, "count");

	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), stiffness, "stiffness");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), stiffness_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/velocity_damping", PROPERTY_HINT_RANGE, "0,1,0.01"), stiffness_velocity_damping, "damping");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), drag, "drag");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), drag_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), gravity, "gravity");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), gravity_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), gravity_direction, "direction");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/falloff", PROPERTY_HINT_RANGE, "0,1,0.01"), gravity_falloff, "falloff");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), limit_angle, "angle");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), limit_angle_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), limit_angle_stiffness, "stiffness");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "limit_angle/mode", PROPERTY_HINT_ENUM, "Cone,Per Axis"), limit_angle_mode, "mode");
	BIND_PROPERTY(PropertyInfo(Variant::VECTOR3, "limit_angle/lower", PROPERTY_HINT_RANGE, "-180,0,0.1,radians_as_degrees"), limit_angle_lower, "lower");
	BIND_PROPERTY(PropertyInfo(Variant::VECTOR3, "limit_angle/upper", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), limit_angle_upper, "upper");

	ADD_GROUP("Collision", "");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), radius, "radius");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), radius_damping_curve, "curve");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "radius/rest_overlap"), "set_rest_overlap_enabled", "is_rest_overlap_enabled");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_friction", PROPERTY_HINT_RANGE, "0,0.5,0.001"), collision_friction, "friction");

	ADD_GROUP("Link", "link_");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), link_mode, "mode");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), link_stiffness, "stiffness");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stretch_limit", PROPERTY_HINT_RANGE, "0,3,0.01,or_greater"), link_stretch_limit, "limit");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "link_split"), "set_link_split_enabled", "is_link_split_enabled");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "link_spacing/value", PROPERTY_HINT_RANGE, "0,1,0.001"), link_spacing, "spacing");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "link_spacing/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), link_spacing_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "link_bending", PROPERTY_HINT_RANGE, "0,1,0.001"), link_bending, "bending");

	ADD_GROUP("Wind", "wind_");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_influence/value", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), wind_influence, "influence");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_influence/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), wind_influence_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_sync", PROPERTY_HINT_RANGE, "0,1,0.01"), wind_sync, "sync");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_turbulence", PROPERTY_HINT_RANGE, "0,2,0.01"), wind_turbulence, "turbulence");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_frequency", PROPERTY_HINT_RANGE, "0,2,0.01"), wind_frequency, "frequency");

	ADD_GROUP("", "");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "sync/value", PROPERTY_HINT_RANGE, "0,1,0.001"), sync, "sync");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "sync/bone_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Sync Bones,sync/bones/"), sync_bone_count, "count");

	ADD_GROUP("Inertia", "");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), inertia, "inertia");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia_smoothing", PROPERTY_HINT_RANGE, "0,1,0.001"), inertia_smoothing, "smoothing");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), depth_inertia, "depth_inertia");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "centrifugal_acceleration", PROPERTY_HINT_RANGE, "0,1,0.001"), centrifugal_acceleration, "acceleration");
	ADD_SUBGROUP("Anchor", "");
	BIND_PROPERTY(PropertyInfo(Variant::NODE_PATH, "anchor", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), anchor, "anchor");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "anchor_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), anchor_inertia, "inertia");
	ADD_SUBGROUP("Speed Limits", "");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "local_movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), local_movement_speed_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "local_rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), local_rotation_speed_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), movement_speed_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), rotation_speed_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_speed_limit", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m/s"), joint_speed_limit, "limit");
	ADD_SUBGROUP("Shock", "shock_");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "shock_propagation", PROPERTY_HINT_RANGE, "0,1,0.001"), shock_propagation, "propagation");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "shock_threshold", PROPERTY_HINT_RANGE, "0,200,0.1,or_greater,suffix:m/s^2"), shock_threshold, "threshold");
	ADD_SUBGROUP("Teleport", "teleport_");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), teleport_distance, "distance");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), teleport_angle, "angle");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "teleport_mode", PROPERTY_HINT_ENUM, "Keep,Reset"), teleport_mode, "mode");

	ADD_GROUP("Root Spring", "root_spring_");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), root_spring_stiffness, "stiffness");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), root_spring_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_forward_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), root_spring_forward_limit, "limit");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_noise", PROPERTY_HINT_RANGE, "0,1,0.001"), root_spring_noise, "noise");

	ADD_GROUP("Movement Limit", "");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "max_distance/value", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), max_distance, "distance");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "max_distance/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), max_distance_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::STRING, "backstop/bone_name"), backstop_bone_name, "bone_name");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "backstop/bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), backstop_bone, "bone");
	BIND_PROPERTY(PropertyInfo(Variant::VECTOR3, "backstop/position_offset", PROPERTY_HINT_NONE, "suffix:m"), backstop_position_offset, "offset");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "backstop/distance", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), backstop_distance, "distance");
	BIND_PROPERTY(PropertyInfo(Variant::OBJECT, "backstop/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), backstop_damping_curve, "curve");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "backstop/radius", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), backstop_radius, "radius");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_limit_stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), movement_limit_stiffness, "stiffness");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "tether_limit", PROPERTY_HINT_RANGE, "0,1,0.01"), tether_limit, "limit");

	ADD_GROUP("Simulation", "");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "ticks_per_second", PROPERTY_HINT_RANGE, "30,150,1,or_greater"), ticks_per_second, "ticks");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "max_steps_per_frame", PROPERTY_HINT_RANGE, "1,8,1,or_greater"), max_steps_per_frame, "steps");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "stabilization_time", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater,suffix:s"), stabilization_time, "seconds");

	ADD_GROUP("Culling", "");
	BIND_PROPERTY(PropertyInfo(Variant::INT, "camera_culling", PROPERTY_HINT_ENUM, "Off,Reset,Keep"), camera_culling, "mode");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "distance_culling", PROPERTY_HINT_RANGE, "0,100,0.1,or_greater,suffix:m"), distance_culling, "distance");
	BIND_PROPERTY(PropertyInfo(Variant::FLOAT, "distance_culling_fade", PROPERTY_HINT_RANGE, "0,1,0.01"), distance_culling_fade, "fade");

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
			if (!mesh_instance.is_empty()) {
				joints_dirty = true;
			}
		} break;
		case NOTIFICATION_EXIT_TREE: {
			_clear_mesh_copy();
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (!is_active() && mesh_copy_shown) {
				_show_mesh_copy(false);
				needs_reset = true;
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
	if (path.begins_with("mesh_fixed_bones/")) {
		const int which = path.get_slice("/", 1).to_int();
		ERR_FAIL_INDEX_V(which, int(mesh_fixed_bones.size()), false);
		set_mesh_fixed_bone(which, p_value);
		return true;
	}

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
	if (path.begins_with("mesh_fixed_bones/")) {
		const int which = path.get_slice("/", 1).to_int();
		ERR_FAIL_INDEX_V(which, int(mesh_fixed_bones.size()), false);
		r_ret = get_mesh_fixed_bone(which);
		return true;
	}

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

	for (uint32_t i = 0; i < mesh_fixed_bones.size(); i++) {
		p_list->push_back(PropertyInfo(Variant::STRING, "mesh_fixed_bones/" + itos(i), hint, bone_names));
	}

	for (uint32_t i = 0; i < sync_bones.size(); i++) {
		String path = "sync/bones/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + "bone_name", hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + "bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
		for (const String axis : { "x", "y", "z" }) {
			p_list->push_back(PropertyInfo(Variant::INT, path + "direction_" + axis, PROPERTY_HINT_ENUM, "Both,Positive,Negative,None"));
		}
		const uint32_t falloff_usage = sync_bones[i].falloff_outer > 0.0f ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NO_EDITOR;
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + "falloff_inner", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", falloff_usage));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + "falloff_outer", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + "falloff_amount", PROPERTY_HINT_RANGE, "0,1,0.001", falloff_usage));
	}

	for (uint32_t i = 0; i < chains.size() && mesh_instance.is_empty(); i++) {
		String path = "chains/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::STRING, path + "root_bone_name", hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + "root_bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));
		p_list->push_back(PropertyInfo(Variant::STRING, path + "end_bone_name", hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + "end_bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));

		const bool has_end_bone = chains[i].end_bone != -1;
		const uint32_t extend_usage = has_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t tip_usage = has_end_bone && chains[i].extend_end_bone ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + "extend_end_bone", PROPERTY_HINT_NONE, "", extend_usage));
		p_list->push_back(PropertyInfo(Variant::INT, path + "end_bone/direction", PROPERTY_HINT_ENUM, "+X,-X,+Y,-Y,+Z,-Z,FromParent", tip_usage));
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + "end_bone/length", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m", tip_usage));

		const Chain &chain = chains[i];
		const bool all = chain.enable_all_child_collisions;
		const uint32_t exclude_usage = all ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		const uint32_t collision_usage = all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::BOOL, path + "enable_all_child_collisions"));
		p_list->push_back(PropertyInfo(Variant::INT, path + "exclude_collision_count", PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY : PROPERTY_USAGE_NONE, "Exclude Collisions," + path + "exclude_collisions/"));
		for (uint32_t j = 0; j < chain.exclude_collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + "exclude_collisions/" + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", exclude_usage));
		}
		p_list->push_back(PropertyInfo(Variant::INT, path + "collision_count", PROPERTY_HINT_NONE, "", all ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Collisions," + path + "collisions/"));
		for (uint32_t j = 0; j < chain.collisions.size(); j++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + "collisions/" + itos(j), PROPERTY_HINT_NODE_PATH_VALID_TYPES, "BoneClothCollision3D", collision_usage));
		}
	}
}

void BoneClothSimulator3D::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == StringName("mesh_surfaces")) {
		const MeshInstance3D *mesh_node = is_inside_tree() ? Object::cast_to<MeshInstance3D>(get_node_or_null(mesh_instance)) : nullptr;
		const Ref<ArrayMesh> mesh = mesh_node ? mesh_node->get_mesh() : Ref<ArrayMesh>();
		PackedStringArray names;
		for (int s = 0; mesh.is_valid() && s < MIN(mesh->get_surface_count(), 32); s++) {
			const String name = mesh->surface_get_name(s);
			names.push_back(name.is_empty() ? vformat("Surface %d", s) : name.replace(",", " ").replace(":", " "));
		}
		p_property.hint_string = String(",").join(names);
		if (mesh_instance.is_empty()) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	}

	if (!mesh_instance.is_empty() && p_property.name == StringName("chain_count")) {
		p_property.usage = PROPERTY_USAGE_NONE;
	}

	if (link_mode == LINK_MODE_NONE && (p_property.name == StringName("link_stiffness") || p_property.name == StringName("link_stretch_limit") || p_property.name == StringName("link_split") || p_property.name == StringName("link_bending") || String(p_property.name).begins_with("link_spacing/"))) {
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

void BoneClothSimulator3D::set_mesh_instance(const NodePath &p_path) {
	if (p_path == mesh_instance) {
		return;
	}
	mesh_instance = p_path;
	chains.clear();
	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

NodePath BoneClothSimulator3D::get_mesh_instance() const {
	return mesh_instance;
}

void BoneClothSimulator3D::set_mesh_surfaces(int64_t p_surfaces) {
	mesh_surfaces = p_surfaces;
	joints_dirty = true;
	update_configuration_warnings();
}

int64_t BoneClothSimulator3D::get_mesh_surfaces() const {
	return mesh_surfaces;
}

void BoneClothSimulator3D::set_mesh_point_spacing(float p_spacing) {
	mesh_point_spacing = MAX(p_spacing, 0.001f);
	joints_dirty = true;
}

float BoneClothSimulator3D::get_mesh_point_spacing() const {
	return mesh_point_spacing;
}

void BoneClothSimulator3D::set_mesh_fixed_bone_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	mesh_fixed_bones.resize(p_count);
	joints_dirty = true;
	notify_property_list_changed();
	update_configuration_warnings();
}

int BoneClothSimulator3D::get_mesh_fixed_bone_count() const {
	return mesh_fixed_bones.size();
}

void BoneClothSimulator3D::set_mesh_fixed_bone(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(mesh_fixed_bones.size()));
	mesh_fixed_bones[p_index] = p_bone_name;
	joints_dirty = true;
	update_configuration_warnings();
}

String BoneClothSimulator3D::get_mesh_fixed_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(mesh_fixed_bones.size()), String());
	return mesh_fixed_bones[p_index];
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
	_build_same_chains();
	notify_property_list_changed();
	update_configuration_warnings();
}

bool BoneClothSimulator3D::are_all_child_collisions_enabled(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	return chains[p_index].enable_all_child_collisions;
}

bool BoneClothSimulator3D::_can_edit_collisions(int p_index, bool p_exclude) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), false);
	ERR_FAIL_COND_V_MSG(chains[p_index].enable_all_child_collisions != p_exclude, false, vformat("Chain %d: the %s list is used only while enable_all_child_collisions is %s.", p_index, p_exclude ? "exclude collision" : "collision", p_exclude ? "on" : "off"));
	return true;
}

void BoneClothSimulator3D::_set_collision_count(int p_index, int p_count, bool p_exclude) {
	ERR_FAIL_COND(p_count < 0 || !_can_edit_collisions(p_index, p_exclude));
	(chains[p_index].*(p_exclude ? &Chain::exclude_collisions : &Chain::collisions)).resize(p_count);
	_build_same_chains();
	notify_property_list_changed();
	update_configuration_warnings();
}

void BoneClothSimulator3D::_set_collision_path(int p_index, int p_collision, const NodePath &p_node_path, bool p_exclude) {
	ERR_FAIL_COND(!_can_edit_collisions(p_index, p_exclude));
	LocalVector<NodePath> &paths = chains[p_index].*(p_exclude ? &Chain::exclude_collisions : &Chain::collisions);
	ERR_FAIL_INDEX(p_collision, int(paths.size()));
	ERR_FAIL_COND_EDMSG(!_is_child_collision(p_node_path), "Collision must be a BoneClothCollision3D child of this BoneClothSimulator3D.");
	paths[p_collision] = p_node_path;
	_build_same_chains();
	update_configuration_warnings();
}

void BoneClothSimulator3D::set_exclude_collision_count(int p_index, int p_count) {
	_set_collision_count(p_index, p_count, true);
}

int BoneClothSimulator3D::get_exclude_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].exclude_collisions.size();
}

void BoneClothSimulator3D::set_exclude_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	_set_collision_path(p_index, p_collision, p_node_path, true);
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
	_set_collision_count(p_index, p_count, false);
}

int BoneClothSimulator3D::get_collision_count(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), 0);
	return chains[p_index].collisions.size();
}

void BoneClothSimulator3D::set_collision_path(int p_index, int p_collision, const NodePath &p_node_path) {
	_set_collision_path(p_index, p_collision, p_node_path, false);
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

void BoneClothSimulator3D::set_link_split_enabled(bool p_enabled) {
	link_split = p_enabled;
}

bool BoneClothSimulator3D::is_link_split_enabled() const {
	return link_split;
}

void BoneClothSimulator3D::set_rest_overlap_enabled(bool p_enabled) {
	rest_overlap = p_enabled;
}

bool BoneClothSimulator3D::is_rest_overlap_enabled() const {
	return rest_overlap;
}

void BoneClothSimulator3D::set_limit_angle_mode(LimitAngleMode p_mode) {
	limit_angle_mode = p_mode;
	notify_property_list_changed();
}

BoneClothSimulator3D::LimitAngleMode BoneClothSimulator3D::get_limit_angle_mode() const {
	return limit_angle_mode;
}

void BoneClothSimulator3D::set_gravity_direction(const Vector3 &p_direction) {
	ERR_FAIL_COND(p_direction.is_zero_approx());
	gravity_direction = p_direction;
}

Vector3 BoneClothSimulator3D::get_gravity_direction() const {
	return gravity_direction;
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

void BoneClothSimulator3D::set_anchor(const NodePath &p_anchor) {
	anchor = p_anchor;
	has_prev_anchor = false;
	update_configuration_warnings();
	notify_property_list_changed();
}

NodePath BoneClothSimulator3D::get_anchor() const {
	return anchor;
}

void BoneClothSimulator3D::set_backstop_bone_name(const String &p_bone_name) {
	backstop_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_backstop_bone(find_bone_or_warn(skeleton, p_bone_name, "Backstop bone"));
	} else {
		notify_property_list_changed();
	}
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
	if (chains.is_empty() && mesh_instance.is_empty()) {
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

	if (!mesh_instance.is_empty() && is_inside_tree()) {
		const MeshInstance3D *mesh_node = Object::cast_to<MeshInstance3D>(get_node_or_null(mesh_instance));
		if (!mesh_node || mesh_node->get_mesh().is_null() || mesh_node->get_skin().is_null()) {
			warnings.push_back(vformat("Mesh '%s' is not a MeshInstance3D with a mesh and a skin.", mesh_instance));
		}
		for (int s = 0; mesh_node && mesh_node->get_mesh().is_valid() && s < 64; s++) {
			if (((mesh_surfaces >> s) & 1) && s >= mesh_node->get_mesh()->get_surface_count()) {
				warnings.push_back(vformat("Surface %d is not in the mesh.", s));
			}
		}
		if (mesh_fixed_bones.is_empty()) {
			warnings.push_back("No fixed bones. Add the bones the mesh hangs from under Fixed Bones, such as the hips.");
		}
		for (uint32_t i = 0; i < mesh_fixed_bones.size() && skeleton; i++) {
			if (skeleton->find_bone(mesh_fixed_bones[i]) < 0) {
				warnings.push_back(vformat("Fixed bone %d: '%s' is not in the skeleton.", i, mesh_fixed_bones[i]));
			}
		}
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

void BoneClothSimulator3D::_make_joints_dirty() {
	joints_dirty = true;
}

void BoneClothSimulator3D::_skeleton_changed(Skeleton3D *p_old_skeleton, Skeleton3D *p_new_skeleton) {
	const Callable dirty = callable_mp(this, &BoneClothSimulator3D::_make_joints_dirty);
	if (p_old_skeleton && p_old_skeleton->is_connected("rest_updated", dirty)) {
		p_old_skeleton->disconnect("rest_updated", dirty);
	}
	if (p_new_skeleton && !p_new_skeleton->is_connected("rest_updated", dirty)) {
		p_new_skeleton->connect("rest_updated", dirty);
	}
	joints_dirty = true;
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
	const bool culling = !_update_culling(skeleton);
	if (!mesh_instance.is_empty() && mesh_copy_shown == culling) {
		_show_mesh_copy(!culling);
	}
	if (culling) {
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
		bool teleported = _is_teleported(skeleton_transform);
		if (teleported && teleport_mode == TELEPORT_MODE_RESET) {
			_reset(skeleton_transform);
			_follow_anchor();
			teleported = false;
		}
		FrameMotion motion = _measure_motion(p_delta, skeleton_transform, teleported);
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

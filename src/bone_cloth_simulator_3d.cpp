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

	ClassDB::bind_method(D_METHOD("set_sync_bone_name", "index", "bone_name"), &BoneClothSimulator3D::set_sync_bone_name);
	ClassDB::bind_method(D_METHOD("get_sync_bone_name", "index"), &BoneClothSimulator3D::get_sync_bone_name);
	ClassDB::bind_method(D_METHOD("set_sync_bone", "index", "bone"), &BoneClothSimulator3D::set_sync_bone);
	ClassDB::bind_method(D_METHOD("get_sync_bone", "index"), &BoneClothSimulator3D::get_sync_bone);

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

	ClassDB::bind_method(D_METHOD("set_drag", "drag"), &BoneClothSimulator3D::set_drag);
	ClassDB::bind_method(D_METHOD("get_drag"), &BoneClothSimulator3D::get_drag);
	ClassDB::bind_method(D_METHOD("set_drag_damping_curve", "curve"), &BoneClothSimulator3D::set_drag_damping_curve);
	ClassDB::bind_method(D_METHOD("get_drag_damping_curve"), &BoneClothSimulator3D::get_drag_damping_curve);

	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothSimulator3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothSimulator3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_stiffness_damping_curve", "curve"), &BoneClothSimulator3D::set_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("get_stiffness_damping_curve"), &BoneClothSimulator3D::get_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothSimulator3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothSimulator3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius_damping_curve", "curve"), &BoneClothSimulator3D::set_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("get_radius_damping_curve"), &BoneClothSimulator3D::get_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("set_rest_overlap_enabled", "enabled"), &BoneClothSimulator3D::set_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("is_rest_overlap_enabled"), &BoneClothSimulator3D::is_rest_overlap_enabled);
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothSimulator3D::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothSimulator3D::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_limit_angle_damping_curve", "curve"), &BoneClothSimulator3D::set_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("get_limit_angle_damping_curve"), &BoneClothSimulator3D::get_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("set_limit_angle_stiffness", "stiffness"), &BoneClothSimulator3D::set_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("get_limit_angle_stiffness"), &BoneClothSimulator3D::get_limit_angle_stiffness);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothSimulator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothSimulator3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity_damping_curve", "curve"), &BoneClothSimulator3D::set_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("get_gravity_damping_curve"), &BoneClothSimulator3D::get_gravity_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity_direction", "direction"), &BoneClothSimulator3D::set_gravity_direction);
	ClassDB::bind_method(D_METHOD("get_gravity_direction"), &BoneClothSimulator3D::get_gravity_direction);
	ClassDB::bind_method(D_METHOD("set_sync", "sync"), &BoneClothSimulator3D::set_sync);
	ClassDB::bind_method(D_METHOD("get_sync"), &BoneClothSimulator3D::get_sync);
	ClassDB::bind_method(D_METHOD("set_sync_damping_curve", "curve"), &BoneClothSimulator3D::set_sync_damping_curve);
	ClassDB::bind_method(D_METHOD("get_sync_damping_curve"), &BoneClothSimulator3D::get_sync_damping_curve);

	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothSimulator3D::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothSimulator3D::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia_smoothing", "smoothing"), &BoneClothSimulator3D::set_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("get_inertia_smoothing"), &BoneClothSimulator3D::get_inertia_smoothing);
	ClassDB::bind_method(D_METHOD("set_depth_inertia", "depth_inertia"), &BoneClothSimulator3D::set_depth_inertia);
	ClassDB::bind_method(D_METHOD("get_depth_inertia"), &BoneClothSimulator3D::get_depth_inertia);
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
	ClassDB::bind_method(D_METHOD("set_teleport_distance", "distance"), &BoneClothSimulator3D::set_teleport_distance);
	ClassDB::bind_method(D_METHOD("get_teleport_distance"), &BoneClothSimulator3D::get_teleport_distance);
	ClassDB::bind_method(D_METHOD("set_teleport_angle", "angle"), &BoneClothSimulator3D::set_teleport_angle);
	ClassDB::bind_method(D_METHOD("get_teleport_angle"), &BoneClothSimulator3D::get_teleport_angle);
	ClassDB::bind_method(D_METHOD("set_root_spring_stiffness", "stiffness"), &BoneClothSimulator3D::set_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("get_root_spring_stiffness"), &BoneClothSimulator3D::get_root_spring_stiffness);
	ClassDB::bind_method(D_METHOD("set_root_spring_limit", "limit"), &BoneClothSimulator3D::set_root_spring_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_limit"), &BoneClothSimulator3D::get_root_spring_limit);
	ClassDB::bind_method(D_METHOD("set_root_spring_forward_limit", "limit"), &BoneClothSimulator3D::set_root_spring_forward_limit);
	ClassDB::bind_method(D_METHOD("get_root_spring_forward_limit"), &BoneClothSimulator3D::get_root_spring_forward_limit);

	ClassDB::bind_method(D_METHOD("reset"), &BoneClothSimulator3D::reset);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "link_mode", PROPERTY_HINT_ENUM, "None,Sequential,Loop"), "set_link_mode", "get_link_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "link_split"), "set_link_split_enabled", "is_link_split_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_spacing/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_spacing", "get_link_spacing");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "link_spacing/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_link_spacing_damping_curve", "get_link_spacing_damping_curve");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_damping_curve", "get_drag_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_stiffness_damping_curve", "get_stiffness_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_radius_damping_curve", "get_radius_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "radius/rest_overlap"), "set_rest_overlap_enabled", "is_rest_overlap_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_limit_angle_damping_curve", "get_limit_angle_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_limit_angle_stiffness", "get_limit_angle_stiffness");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity/value", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,or_less,suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gravity/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_gravity_damping_curve", "get_gravity_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity/direction"), "set_gravity_direction", "get_gravity_direction");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sync/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_sync", "get_sync");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "sync/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_sync_damping_curve", "get_sync_damping_curve");

	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia_smoothing", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia_smoothing", "get_inertia_smoothing");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth_inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_depth_inertia", "get_depth_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_local_movement_speed_limit", "get_local_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "local_rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_local_rotation_speed_limit", "get_local_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "joint_speed_limit", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m/s"), "set_joint_speed_limit", "get_joint_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");

	ADD_GROUP("Root Spring", "root_spring_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_root_spring_stiffness", "get_root_spring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_limit", "get_root_spring_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "root_spring_forward_limit", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_root_spring_forward_limit", "get_root_spring_forward_limit");
	ADD_GROUP("", "");

	// godot-cpp에는 ADD_ARRAY_COUNT가 없어서 PROPERTY_USAGE_ARRAY로 대신한다.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_count", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY, "Chains,chains/"), "set_chain_count", "get_chain_count");

	BIND_ENUM_CONSTANT(LINK_MODE_NONE);
	BIND_ENUM_CONSTANT(LINK_MODE_SEQUENTIAL);
	BIND_ENUM_CONSTANT(LINK_MODE_LOOP);
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
	} else if (what == "sync_bone_name") {
		set_sync_bone_name(which, p_value);
	} else if (what == "sync_bone") {
		set_sync_bone(which, p_value);
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
	} else if (what == "sync_bone_name") {
		r_ret = get_sync_bone_name(which);
	} else if (what == "sync_bone") {
		r_ret = get_sync_bone(which);
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
		p_list->push_back(PropertyInfo(Variant::STRING, path + String("sync_bone_name"), hint, bone_names));
		p_list->push_back(PropertyInfo(Variant::INT, path + String("sync_bone"), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR));

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
	if (link_mode == LINK_MODE_NONE && (p_property.name == StringName("link_stiffness") || p_property.name == StringName("link_split") || String(p_property.name).begins_with("link_spacing/"))) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void BoneClothSimulator3D::apply_preset(const Ref<BoneClothPreset> &p_preset) {
	ERR_FAIL_COND_MSG(p_preset.is_null(), "No preset to apply.");
	for (const char *name : BoneClothPreset::VALUES) {
		set(name, BoneClothPreset::copy_value(p_preset->get(name)));
	}
}

Ref<BoneClothPreset> BoneClothSimulator3D::make_preset() const {
	Ref<BoneClothPreset> made;
	made.instantiate();
	for (const char *name : BoneClothPreset::VALUES) {
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

void BoneClothSimulator3D::set_sync_bone_name(int p_index, const String &p_bone_name) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	chains[p_index].sync_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_sync_bone(p_index, find_bone_or_warn(skeleton, p_bone_name, "Chain " + itos(p_index) + ": sync bone"));
	}
}

String BoneClothSimulator3D::get_sync_bone_name(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), String());
	return chains[p_index].sync_bone_name;
}

void BoneClothSimulator3D::set_sync_bone(int p_index, int p_bone) {
	ERR_FAIL_INDEX(p_index, int(chains.size()));
	Chain &chain = chains[p_index];
	chain.sync_bone = p_bone;
	read_bone_name(get_skeleton(), chain.sync_bone, chain.sync_bone_name, "Chain " + itos(p_index) + ": sync bone");

	update_configuration_warnings();
}

int BoneClothSimulator3D::get_sync_bone(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, int(chains.size()), -1);
	return chains[p_index].sync_bone;
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

void BoneClothSimulator3D::set_sync(float p_sync) {
	sync = p_sync;
}

float BoneClothSimulator3D::get_sync() const {
	return sync;
}

void BoneClothSimulator3D::set_sync_damping_curve(const Ref<Curve> &p_curve) {
	sync_damping_curve = p_curve;
}

Ref<Curve> BoneClothSimulator3D::get_sync_damping_curve() const {
	return sync_damping_curve;
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

			if (!chain.sync_bone_name.is_empty() && chain.sync_bone < 0) {
				problems.push_back(vformat("sync bone '%s' is not in the skeleton", chain.sync_bone_name));
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

		const String sync_name = chains[i].sync_bone_name;
		if (!sync_name.is_empty()) {
			set_sync_bone_name(i, sync_name);
		} else if (chains[i].sync_bone != -1) {
			set_sync_bone(i, chains[i].sync_bone);
		}
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
	_read_frame(skeleton);

	const Transform3D skeleton_transform = skeleton->get_global_transform_interpolated();
	if (needs_reset) {
		_reset(skeleton_transform);
	}

	LocalVector<PosedCollision> collisions;
	_gather_collisions(collisions);
	_find_chain_collisions(collisions);

	if (p_delta > 0.0) {
		FrameMotion motion = _measure_motion(p_delta, skeleton_transform);
		_run_steps(motion, collisions);
		_settle_display(p_delta, motion, collisions);
	}

	_end_frame(skeleton);

	uint64_t joint_count = 0;
	for (const Chain &chain : chains) {
		joint_count += chain.joints.size();
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

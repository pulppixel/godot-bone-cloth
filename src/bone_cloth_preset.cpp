#include "bone_cloth_preset.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothPreset::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_link_stiffness", "stiffness"), &BoneClothPreset::set_link_stiffness);
	ClassDB::bind_method(D_METHOD("get_link_stiffness"), &BoneClothPreset::get_link_stiffness);
	ClassDB::bind_method(D_METHOD("set_end_bone_length", "length"), &BoneClothPreset::set_end_bone_length);
	ClassDB::bind_method(D_METHOD("get_end_bone_length"), &BoneClothPreset::get_end_bone_length);
	ClassDB::bind_method(D_METHOD("set_drag", "drag"), &BoneClothPreset::set_drag);
	ClassDB::bind_method(D_METHOD("get_drag"), &BoneClothPreset::get_drag);
	ClassDB::bind_method(D_METHOD("set_drag_damping_curve", "curve"), &BoneClothPreset::set_drag_damping_curve);
	ClassDB::bind_method(D_METHOD("get_drag_damping_curve"), &BoneClothPreset::get_drag_damping_curve);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &BoneClothPreset::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &BoneClothPreset::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_stiffness_damping_curve", "curve"), &BoneClothPreset::set_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("get_stiffness_damping_curve"), &BoneClothPreset::get_stiffness_damping_curve);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothPreset::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothPreset::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius_damping_curve", "curve"), &BoneClothPreset::set_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("get_radius_damping_curve"), &BoneClothPreset::get_radius_damping_curve);
	ClassDB::bind_method(D_METHOD("set_limit_angle", "angle"), &BoneClothPreset::set_limit_angle);
	ClassDB::bind_method(D_METHOD("get_limit_angle"), &BoneClothPreset::get_limit_angle);
	ClassDB::bind_method(D_METHOD("set_limit_angle_damping_curve", "curve"), &BoneClothPreset::set_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("get_limit_angle_damping_curve"), &BoneClothPreset::get_limit_angle_damping_curve);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &BoneClothPreset::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &BoneClothPreset::get_gravity);
	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &BoneClothPreset::set_inertia);
	ClassDB::bind_method(D_METHOD("get_inertia"), &BoneClothPreset::get_inertia);
	ClassDB::bind_method(D_METHOD("set_movement_speed_limit", "limit"), &BoneClothPreset::set_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("get_movement_speed_limit"), &BoneClothPreset::get_movement_speed_limit);
	ClassDB::bind_method(D_METHOD("set_rotation_speed_limit", "limit"), &BoneClothPreset::set_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("get_rotation_speed_limit"), &BoneClothPreset::get_rotation_speed_limit);
	ClassDB::bind_method(D_METHOD("set_teleport_distance", "distance"), &BoneClothPreset::set_teleport_distance);
	ClassDB::bind_method(D_METHOD("get_teleport_distance"), &BoneClothPreset::get_teleport_distance);
	ClassDB::bind_method(D_METHOD("set_teleport_angle", "angle"), &BoneClothPreset::set_teleport_angle);
	ClassDB::bind_method(D_METHOD("get_teleport_angle"), &BoneClothPreset::get_teleport_angle);

	// The simulator's names and hints, so a preset reads as the node's values do.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "link_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_link_stiffness", "get_link_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "end_bone_length", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_end_bone_length", "get_end_bone_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "drag/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_drag_damping_curve", "get_drag_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness/value", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stiffness/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_stiffness_damping_curve", "get_stiffness_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius/value", PROPERTY_HINT_RANGE, "0,0.2,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "radius/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_radius_damping_curve", "get_radius_damping_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "limit_angle/value", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_limit_angle", "get_limit_angle");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "limit_angle/damping_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_limit_angle_damping_curve", "get_limit_angle_damping_curve");

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity", PROPERTY_HINT_NONE, "suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_GROUP("Inertia", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "movement_speed_limit", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_movement_speed_limit", "get_movement_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotation_speed_limit", PROPERTY_HINT_RANGE, "0,1440,0.1,radians_as_degrees,suffix:/s"), "set_rotation_speed_limit", "get_rotation_speed_limit");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_distance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater,suffix:m"), "set_teleport_distance", "get_teleport_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "teleport_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians_as_degrees"), "set_teleport_angle", "get_teleport_angle");
}

Variant BoneClothPreset::copy_value(const Variant &p_value) {
	const Ref<Curve> curve = p_value;
	return curve.is_valid() ? Variant(curve->duplicate()) : p_value;
}

void BoneClothPreset::set_link_stiffness(float p_stiffness) {
	link_stiffness = p_stiffness;
	emit_changed();
}

float BoneClothPreset::get_link_stiffness() const {
	return link_stiffness;
}

void BoneClothPreset::set_end_bone_length(float p_length) {
	end_bone_length = p_length;
	emit_changed();
}

float BoneClothPreset::get_end_bone_length() const {
	return end_bone_length;
}

void BoneClothPreset::set_drag(float p_drag) {
	drag = p_drag;
	emit_changed();
}

float BoneClothPreset::get_drag() const {
	return drag;
}

void BoneClothPreset::set_drag_damping_curve(const Ref<Curve> &p_curve) {
	drag_damping_curve = p_curve;
	emit_changed();
}

Ref<Curve> BoneClothPreset::get_drag_damping_curve() const {
	return drag_damping_curve;
}

void BoneClothPreset::set_stiffness(float p_stiffness) {
	stiffness = p_stiffness;
	emit_changed();
}

float BoneClothPreset::get_stiffness() const {
	return stiffness;
}

void BoneClothPreset::set_stiffness_damping_curve(const Ref<Curve> &p_curve) {
	stiffness_damping_curve = p_curve;
	emit_changed();
}

Ref<Curve> BoneClothPreset::get_stiffness_damping_curve() const {
	return stiffness_damping_curve;
}

void BoneClothPreset::set_radius(float p_radius) {
	radius = p_radius;
	emit_changed();
}

float BoneClothPreset::get_radius() const {
	return radius;
}

void BoneClothPreset::set_radius_damping_curve(const Ref<Curve> &p_curve) {
	radius_damping_curve = p_curve;
	emit_changed();
}

Ref<Curve> BoneClothPreset::get_radius_damping_curve() const {
	return radius_damping_curve;
}

void BoneClothPreset::set_limit_angle(float p_angle) {
	limit_angle = p_angle;
	emit_changed();
}

float BoneClothPreset::get_limit_angle() const {
	return limit_angle;
}

void BoneClothPreset::set_limit_angle_damping_curve(const Ref<Curve> &p_curve) {
	limit_angle_damping_curve = p_curve;
	emit_changed();
}

Ref<Curve> BoneClothPreset::get_limit_angle_damping_curve() const {
	return limit_angle_damping_curve;
}

void BoneClothPreset::set_gravity(const Vector3 &p_gravity) {
	gravity = p_gravity;
	emit_changed();
}

Vector3 BoneClothPreset::get_gravity() const {
	return gravity;
}

void BoneClothPreset::set_inertia(float p_inertia) {
	inertia = p_inertia;
	emit_changed();
}

float BoneClothPreset::get_inertia() const {
	return inertia;
}

void BoneClothPreset::set_movement_speed_limit(float p_limit) {
	movement_speed_limit = p_limit;
	emit_changed();
}

float BoneClothPreset::get_movement_speed_limit() const {
	return movement_speed_limit;
}

void BoneClothPreset::set_rotation_speed_limit(float p_limit) {
	rotation_speed_limit = p_limit;
	emit_changed();
}

float BoneClothPreset::get_rotation_speed_limit() const {
	return rotation_speed_limit;
}

void BoneClothPreset::set_teleport_distance(float p_distance) {
	teleport_distance = p_distance;
	emit_changed();
}

float BoneClothPreset::get_teleport_distance() const {
	return teleport_distance;
}

void BoneClothPreset::set_teleport_angle(float p_angle) {
	teleport_angle = p_angle;
	emit_changed();
}

float BoneClothPreset::get_teleport_angle() const {
	return teleport_angle;
}

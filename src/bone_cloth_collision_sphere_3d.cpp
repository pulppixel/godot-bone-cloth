#include "bone_cloth_collision_sphere_3d.h"

#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/core/class_db.hpp>


using namespace godot;

void BoneClothCollisionSphere3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &BoneClothCollisionSphere3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &BoneClothCollisionSphere3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_inside", "enabled"), &BoneClothCollisionSphere3D::set_inside);
	ClassDB::bind_method(D_METHOD("is_inside"), &BoneClothCollisionSphere3D::is_inside);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inside"), "set_inside", "is_inside");
}

void BoneClothCollisionSphere3D::set_radius(float p_radius)
{
	radius = p_radius;
}

float BoneClothCollisionSphere3D::get_radius() const
{
	return radius;
}

void BoneClothCollisionSphere3D::set_inside(bool p_enabled)
{
	inside = p_enabled;
}

bool BoneClothCollisionSphere3D::is_inside() const
{
	return inside;
}

// A point p_point_radius thick against a sphere. Outside, it goes out to the two radii from the centre, as KawaiiPhysics' and the engine's do;
// a point at the centre has no direction out and takes p_fallback. Inside, it comes in to the sphere's radius less its own, never below zero,
// so a joint thicker than the sphere stays at the centre: KawaiiPhysics' guard, where the engine's limit goes negative and throws it to the far side.
Vector3 BoneClothCollisionSphere3D::_collide_sphere(const Vector3& p_center, float p_radius, bool p_inside, const Vector3& p_fallback, const Vector3& p_point, float p_point_radius)
{
	const Vector3 offset = p_point - p_center;
	const float distance_squared = offset.length_squared();
	if (p_inside) {
		const float limit = MAX(p_radius - p_point_radius, 0.0f);
		if (distance_squared <= limit * limit) {
			return p_point;
		}
		return p_center + offset / Math::sqrt(distance_squared) * limit;
	}

	const float limit = p_radius + p_point_radius;
	if (distance_squared >= limit * limit) {
		return p_point;
	}

	const Vector3 direction = distance_squared > CMP_EPSILON2 ? offset / Math::sqrt(distance_squared) : p_fallback;
	return p_center + direction * limit;
}

// The fallback direction is the node's X.
Vector3 BoneClothCollisionSphere3D::collide(const Vector3& p_point, float p_radius) const
{
	return _collide_sphere(pose.origin, radius, inside, pose.basis.get_column(0).normalized(), p_point, p_radius);
}

// Inside, the room left for the line is a smaller sphere and convex, so the line stays in when both of its ends do.
void BoneClothCollisionSphere3D::collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const
{
	if (inside) {
		r_a = collide(r_a, p_radius);
		r_b = collide(r_b, p_radius);
		return;
	}

	const Vector3 on_segment = Geometry3D::get_singleton()->get_closest_point_to_segment(pose.origin, r_a, r_b);
	_push_segment_out(r_a, r_b, on_segment, pose.origin, radius + p_radius, pose.basis.get_column(0).normalized());
}

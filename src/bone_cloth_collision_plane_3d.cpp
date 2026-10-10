#include "bone_cloth_collision_plane_3d.h"

using namespace godot;

void BoneClothCollisionPlane3D::_bind_methods()
{
}

// A joint behind the plane, or nearer it than its radius, goes out along the normal to its radius, as the engine's.
// The distance is signed, so a joint that went far through in one step still comes back; KawaiiPhysics measures it unsigned and needs a crossing test for that.
Vector3 BoneClothCollisionPlane3D::collide(const Vector3& p_point, float p_radius) const
{
	const Vector3 normal = pose.basis.get_column(1).normalized();
	const float distance = (p_point - pose.origin).dot(normal) - p_radius;
	if (distance >= 0.0f) {
		return p_point;
	}

	return p_point - normal * distance;
}

// The side the plane faces, less the line's thickness, is convex, so the line stays out when both of its ends do.
void BoneClothCollisionPlane3D::collide_segment(Vector3& r_a, Vector3& r_b, float p_radius) const
{
	r_a = collide(r_a, p_radius);
	r_b = collide(r_b, p_radius);
}

// A capsule with a radius at each end on a bone, named after CylinderMesh: the joints stay out of it, or in it with inside on.
// The axis is the node's Y, top_radius at +Y. The shape is the convex hull of the two end spheres, as Jolt's TaperedCapsuleShape.
// The push is KawaiiPhysics' tapered capsule collision (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md,
// with the axis point moved so it collides with the hull rather than KawaiiPhysics' approximation of it.

#pragma once

#include "collision/bone_cloth_collision_3d.h"

using namespace godot;

class BoneClothCollisionTaperedCapsule3D : public BoneClothCollision3D {
	GDCLASS(BoneClothCollisionTaperedCapsule3D, BoneClothCollision3D)

	float top_radius = 0.1f;
	float bottom_radius = 0.05f;
	float height = 0.5f;
	bool inside = false;

	void _get_head_and_tail(Vector3 &r_head, Vector3 &r_tail) const;
	void _get_nearest_sphere(const Vector3 &p_head, const Vector3 &p_tail, const Vector3 &p_point, Vector3 &r_center, float &r_radius) const;

protected:
	static void _bind_methods();

public:
	void set_top_radius(float p_radius);
	float get_top_radius() const;
	void set_bottom_radius(float p_radius);
	float get_bottom_radius() const;
	void set_height(float p_height);
	float get_height() const;
	void set_inside(bool p_enabled);
	bool is_inside() const;

	Vector3 collide(const Vector3 &p_point, float p_radius) const override;
	void collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius) const override;
};

// The per-axis limit is KawaiiPhysics' (MIT, Copyright (c) 2019-2026 pafuhana1213), see THIRD_PARTY_NOTICES.md.

#pragma once

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

struct SwingLimit {
	Vector2 lower;
	Vector2 upper;
};

struct SwingFrame {
	Vector3 axis_a;
	Vector3 axis_b;
};

int find_twist_axis(const Basis &p_basis, const Vector3 &p_direction);
SwingLimit make_swing_limit(int p_twist_axis, const Vector3 &p_lower, const Vector3 &p_upper, float p_scale);
SwingFrame make_swing_frame(const Basis &p_parent_basis, int p_twist_axis, const Vector3 &p_pose_direction);
Vector3 compose_swing(const SwingFrame &p_frame, const Vector3 &p_pose_direction, const Vector2 &p_swing);
bool clamp_direction(const SwingFrame &p_frame, const Vector3 &p_pose_direction, const SwingLimit &p_limit, Vector3 &r_direction);
Vector2 get_limit_boundary(const SwingLimit &p_limit, float p_alpha);

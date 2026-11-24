#include "bone_cloth_angle_limit.h"

#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/quaternion.hpp>

using namespace godot;

static constexpr float LOCKED_EXTENT = Math::deg_to_rad(0.01f);
static constexpr int CLOSEST_POINT_STEPS = 12;
static constexpr double CLOSEST_POINT_TOLERANCE = 1.0e-9;
static constexpr double ANTI_PARALLEL_SINE = 1.0e-4;

static void get_swing_axes(int p_twist_axis, int &r_axis_a, int &r_axis_b) {
	r_axis_a = p_twist_axis == 0 ? 1 : 0;
	r_axis_b = p_twist_axis == 2 ? 1 : 2;
}

static bool is_locked(float p_lower, float p_upper) {
	return p_upper <= LOCKED_EXTENT && -p_lower <= LOCKED_EXTENT;
}

int find_twist_axis(const Basis &p_basis, const Vector3 &p_direction) {
	const Basis basis = p_basis.orthonormalized();
	int twist_axis = 0;
	for (int i = 1; i < 3; i++) {
		if (Math::abs(basis.get_column(i).dot(p_direction)) > Math::abs(basis.get_column(twist_axis).dot(p_direction))) {
			twist_axis = i;
		}
	}
	return twist_axis;
}

SwingLimit make_swing_limit(int p_twist_axis, const Vector3 &p_lower, const Vector3 &p_upper, float p_scale) {
	int axis_a;
	int axis_b;
	get_swing_axes(p_twist_axis, axis_a, axis_b);
	const float scale = MAX(p_scale, 0.0f);

	SwingLimit limit;
	limit.lower = Vector2(CLAMP(p_lower[axis_a] * scale, -float(Math::PI), 0.0f), CLAMP(p_lower[axis_b] * scale, -float(Math::PI), 0.0f));
	limit.upper = Vector2(CLAMP(p_upper[axis_a] * scale, 0.0f, float(Math::PI)), CLAMP(p_upper[axis_b] * scale, 0.0f, float(Math::PI)));
	return limit;
}

SwingFrame make_swing_frame(const Basis &p_parent_basis, int p_twist_axis, const Vector3 &p_pose_direction) {
	const Basis basis = p_parent_basis.orthonormalized();
	Vector3 forward = basis.get_column(p_twist_axis);
	if (forward.dot(p_pose_direction) < 0.0f) {
		forward = -forward;
	}

	const Quaternion alignment(forward, p_pose_direction);
	int axis_a;
	int axis_b;
	get_swing_axes(p_twist_axis, axis_a, axis_b);

	SwingFrame frame;
	frame.axis_a = alignment.xform(basis.get_column(axis_a));
	frame.axis_b = alignment.xform(basis.get_column(axis_b));
	return frame;
}

static Vector2 find_closest_on_ellipse(float p_x, float p_y, float p_extent_x, float p_extent_y) {
	double x = p_x;
	double y = p_y;
	double extent_x = p_extent_x;
	double extent_y = p_extent_y;
	const bool swapped = extent_x < extent_y;
	if (swapped) {
		SWAP(x, y);
		SWAP(extent_x, extent_y);
	}

	double closest_x;
	double closest_y;
	if (y <= 0.0) {
		closest_x = extent_x;
		closest_y = 0.0;
	} else if (x <= 0.0) {
		closest_x = 0.0;
		closest_y = extent_y;
	} else {
		const double normal_x = x / extent_x;
		const double normal_y = y / extent_y;
		const double ratio = (extent_x / extent_y) * (extent_x / extent_y);
		double root = MAX(MAX(0.0, normal_y - 1.0), ratio * (normal_x - 1.0));
		for (int i = 0; i < CLOSEST_POINT_STEPS; i++) {
			const double denominator_x = root + ratio;
			const double denominator_y = root + 1.0;
			const double term_x = ratio * normal_x / denominator_x;
			const double term_y = normal_y / denominator_y;
			const double value = term_x * term_x + term_y * term_y - 1.0;
			if (value <= CLOSEST_POINT_TOLERANCE) {
				break;
			}
			root += value / (2.0 * (term_x * term_x / denominator_x + term_y * term_y / denominator_y));
		}

		closest_x = ratio * x / (root + ratio);
		closest_y = y / (root + 1.0);
		const double boundary = 1.0 / Math::sqrt((closest_x / extent_x) * (closest_x / extent_x) + (closest_y / extent_y) * (closest_y / extent_y));
		closest_x *= boundary;
		closest_y *= boundary;
	}

	return swapped ? Vector2(float(closest_y), float(closest_x)) : Vector2(float(closest_x), float(closest_y));
}

static bool clamp_hinge(const Vector3 &p_axis, const Vector3 &p_pose_direction, float p_lower, float p_upper, Vector3 &r_direction) {
	const Vector3 tangent = p_axis.cross(p_pose_direction);
	const float angle = Math::atan2(r_direction.dot(tangent), r_direction.dot(p_pose_direction));
	float clamped = angle;
	if (angle < p_lower || angle > p_upper) {
		clamped = Math::abs(Math::angle_difference(angle, p_lower)) < Math::abs(Math::angle_difference(angle, p_upper)) ? p_lower : p_upper;
	}

	const Vector3 direction = p_pose_direction * Math::cos(clamped) + tangent * Math::sin(clamped);
	if (direction.is_equal_approx(r_direction)) {
		return false;
	}
	r_direction = direction;
	return true;
}

static Vector2 decompose_swing(const SwingFrame &p_frame, const Vector3 &p_pose_direction, const Vector3 &p_direction, const SwingLimit &p_limit) {
	const Vector3 cross = p_pose_direction.cross(p_direction);
	const float sine = cross.length();
	const float cosine = p_pose_direction.dot(p_direction);
	if (cosine >= 0.0f && sine <= CMP_EPSILON2) {
		return Vector2();
	}

	const float angle = Math::atan2(sine, cosine);
	if (cosine < 0.0f && sine <= ANTI_PARALLEL_SINE) {
		const Vector2 candidates[4] = { Vector2(angle, 0.0f), Vector2(-angle, 0.0f), Vector2(0.0f, angle), Vector2(0.0f, -angle) };
		const float extents[4] = { p_limit.upper.x, -p_limit.lower.x, p_limit.upper.y, -p_limit.lower.y };
		int widest = 0;
		for (int i = 1; i < 4; i++) {
			if (extents[i] > extents[widest]) {
				widest = i;
			}
		}
		return candidates[widest];
	}

	const Vector3 rotation = cross / sine * angle;
	return Vector2(rotation.dot(p_frame.axis_a), rotation.dot(p_frame.axis_b));
}

Vector3 compose_swing(const SwingFrame &p_frame, const Vector3 &p_pose_direction, const Vector2 &p_swing) {
	const Vector3 rotation = p_frame.axis_a * p_swing.x + p_frame.axis_b * p_swing.y;
	const float angle = rotation.length();
	if (angle <= CMP_EPSILON2) {
		return p_pose_direction;
	}
	return p_pose_direction.rotated(rotation / angle, angle);
}

static Vector2 clamp_swing(const Vector2 &p_swing, const SwingLimit &p_limit) {
	const float extent_a = p_swing.x >= 0.0f ? p_limit.upper.x : -p_limit.lower.x;
	const float extent_b = p_swing.y >= 0.0f ? p_limit.upper.y : -p_limit.lower.y;
	const float a = Math::abs(p_swing.x);
	const float b = Math::abs(p_swing.y);

	Vector2 clamped;
	if (extent_a <= LOCKED_EXTENT) {
		clamped = Vector2(0.0f, MIN(b, extent_b));
	} else if (extent_b <= LOCKED_EXTENT) {
		clamped = Vector2(MIN(a, extent_a), 0.0f);
	} else {
		if ((a / extent_a) * (a / extent_a) + (b / extent_b) * (b / extent_b) <= 1.0f) {
			return p_swing;
		}
		clamped = find_closest_on_ellipse(a, b, extent_a, extent_b);
	}
	return Vector2(SIGN(p_swing.x) * clamped.x, SIGN(p_swing.y) * clamped.y);
}

bool clamp_direction(const SwingFrame &p_frame, const Vector3 &p_pose_direction, const SwingLimit &p_limit, Vector3 &r_direction) {
	const bool locked_a = is_locked(p_limit.lower.x, p_limit.upper.x);
	const bool locked_b = is_locked(p_limit.lower.y, p_limit.upper.y);
	if (locked_a != locked_b) {
		return locked_a ? clamp_hinge(p_frame.axis_b, p_pose_direction, p_limit.lower.y, p_limit.upper.y, r_direction) : clamp_hinge(p_frame.axis_a, p_pose_direction, p_limit.lower.x, p_limit.upper.x, r_direction);
	}

	const Vector2 swing = decompose_swing(p_frame, p_pose_direction, r_direction, p_limit);
	const Vector2 clamped = clamp_swing(swing, p_limit);
	if (clamped == swing) {
		return false;
	}
	r_direction = compose_swing(p_frame, p_pose_direction, clamped);
	return true;
}

Vector2 get_limit_boundary(const SwingLimit &p_limit, float p_alpha) {
	const float angle = p_alpha * float(Math::TAU);
	const float cosine = Math::cos(angle);
	const float sine = Math::sin(angle);
	const float extent_a = cosine >= 0.0f ? p_limit.upper.x : -p_limit.lower.x;
	const float extent_b = sine >= 0.0f ? p_limit.upper.y : -p_limit.lower.y;
	return Vector2(extent_a * cosine, extent_b * sine);
}

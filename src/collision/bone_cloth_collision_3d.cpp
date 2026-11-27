#include "collision/bone_cloth_collision_3d.h"

#include "bone_cloth_bone_lookup.h"
#include "bone_cloth_simulator_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void BoneClothCollision3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_skeleton"), &BoneClothCollision3D::get_skeleton);

	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &BoneClothCollision3D::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &BoneClothCollision3D::get_bone_name);
	ClassDB::bind_method(D_METHOD("set_bone", "bone"), &BoneClothCollision3D::set_bone);
	ClassDB::bind_method(D_METHOD("get_bone"), &BoneClothCollision3D::get_bone);
	ClassDB::bind_method(D_METHOD("set_mirror_bone_name", "bone_name"), &BoneClothCollision3D::set_mirror_bone_name);
	ClassDB::bind_method(D_METHOD("get_mirror_bone_name"), &BoneClothCollision3D::get_mirror_bone_name);
	ClassDB::bind_method(D_METHOD("set_mirror_bone", "bone"), &BoneClothCollision3D::set_mirror_bone);
	ClassDB::bind_method(D_METHOD("get_mirror_bone"), &BoneClothCollision3D::get_mirror_bone);

	ClassDB::bind_method(D_METHOD("set_position_offset", "offset"), &BoneClothCollision3D::set_position_offset);
	ClassDB::bind_method(D_METHOD("get_position_offset"), &BoneClothCollision3D::get_position_offset);
	ClassDB::bind_method(D_METHOD("set_rotation_offset", "offset"), &BoneClothCollision3D::set_rotation_offset);
	ClassDB::bind_method(D_METHOD("get_rotation_offset"), &BoneClothCollision3D::get_rotation_offset);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "bone_name"), "set_bone_name", "get_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_bone", "get_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "mirror_bone_name"), "set_mirror_bone_name", "get_mirror_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mirror_bone", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_mirror_bone", "get_mirror_bone");

	ADD_GROUP("Offset", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position_offset", PROPERTY_HINT_NONE, "suffix:m"), "set_position_offset", "get_position_offset");
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "rotation_offset"), "set_rotation_offset", "get_rotation_offset");
}

void BoneClothCollision3D::_validate_property(PropertyInfo &p_property) const {
	if (Engine::get_singleton()->is_editor_hint() && (p_property.name == StringName("bone_name") || p_property.name == StringName("mirror_bone_name"))) {
		Skeleton3D *skeleton = get_skeleton();
		if (skeleton) {
			p_property.hint = PROPERTY_HINT_ENUM_SUGGESTION;
			p_property.hint_string = skeleton->get_concatenated_bone_names();
		} else {
			p_property.hint = PROPERTY_HINT_NONE;
			p_property.hint_string = "";
		}
	}
	if (bone < 0 && (p_property.name == StringName("position_offset") || p_property.name == StringName("rotation_offset"))) {
		p_property.usage = PROPERTY_USAGE_NONE;
	} else if (bone < 0 && p_property.name == StringName("mirror_bone_name")) {
		// PROPERTY_USAGE_NONE을 쓰면 mirror bone이 없는 collision도 씬 파일에 mirror_bone_name = ""을 저장한다.
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void BoneClothCollision3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
		case NOTIFICATION_PARENTED: {
			_validate_bone_name();
		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			_take_offsets_from_transform();
		} break;
	}
}

// set_bone_name(bone_name)을 바로 부르면 이름이 지워지므로, 복사해서 넘긴다.
void BoneClothCollision3D::_validate_bone_name() {
	const String name = bone_name;
	if (!name.is_empty()) {
		set_bone_name(name);
	} else if (bone != -1) {
		set_bone(bone);
	}

	const String mirror_name = mirror_bone_name;
	if (!mirror_name.is_empty()) {
		set_mirror_bone_name(mirror_name);
	} else if (mirror_bone != -1) {
		set_mirror_bone(mirror_bone);
	}
}

PackedStringArray BoneClothCollision3D::_get_configuration_warnings() const {
	PackedStringArray warnings;
	if (!Object::cast_to<BoneClothSimulator3D>(get_parent())) {
		warnings.push_back("Parent node should be a BoneClothSimulator3D node.");
	}

	return warnings;
}

Skeleton3D *BoneClothCollision3D::get_skeleton() const {
	BoneClothSimulator3D *parent = Object::cast_to<BoneClothSimulator3D>(get_parent());
	return parent ? parent->get_skeleton() : nullptr;
}

void BoneClothCollision3D::set_bone_name(const String &p_bone_name) {
	bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_bone(find_bone_or_warn(skeleton, bone_name, "Bone"));
	}
}

String BoneClothCollision3D::get_bone_name() const {
	return bone_name;
}

void BoneClothCollision3D::set_bone(int p_bone) {
	bone = p_bone;
	read_bone_name(get_skeleton(), bone, bone_name, "Bone");

	notify_property_list_changed();
}

int BoneClothCollision3D::get_bone() const {
	return bone;
}

void BoneClothCollision3D::set_mirror_bone_name(const String &p_bone_name) {
	mirror_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_mirror_bone(find_bone_or_warn(skeleton, mirror_bone_name, "Mirror bone"));
	}
}

String BoneClothCollision3D::get_mirror_bone_name() const {
	return mirror_bone_name;
}

void BoneClothCollision3D::set_mirror_bone(int p_bone) {
	mirror_bone = p_bone;
	read_bone_name(get_skeleton(), mirror_bone, mirror_bone_name, "Mirror bone");

	sync_pose();
	update_gizmos();
}

int BoneClothCollision3D::get_mirror_bone() const {
	return mirror_bone;
}

bool BoneClothCollision3D::has_mirror() const {
	return bone >= 0 && mirror_bone >= 0;
}

void BoneClothCollision3D::set_position_offset(const Vector3 &p_offset) {
	if (position_offset == p_offset) {
		return;
	}
	position_offset = p_offset;
	sync_pose();
	update_gizmos();
}

Vector3 BoneClothCollision3D::get_position_offset() const {
	return position_offset;
}

void BoneClothCollision3D::set_rotation_offset(const Quaternion &p_offset) {
	ERR_FAIL_COND_MSG(p_offset.length_squared() == 0.0f, "rotation_offset cannot be (0, 0, 0, 0): that is not a rotation.");
	if (rotation_offset == p_offset) {
		return;
	}
	rotation_offset = p_offset;
	sync_pose();
	update_gizmos();
}

Quaternion BoneClothCollision3D::get_rotation_offset() const {
	return rotation_offset;
}

static Transform3D get_pose_on_bone(const Skeleton3D *p_skeleton, int p_bone, const Vector3 &p_position_offset, const Quaternion &p_rotation_offset, float p_units, bool p_rest = false) {
	Transform3D result = p_rest ? p_skeleton->get_bone_global_rest(p_bone) : p_skeleton->get_bone_global_pose(p_bone);
	result.origin += result.basis.get_rotation_quaternion().xform(p_position_offset * p_units);
	result.basis *= Basis(p_rotation_offset) * p_units;
	return result;
}

void BoneClothCollision3D::sync_pose() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D skeleton_global = skeleton->get_global_transform();
	if (bone >= 0) {
		units_per_meter = Object::cast_to<BoneClothSimulator3D>(get_parent())->get_units_per_meter();
		pose = get_pose_on_bone(skeleton, bone, position_offset, rotation_offset, units_per_meter);
		rest_pose = get_pose_on_bone(skeleton, bone, position_offset, rotation_offset, units_per_meter, true);
		syncing = true;
		set_global_transform(skeleton_global * pose);
		syncing = false;
	} else {
		units_per_meter = 1.0f;
		pose = skeleton_global.affine_inverse() * get_global_transform();
	}

	const float meters = 1.0f / units_per_meter;
	reach = _get_reach(Transform3D(pose.basis * meters, pose.origin * meters)) * units_per_meter;
	if (!has_mirror()) {
		return;
	}
	const Quaternion rest = skeleton->get_bone_global_rest(bone).basis.get_rotation_quaternion();
	const Quaternion mirror_rest = skeleton->get_bone_global_rest(mirror_bone).basis.get_rotation_quaternion();
	const Vector3 position = rest.xform(position_offset);
	const Quaternion rotation = rest * rotation_offset;
	const Vector3 mirrored_position(-position.x, position.y, position.z);
	const Quaternion mirrored_rotation(rotation.x, -rotation.y, -rotation.z, rotation.w);
	const Vector3 mirror_offset = mirror_rest.inverse().xform(mirrored_position);
	const Quaternion mirror_rotation = mirror_rest.inverse() * mirrored_rotation;
	mirror_pose = get_pose_on_bone(skeleton, mirror_bone, mirror_offset, mirror_rotation, units_per_meter);
	mirror_rest_pose = get_pose_on_bone(skeleton, mirror_bone, mirror_offset, mirror_rotation, units_per_meter, true);
	mirror_reach = _get_reach(Transform3D(mirror_pose.basis * meters, mirror_pose.origin * meters)) * units_per_meter;
	update_gizmos();
}

void BoneClothCollision3D::_take_offsets_from_transform() {
	if (syncing || bone < 0 || !is_inside_tree()) {
		return;
	}
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D bone_pose = skeleton->get_bone_global_pose(bone);
	const Transform3D moved = skeleton->get_global_transform().affine_inverse() * get_global_transform();
	// 노드가 움직이지 않았는데 offset을 다시 계산하면, float 오차가 씬 파일에 저장된다.
	if (moved.is_equal_approx(pose)) {
		return;
	}

	position_offset = bone_pose.basis.get_rotation_quaternion().inverse().xform(moved.origin - bone_pose.origin) / units_per_meter;
	rotation_offset = (bone_pose.basis.inverse() * moved.basis).get_rotation_quaternion();
	sync_pose();
}

void BoneClothCollision3D::collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius, bool p_mirror) const {
	const Transform3D &shape = p_mirror ? mirror_pose : pose;
	const float shape_reach = p_mirror ? mirror_reach : reach;
	if (shape_reach >= 0.0f) {
		// float 오차로 닿는 선분을 건너뛰지 않게 조금 넉넉하게 본다.
		const float limit = (shape_reach + p_radius) * 1.001f;
		if (_get_closest_point_to_segment(shape.origin, r_a, r_b).distance_squared_to(shape.origin) > limit * limit) {
			return;
		}
	}

	const float meters = 1.0f / units_per_meter;
	const Vector3 a = r_a * meters;
	const Vector3 b = r_b * meters;
	Vector3 pushed_a = a;
	Vector3 pushed_b = b;
	_collide_segment(Transform3D(shape.basis * meters, shape.origin * meters), pushed_a, pushed_b, p_radius * meters);
	// meters를 곱했다 되돌리면 float 오차가 생겨서, 닿지 않은 관절도 밀린 것으로 보인다.
	if (pushed_a != a || pushed_b != b) {
		r_a = pushed_a * units_per_meter;
		r_b = pushed_b * units_per_meter;
	}
}

float BoneClothCollision3D::get_rest_depth(const Vector3 &p_a, const Vector3 &p_b, float p_radius, bool p_mirror) const {
	if (bone < 0) {
		return 0.0f;
	}

	const float meters = 1.0f / units_per_meter;
	const Transform3D &shape = p_mirror ? mirror_rest_pose : rest_pose;
	return _segment_depth(Transform3D(shape.basis * meters, shape.origin * meters), p_a * meters, p_b * meters, p_radius * meters) * units_per_meter;
}

bool BoneClothCollision3D::separates(const Vector3 &p_a, const Vector3 &p_b, bool p_mirror) const {
	const float meters = 1.0f / units_per_meter;
	const Transform3D &shape = p_mirror ? mirror_pose : pose;
	const Transform3D shape_in_meters(shape.basis * meters, shape.origin * meters);
	const Vector3 a = p_a * meters;
	const Vector3 b = p_b * meters;
	const Vector3 out_a = _collide(shape_in_meters, a, 1000.0f) - a;
	const Vector3 out_b = _collide(shape_in_meters, b, 1000.0f) - b;
	return out_a.dot(out_b) < 0.0f;
}

float BoneClothCollision3D::_segment_depth(const Transform3D &p_pose, const Vector3 &p_a, const Vector3 &p_b, float p_radius) const {
	return MAX(_collide(p_pose, p_a, p_radius).distance_to(p_a), _collide(p_pose, p_b, p_radius).distance_to(p_b));
}

Vector3 BoneClothCollision3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	return p_point;
}

float BoneClothCollision3D::_get_reach(const Transform3D &p_pose) const {
	return -1.0f;
}

void BoneClothCollision3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
	r_a = _collide(p_pose, r_a, p_radius);
	r_b = _collide(p_pose, r_b, p_radius);
}

void BoneClothCollision3D::_push_segment_out(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_on_segment, const Vector3 &p_on_shape, float p_limit, const Vector3 &p_fallback) {
	const Vector3 offset = p_on_segment - p_on_shape;
	const float distance_squared = offset.length_squared();
	if (distance_squared >= p_limit * p_limit) {
		return;
	}

	const float distance = Math::sqrt(distance_squared);
	const Vector3 direction = distance > CMP_EPSILON ? offset / distance : p_fallback;
	_push_segment(r_a, r_b, p_on_segment, direction * (p_limit - distance));
}

// 회전만 있는 basis의 float 오차를 scale로 읽지 않도록, 1에 가까우면 1로 둔다.
Vector3 BoneClothCollision3D::_get_scale(const Basis &p_basis) {
	const Vector3 scale = p_basis.get_scale_abs();
	return scale.is_equal_approx(Vector3(1, 1, 1)) ? Vector3(1, 1, 1) : scale;
}

// Godot 4.7.2 Geometry3D의 같은 함수를 옮겨 왔다. 엔진 API로 부르면 호출마다 배열을 새로 만들어서 그것이 충돌 비용의 대부분이 된다.
static void closest_points_between_segments(const Vector3 &p_p0, const Vector3 &p_p1, const Vector3 &p_q0, const Vector3 &p_q1, Vector3 &r_ps, Vector3 &r_qt) {
	const Vector3 p = p_p1 - p_p0;
	const Vector3 q = p_q1 - p_q0;
	const Vector3 r = p_p0 - p_q0;

	const float a = p.dot(p);
	const float b = p.dot(q);
	const float c = q.dot(q);
	const float d = p.dot(r);
	const float e = q.dot(r);

	float s = 0.0f;
	float t = 0.0f;

	const float det = a * c - b * b;
	if (det > CMP_EPSILON) {
		const float bte = b * e;
		const float ctd = c * d;

		if (bte <= ctd) {
			if (e <= 0.0f) {
				s = (-d >= a ? 1 : (-d > 0.0f ? -d / a : 0.0f));
				t = 0.0f;
			} else if (e < c) {
				s = 0.0f;
				t = e / c;
			} else {
				s = (b - d >= a ? 1 : (b - d > 0.0f ? (b - d) / a : 0.0f));
				t = 1;
			}
		} else {
			s = bte - ctd;
			if (s >= det) {
				if (b + e <= 0.0f) {
					s = (-d <= 0.0f ? 0.0f : (-d < a ? -d / a : 1));
					t = 0.0f;
				} else if (b + e < c) {
					s = 1;
					t = (b + e) / c;
				} else {
					s = (b - d <= 0.0f ? 0.0f : (b - d < a ? (b - d) / a : 1));
					t = 1;
				}
			} else {
				const float ate = a * e;
				const float btd = b * d;

				if (ate <= btd) {
					s = (-d <= 0.0f ? 0.0f : (-d >= a ? 1 : -d / a));
					t = 0.0f;
				} else {
					t = ate - btd;
					if (t >= det) {
						s = (b - d <= 0.0f ? 0.0f : (b - d >= a ? 1 : (b - d) / a));
						t = 1;
					} else {
						s /= det;
						t /= det;
					}
				}
			}
		}
	} else {
		if (e <= 0.0f) {
			s = (-d <= 0.0f ? 0.0f : (-d >= a ? 1 : -d / a));
			t = 0.0f;
		} else if (e >= c) {
			s = (b - d <= 0.0f ? 0.0f : (b - d >= a ? 1 : (b - d) / a));
			t = 1;
		} else {
			s = 0.0f;
			t = e / c;
		}
	}

	r_ps = (1 - s) * p_p0 + s * p_p1;
	r_qt = (1 - t) * p_q0 + t * p_q1;
}

void BoneClothCollision3D::_get_closest_points(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_head, const Vector3 &p_tail, Vector3 &r_on_segment, Vector3 &r_on_axis) {
	const Vector3 segment = p_b - p_a;
	const Vector3 axis = p_tail - p_head;
	const float a = segment.dot(segment);
	const float b = segment.dot(axis);
	const float c = axis.dot(axis);
	const float det = a * c - b * b;
	// Geometry3D는 det를 고정값 1e-5 m^4와 비교해서, 짧은 마디와 짧은 캡슐은 직각으로 만나도 평행으로 본다.
	if (det > CMP_EPSILON || det <= CMP_EPSILON * a * c) {
		closest_points_between_segments(p_a, p_b, p_head, p_tail, r_on_segment, r_on_axis);
		return;
	}

	const float scale = 1.0f / Math::sqrt(Math::sqrt(a * c));
	closest_points_between_segments(p_a * scale, p_b * scale, p_head * scale, p_tail * scale, r_on_segment, r_on_axis);
	r_on_segment /= scale;
	r_on_axis /= scale;
}

Vector3 BoneClothCollision3D::_get_closest_point_to_segment(const Vector3 &p_point, const Vector3 &p_a, const Vector3 &p_b) {
	const Vector3 p = p_point - p_a;
	const Vector3 n = p_b - p_a;
	const float l2 = n.length_squared();
	if (l2 < 1e-20f) {
		return p_a;
	}

	const float d = n.dot(p) / l2;
	if (d <= 0.0f) {
		return p_a;
	} else if (d >= 1.0f) {
		return p_b;
	}
	return p_a + n * d;
}

Vector3 BoneClothCollision3D::_collide_sphere(const Vector3 &p_center, float p_radius, bool p_inside, const Vector3 &p_fallback, const Vector3 &p_point, float p_point_radius) {
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

void BoneClothCollision3D::_push_segment(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_on_segment, const Vector3 &p_push) {
	const Vector3 line = r_b - r_a;
	const float line_length_squared = line.length_squared();
	const float s = line_length_squared > 0.0f ? (p_on_segment - r_a).dot(line) / line_length_squared : 0.0f;
	const float scale = 1.0f / ((1.0f - s) * (1.0f - s) + s * s);
	r_a += p_push * ((1.0f - s) * scale);
	r_b += p_push * (s * scale);
}

BoneClothCollision3D::BoneClothCollision3D() {
	set_notify_local_transform(true);
}

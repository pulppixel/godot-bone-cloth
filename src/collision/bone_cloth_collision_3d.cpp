#include "collision/bone_cloth_collision_3d.h"

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

// 인스펙터 표시 형식을 정한다. bone 이름 두 개는 skeleton의 bone 목록에서 고른다(연필 버튼으로 직접 입력도 된다).
// bone이 없으면 mirror_bone_name을 숨기되 저장은 한다(저장까지 막으면 모든 collision에 빈 이름이 저장된다).
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
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

// 노드가 simulator 아래에 들어오면, bone을 다시 찾는다 (이름으로 찾는다).
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

// 이름을 먼저 복사해 둔다. set_bone_name(bone_name)처럼 자기 값을 넘기면 이름이 지워진다.
// (godot-cpp의 String 대입이 자기 자신을 검사하지 않는다)
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

// bone 이름으로 번호를 찾는다.
void BoneClothCollision3D::set_bone_name(const String &p_bone_name) {
	bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(bone_name);
		if (found < 0 && !bone_name.is_empty()) {
			WARN_PRINT_ED("Bone not found: '" + bone_name + "'.");
		}
		set_bone(found);
	}
}

String BoneClothCollision3D::get_bone_name() const {
	return bone_name;
}

// 번호로 bone을 정하고 이름을 설정한다. -1은 bone 없음.
void BoneClothCollision3D::set_bone(int p_bone) {
	bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && bone >= 0) {
		if (bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Bone index " + itos(bone) + " is out of range.");
			bone = -1;
		} else {
			bone_name = skeleton->get_bone_name(bone);
		}
	}

	notify_property_list_changed();
}

int BoneClothCollision3D::get_bone() const {
	return bone;
}

// 반대쪽 bone(예: bone이 LeftUpperLeg면 RightUpperLeg).
// 자기 bone도 고를 수 있다(Hips를 고르면 몸 가운데를 기준으로 뒤집는다).
void BoneClothCollision3D::set_mirror_bone_name(const String &p_bone_name) {
	mirror_bone_name = p_bone_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		const int found = skeleton->find_bone(mirror_bone_name);
		if (found < 0 && !mirror_bone_name.is_empty()) {
			WARN_PRINT_ED("Mirror bone not found: '" + mirror_bone_name + "'.");
		}
		set_mirror_bone(found);
	}
}

String BoneClothCollision3D::get_mirror_bone_name() const {
	return mirror_bone_name;
}

// -1은 mirror 없음.
void BoneClothCollision3D::set_mirror_bone(int p_bone) {
	mirror_bone = p_bone;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton && mirror_bone >= 0) {
		if (mirror_bone >= skeleton->get_bone_count()) {
			WARN_PRINT_ED("Mirror bone index " + itos(mirror_bone) + " is out of range.");
			mirror_bone = -1;
		} else {
			mirror_bone_name = skeleton->get_bone_name(mirror_bone);
		}
	}

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

// bone의 이번 프레임 pose에 offset을 더한 위치와 회전(skeleton 공간). offset은 bone 방향 기준, m 단위다.
// p_units로 offset을 skeleton 단위로 바꾸고, 모양의 크기도 같은 배율로 커지도록 scale을 넣는다(예: cm rig는 100배).
static Transform3D get_pose_on_bone(const Skeleton3D *p_skeleton, int p_bone, const Vector3 &p_position_offset, const Quaternion &p_rotation_offset, float p_units) {
	Transform3D result = p_skeleton->get_bone_global_pose(p_bone);
	result.origin += result.basis.get_rotation_quaternion().xform(p_position_offset * p_units);
	result.basis *= Basis(p_rotation_offset) * p_units;
	return result;
}

// 노드를 bone 위로 옮긴다(이번 프레임 bone pose + offset).
// 아래 mirror 부분: offset을 skeleton 공간에서 좌우(X)로 뒤집어 반대쪽 bone에 놓는다(예: X +0.05 -> -0.05).
// 양쪽 bone 축이 대칭이 아닌 리그에서도 맞도록, rest 자세에서 뒤집는다.
void BoneClothCollision3D::sync_pose() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}

	const Transform3D skeleton_global = skeleton->get_global_transform();
	if (bone >= 0) {
		units_per_meter = Object::cast_to<BoneClothSimulator3D>(get_parent())->get_units_per_meter();
		pose = get_pose_on_bone(skeleton, bone, position_offset, rotation_offset, units_per_meter);
		syncing = true;
		set_global_transform(skeleton_global * pose);
		syncing = false;
	} else {
		units_per_meter = 1.0f;
		pose = skeleton_global.affine_inverse() * get_global_transform();
	}

	if (!has_mirror()) {
		return;
	}
	const Quaternion rest = skeleton->get_bone_global_rest(bone).basis.get_rotation_quaternion();
	const Quaternion mirror_rest = skeleton->get_bone_global_rest(mirror_bone).basis.get_rotation_quaternion();
	const Vector3 position = rest.xform(position_offset);
	const Quaternion rotation = rest * rotation_offset;
	const Vector3 mirrored_position(-position.x, position.y, position.z);
	const Quaternion mirrored_rotation(rotation.x, -rotation.y, -rotation.z, rotation.w);
	mirror_pose = get_pose_on_bone(skeleton, mirror_bone, mirror_rest.inverse().xform(mirrored_position), mirror_rest.inverse() * mirrored_rotation, units_per_meter);
	update_gizmos();
}

// 에디터에서 노드를 옮기거나 돌리면 그 자리를 offset으로 저장한다(undo 포함, scale은 버린다).
// 그래서 애니메이션이 돌아도 옮긴 자리에 붙어 있다. 씬을 불러오는 중에는 저장된 offset을 그대로 쓴다.
// 지금과 같은 위치가 들어오면 다시 계산하지 않는다(에디터는 안 움직여도 이동을 확정해서, 다시 계산하면 float 오차가 저장된다).
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
	if (moved.is_equal_approx(pose)) {
		return;
	}

	position_offset = bone_pose.basis.get_rotation_quaternion().inverse().xform(moved.origin - bone_pose.origin) / units_per_meter;
	rotation_offset = (bone_pose.basis.inverse() * moved.basis).get_rotation_quaternion();
	sync_pose();
}

// simulator가 호출할 충돌 함수. p_mirror면 mirror 쪽으로 충돌한다.
// 모양 함수는 크기 값(m)을 그대로 쓴다.
// 그래서 공간 전체를 m 단위로 줄여서(1 / units_per_meter배) 계산하고, 결과를 되돌린다.
Vector3 BoneClothCollision3D::collide(const Vector3 &p_point, float p_radius, bool p_mirror) const {
	const float meters = 1.0f / units_per_meter;
	const Transform3D &shape = p_mirror ? mirror_pose : pose;
	return _collide(Transform3D(shape.basis * meters, shape.origin * meters), p_point * meters, p_radius * meters) * units_per_meter;
}

void BoneClothCollision3D::collide_segment(Vector3 &r_a, Vector3 &r_b, float p_radius, bool p_mirror) const {
	const float meters = 1.0f / units_per_meter;
	const Transform3D &shape = p_mirror ? mirror_pose : pose;
	r_a *= meters;
	r_b *= meters;
	_collide_segment(Transform3D(shape.basis * meters, shape.origin * meters), r_a, r_b, p_radius * meters);
	r_a *= units_per_meter;
	r_b *= units_per_meter;
}

// base class는 모양이 없어서 아무것도 밀지 않는다(구, 캡슐, 평면이 이 함수를 다시 쓴다).
// 에디터의 노드 추가 창에 모양 class만 나오도록 virtual class로 등록한다.
Vector3 BoneClothCollision3D::_collide(const Transform3D &p_pose, const Vector3 &p_point, float p_radius) const {
	return p_point;
}

void BoneClothCollision3D::_collide_segment(const Transform3D &p_pose, Vector3 &r_a, Vector3 &r_b, float p_radius) const {
}

// 선분(r_a ~ r_b, link 선)을 모양 밖으로 민다. 두 joint 사이로 다리가 빠져나가지 않게 한다.
// p_on_segment: 선분에서 모양에 가장 가까운 점. p_on_shape: 그 점에 가장 가까운 모양의 중심(또는 축 위 점). 두 점이 p_limit보다 가까우면 밀어낸다.
// 민 양은 가까운 joint에 더 많이 나눈다(s: 선분 위 위치, r_a 0 ~ r_b 1).
void BoneClothCollision3D::_push_segment_out(Vector3 &r_a, Vector3 &r_b, const Vector3 &p_on_segment, const Vector3 &p_on_shape, float p_limit, const Vector3 &p_fallback) {
	const Vector3 offset = p_on_segment - p_on_shape;
	const float distance_squared = offset.length_squared();
	if (distance_squared >= p_limit * p_limit) {
		return;
	}

	const float distance = Math::sqrt(distance_squared);
	const Vector3 direction = distance > CMP_EPSILON ? offset / distance : p_fallback;
	const Vector3 push = direction * (p_limit - distance);

	const Vector3 line = r_b - r_a;
	const float line_length_squared = line.length_squared();
	const float s = line_length_squared > 0.0f ? (p_on_segment - r_a).dot(line) / line_length_squared : 0.0f;
	const float scale = 1.0f / ((1.0f - s) * (1.0f - s) + s * s);
	r_a += push * ((1.0f - s) * scale);
	r_b += push * (s * scale);
}

BoneClothCollision3D::BoneClothCollision3D() {
	set_notify_local_transform(true);
}

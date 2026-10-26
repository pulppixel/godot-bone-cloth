extends Node

const LEGS := [
	[&"LeftUpperLeg", &"LeftLowerLeg", &"LeftFoot", &"LeftToes"],
	[&"RightUpperLeg", &"RightLowerLeg", &"RightFoot", &"RightToes"],
]
const REACH := 0.35
const FOOT_SPRING := 31.6
const HIP_SPRING := 18.7
const HIP_DROP := 0.35
const HIP_RISE := 0.15

var _skeleton: Skeleton3D
var _body: CharacterBody3D
var _hips := -1
var _bones := []
var _targets: Array[Node3D] = []
var _poles: Array[Node3D] = []
var _foot: Array[Vector2] = [Vector2.ZERO, Vector2.ZERO]
var _hip := Vector2.ZERO
var _foot_basis: Array[Basis] = [Basis(), Basis()]


class FootGround extends SkeletonModifier3D:
	var feet: Node

	func _process_modification_with_delta(delta: float) -> void:
		feet.call(&"_ground", delta)


class FootRestore extends SkeletonModifier3D:
	var feet: Node

	func _process_modification_with_delta(_delta: float) -> void:
		feet.call(&"_restore")


func _ready() -> void:
	var parent := get_parent()
	_skeleton = parent.find_children("*", "Skeleton3D", true, false)[0]
	var node := parent
	while node != null and node is not CharacterBody3D:
		node = node.get_parent()
	_body = node
	_hips = _skeleton.find_bone(&"Hips")
	for leg in LEGS:
		_bones.append(leg.map(func(bone: StringName) -> int: return _skeleton.find_bone(bone)))
		var target := Node3D.new()
		var pole := Node3D.new()
		add_child(target)
		add_child(pole)
		_targets.append(target)
		_poles.append(pole)

	var ground := FootGround.new()
	ground.name = "FootGround"
	ground.feet = self
	var ik := TwoBoneIK3D.new()
	ik.name = "FootIK"
	var restore := FootRestore.new()
	restore.name = "FootRestore"
	restore.feet = self
	for modifier in [ground, ik, restore]:
		_skeleton.add_child(modifier)
	ik.setting_count = LEGS.size()
	for i in LEGS.size():
		ik.set_root_bone_name(i, LEGS[i][0])
		ik.set_middle_bone_name(i, LEGS[i][1])
		ik.set_end_bone_name(i, LEGS[i][2])
		ik.set_target_node(i, ik.get_path_to(_targets[i]))
		ik.set_pole_node(i, ik.get_path_to(_poles[i]))
	_skeleton.move_child(ground, 0)
	_skeleton.move_child(ik, 1)
	_skeleton.move_child(restore, 2)


func _ground(delta: float) -> void:
	var space := _skeleton.global_transform
	var floor_y := space.origin.y
	var grounded := _body.is_on_floor()
	var lift := [0.0, 0.0]
	for i in LEGS.size():
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var toes := space * _skeleton.get_bone_global_pose(_bones[i][3]).origin
		if grounded:
			lift[i] = maxf(_floor_under(ankle, floor_y), _floor_under(toes, floor_y)) - floor_y
		_foot[i] = _spring(_foot[i], lift[i], FOOT_SPRING, delta)

	var hip_target := clampf(minf(_foot[0].x, _foot[1].x), -HIP_DROP, HIP_RISE)
	_hip = _spring(_hip, hip_target, HIP_SPRING, delta)

	for i in LEGS.size():
		var hip := space * _skeleton.get_bone_global_pose(_bones[i][0]).origin
		var knee := space * _skeleton.get_bone_global_pose(_bones[i][1]).origin
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var toes := space * _skeleton.get_bone_global_pose(_bones[i][3]).origin
		_targets[i].global_position = ankle + Vector3.UP * _foot[i].x
		var line := (ankle - hip).normalized()
		var out := knee - hip
		out -= line * out.dot(line)
		if out.length() < 0.02:
			out = toes - ankle
			out -= line * out.dot(line)
		_poles[i].global_position = knee + Vector3.UP * _hip.x + out.normalized() * 0.5
		_foot_basis[i] = _skeleton.get_bone_global_pose(_bones[i][2]).basis

	var pose := _skeleton.get_bone_pose_position(_hips)
	var parent := _skeleton.get_bone_parent(_hips)
	var up := space.basis.inverse() * Vector3.UP * _hip.x
	if parent >= 0:
		up = _skeleton.get_bone_global_pose(parent).basis.inverse() * up
	_skeleton.set_bone_pose_position(_hips, pose + up)


func _restore() -> void:
	for i in LEGS.size():
		var foot: int = _bones[i][2]
		var parent := _skeleton.get_bone_parent(foot)
		var local: Basis = _skeleton.get_bone_global_pose(parent).basis.inverse() * _foot_basis[i]
		_skeleton.set_bone_pose_rotation(foot, local.get_rotation_quaternion())


func _floor_under(point: Vector3, floor_y: float) -> float:
	var from := Vector3(point.x, floor_y + REACH, point.z)
	var to := Vector3(point.x, floor_y - REACH, point.z)
	var query := PhysicsRayQueryParameters3D.create(from, to, _body.collision_mask, [_body.get_rid()])
	var hit := _skeleton.get_world_3d().direct_space_state.intersect_ray(query)
	if hit.is_empty() or hit.normal.y < 0.7:
		return floor_y
	return hit.position.y


func _spring(state: Vector2, target: float, w: float, delta: float) -> Vector2:
	var x := state.x - target
	var e := exp(-w * delta)
	var k := (state.y + w * x) * delta
	return Vector2((x + k) * e + target, (state.y - w * k) * e)
